from tests.controller_scheduling.test_champsim_m3 import write_traces

import ramulator


def make_simulation(traces, *, mode, tolerant_bas, warmup=16, roi=64, sidecars=None):
    translation = ramulator.translation.FirstTouchPageColoringM4(
        max_addr=2 * 1024**3,
        page_size=4096,
        ba_bit_offset=13,
        tolerant_bas=tolerant_bas,
        num_cores=4,
        tolerant_cores=[0, 1],
    )
    frontend_args = dict(
        clock_ratio=4,
        warmup_insts=warmup,
        num_expected_insts=roi,
        traces=traces,
        ipc=4,
        inst_window_depth=128,
        llc_capacity_per_core="64KB",
        translation=translation,
    )
    if sidecars is not None:
        frontend_args["object_sidecars"] = sidecars
    frontend = ramulator.frontend.ChampSimO3(**frontend_args)
    dram = ramulator.dram.LPDDR6(
        org_preset="LPDDR6_16Gb_x12",
        timing_preset="LPDDR6_10667_BL24",
        rank=1,
    )
    if mode == "standard":
        manager = ramulator.refresh_manager.LPDDR6DualBankM4(
            refresh_multiplier=0.125
        )
        refdb_mode = "standard"
    else:
        manager = ramulator.refresh_manager.LPDDR6LetheM4(
            refresh_multiplier=0.125,
            ba_ratios=[0.25, 0.25, 1, 1],
        )
        refdb_mode = "lethe"
    controller = ramulator.controller.LPDDR6(
        dram=dram,
        scheduler=ramulator.scheduler.FRFCFS(),
        refresh_manager=manager,
        row_policy=ramulator.row_policy.Open(),
        addr_mapper=ramulator.addr_mapper.RoBaRaCoCh(),
        tolerant_cores=[0, 1],
        wck_sync_mode="always_on",
        refdb_mode=refdb_mode,
    )
    memory = ramulator.memory_system.GenericDRAM(
        clock_ratio=3,
        controllers=[controller],
        channel_mapper=ramulator.channel_mapper.CacheLineInterleave(),
    )
    return ramulator.Simulation(frontend, memory)


def test_m4_warmup_barrier_drains_and_records_only_roi(tmp_path):
    sim = make_simulation(
        write_traces(tmp_path, records=8), mode="standard", tolerant_bas=[0]
    )
    sim.run()
    frontend = sim.stats["frontend"]

    assert frontend["warmup_insts"] == 16
    assert frontend["warmup_cycles"] > 0
    assert all(value == 64 for value in frontend["insts_retired_per_core"])
    assert all(value > 0 for value in frontend["cycles_recorded_per_core"])


def test_m4_two_ba_coloring_keeps_page_classes_in_their_legal_bas(tmp_path):
    traces = write_traces(tmp_path, records=16)
    baseline = make_simulation(traces, mode="standard", tolerant_bas=[0, 1])
    lethe = make_simulation(traces, mode="lethe", tolerant_bas=[0, 1])
    baseline.run()
    lethe.run()

    first = baseline.stats["frontend"]["translation"]
    second = lethe.stats["frontend"]["translation"]
    assert first["pages_borrowed"] == second["pages_borrowed"] == 0
    for stats in (first, second):
        assert stats["pages_tolerant"] > 0
        assert stats["pages_reliable"] > 0
        assert stats["reliable_pages_ba_0"] == 0
        assert stats["reliable_pages_ba_1"] == 0
        assert stats["tolerant_pages_ba_2"] == 0
        assert stats["tolerant_pages_ba_3"] == 0


def test_object_sidecar_overrides_whole_core_class_and_replays_on_loop(tmp_path):
    traces = write_traces(tmp_path, records=8)
    reliable = tmp_path / "core0.sidecar"
    reliable.write_text(
        "H\t1\tchampsim-input-instr-64\tglobal-record-order\treliable-default\n"
        "B\t0\t0\n"
        "E\t8\twindow_complete\t0\t1\n"
    )
    tolerant = tmp_path / "core1.sidecar"
    tolerant.write_text(
        "H\t1\tchampsim-input-instr-64\tglobal-record-order\treliable-default\n"
        "B\t0\t0\n"
        "A\t0\t0\t1\t0x20000000\t32768\ttolerant\tactivation\n"
        "E\t8\twindow_complete\t0\t2\n"
    )
    sim = make_simulation(
        traces,
        mode="standard",
        tolerant_bas=[0],
        warmup=0,
        roi=64,
        sidecars=[str(reliable), str(tolerant), "", ""],
    )
    sim.run()
    coloring = sim.stats["frontend"]["translation"]
    controller = sim.stats["memory_system"]["controller"]

    assert sim.stats["frontend"]["trace_passes_completed_per_core"][1] > 0
    assert coloring["page_class_conflicts"] == 0
    assert coloring["pages_tolerant"] == 8
    assert coloring["tolerant_pages_ba_0"] == 8
    assert coloring["pages_reliable"] > coloring["pages_tolerant"]
    assert controller["tolerant_demand_reads_served"] > 0
    assert controller["reliable_demand_reads_served"] > 0
