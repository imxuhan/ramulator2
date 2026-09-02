import gzip
import lzma

from tests.controller_scheduling.test_champsim_m3 import TRACE_RECORD, trace_bytes, write_traces

import ramulator
from ramulator._ramulator_test import _ChannelMapperUnderTest


def write_same_virtual_traces(tmp_path, *, records=8):
    encoded = bytearray()
    for index in range(records):
        page = 0x20000000 + index * 4096
        encoded.extend(
            TRACE_RECORD.pack(
                0x400000 + index, 0, 0, 1, 0, 2, 3, 0, 0,
                page + 192, 0, page + 64, page + 128, 0, 0,
            )
        )
    paths = []
    for core_id, suffix in enumerate((".xz", ".gz", ".bin", ".xz")):
        path = tmp_path / f"same{core_id}.champsimtrace{suffix}"
        payload = bytes(encoded)
        if suffix == ".xz":
            path.write_bytes(lzma.compress(payload))
        elif suffix == ".gz":
            path.write_bytes(gzip.compress(payload))
        else:
            path.write_bytes(payload)
        paths.append(str(path))
    return paths


def write_channel_spread_traces(tmp_path, *, records=24):
    paths = []
    for core_id, suffix in enumerate((".xz", ".gz", ".bin", ".xz")):
        encoded = bytearray()
        core_base = (core_id + 1) << 28
        for index in range(records):
            page = core_base + index * 4096
            offsets = [((index + step) % 4) * 64 for step in range(3)]
            encoded.extend(
                TRACE_RECORD.pack(
                    0x500000 + index, 0, 0, 1, 0, 2, 3, 0, 0,
                    page + offsets[0], 0, page + offsets[1], page + offsets[2], 0, 0,
                )
            )
        path = tmp_path / f"spread{core_id}.champsimtrace{suffix}"
        payload = bytes(encoded)
        if suffix == ".xz":
            path.write_bytes(lzma.compress(payload))
        elif suffix == ".gz":
            path.write_bytes(gzip.compress(payload))
        else:
            path.write_bytes(payload)
        paths.append(str(path))
    return paths


def sparse_histogram(stats, prefix):
    return list(zip(stats[f"{prefix}_bins"], stats[f"{prefix}_counts"]))


def sparse_p99(histogram):
    count = sum(occurrences for _, occurrences in histogram)
    target = (count * 99 + 99) // 100
    cumulative = 0
    for latency, occurrences in histogram:
        cumulative += occurrences
        if cumulative >= target:
            return latency
    return 0


def make_simulation(
    traces, *, mode, tolerant_bas, warmup=16, roi=64, sidecars=None,
    channels=1, density=16, max_addr=None,
):
    max_addr = max_addr or channels * density * 1024**3 // 8
    translation = ramulator.translation.FirstTouchPageColoringM4(
        max_addr=max_addr,
        page_size=4096,
        ba_bit_offset=13,
        channels=channels,
        channel_interleave_size=64,
        seed=0,
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
    def make_controller():
        dram = ramulator.dram.LPDDR6(
            org_preset=f"LPDDR6_{density}Gb_x12",
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
        return ramulator.controller.LPDDR6(
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
        controllers=[make_controller() for _ in range(channels)],
        channel_mapper=ramulator.channel_mapper.CacheLineInterleave(interleave_bits=1),
        tolerant_cores=[0, 1],
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
    assert [first[f"pages_ba_{ba}"] for ba in range(4)] == [
        second[f"pages_ba_{ba}"] for ba in range(4)
    ]
    for stats in (first, second):
        assert stats["pages_tolerant"] > 0
        assert stats["pages_reliable"] > 0
        assert stats["reliable_pages_ba_0"] == 0
        assert stats["reliable_pages_ba_1"] == 0
        assert stats["tolerant_pages_ba_2"] == 0
        assert stats["tolerant_pages_ba_3"] == 0


def test_global_ba_pool_is_not_partitioned_by_core(tmp_path):
    sim = make_simulation(
        write_traces(tmp_path, records=10), mode="standard", tolerant_bas=[0, 1],
        warmup=0, roi=40, max_addr=64 * 4096,
    )
    sim.run()
    coloring = sim.stats["frontend"]["translation"]
    assert coloring["pages_tolerant"] == 20
    assert coloring["pages_reliable"] == 20
    assert coloring["pages_borrowed"] == 0
    assert coloring["unique_physical_pages"] == 40
    assert coloring["physical_page_aliases"] == 0


def test_page_table_keeps_core_virtual_page_identity_and_unique_physical_pages(tmp_path):
    sim = make_simulation(
        write_same_virtual_traces(tmp_path, records=8), mode="standard", tolerant_bas=[0, 1],
        warmup=0, roi=32, max_addr=64 * 4096,
    )
    sim.run()
    coloring = sim.stats["frontend"]["translation"]
    assert coloring["pages_tolerant"] == 16
    assert coloring["pages_reliable"] == 16
    assert coloring["unique_physical_pages"] == 32
    assert coloring["page_class_conflicts"] == 0


def test_cache_line_channel_interleave_is_bijective_and_keeps_line_halves_together():
    for channels in (1, 2, 4):
        mapper = _ChannelMapperUnderTest(
            {"impl": "CacheLineInterleave", "interleave_bits": 1}, channels, 5
        )
        mapped = [mapper.apply(addr) for addr in range(0, 4096, 32)]
        assert len({(item["channel"], item["intra_channel_addr"]) for item in mapped}) == len(mapped)
        for line in range(0, 4096, 64):
            assert mapper.apply(line)["channel"] == mapper.apply(line + 32)["channel"]


def test_multichannel_capacity_topology_and_pooled_p99(tmp_path):
    traces = write_channel_spread_traces(tmp_path, records=24)
    for channels, expected_bytes in ((1, 2 * 1024**3), (2, 8 * 1024**3), (4, 16 * 1024**3)):
        density = 16 if channels == 1 else 32
        sim = make_simulation(
            traces, mode="standard", tolerant_bas=[0, 1], warmup=0, roi=96,
            channels=channels, density=density,
        )
        sim.run()
        stats = sim.stats
        coloring = stats["frontend"]["translation"]
        memory = stats["memory_system"]
        controllers = memory["controller"] if isinstance(memory["controller"], list) else [memory["controller"]]
        assert coloring["allocator_max_addr"] == expected_bytes
        assert coloring["allocator_channels"] == channels
        assert coloring["pages_borrowed"] == 0
        assert coloring["physical_page_aliases"] == 0
        assert coloring["reliable_pages_ba_0"] == 0
        assert coloring["reliable_pages_ba_1"] == 0
        assert coloring["tolerant_pages_ba_2"] == 0
        assert coloring["tolerant_pages_ba_3"] == 0
        assert len(controllers) == channels
        assert all(
            controller["tolerant_demand_reads_served"] + controller["reliable_demand_reads_served"] > 0
            for controller in controllers
        )
        assert sparse_p99(sparse_histogram(memory, "system_tolerant_read_latency_histogram")) == memory["system_tolerant_p99_read_latency"]
        assert sparse_p99(sparse_histogram(memory, "system_reliable_read_latency_histogram")) == memory["system_reliable_p99_read_latency"]
        if channels == 2:
            class_name = "tolerant"
            channel_p99 = [controller[f"{class_name}_p99_read_latency"] for controller in controllers]
            pooled = memory[f"system_{class_name}_p99_read_latency"]
            assert pooled != sum(channel_p99) / len(channel_p99)


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
