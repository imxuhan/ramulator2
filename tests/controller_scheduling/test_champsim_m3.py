import gzip
import lzma
import struct

import pytest

import ramulator


pytestmark = pytest.mark.controller_scheduling

TRACE_RECORD = struct.Struct("<QBB2B4B2Q4Q")


def trace_bytes(core_id, records=8):
    encoded = bytearray()
    core_base = (core_id + 1) << 28
    for index in range(records):
        page = core_base + index * 4096
        encoded.extend(
            TRACE_RECORD.pack(
                0x400000 + index,
                0,
                0,
                1,
                0,
                2,
                3,
                0,
                0,
                page + 192,
                0,
                page + 64,
                page + 128,
                0,
                0,
            )
        )
    return bytes(encoded)


def write_traces(tmp_path, *, records=8):
    paths = []
    for core_id, suffix in enumerate((".xz", ".gz", ".bin", ".xz")):
        path = tmp_path / f"core{core_id}.champsimtrace{suffix}"
        payload = trace_bytes(core_id, records)
        if suffix == ".xz":
            path.write_bytes(lzma.compress(payload))
        elif suffix == ".gz":
            path.write_bytes(gzip.compress(payload))
        else:
            path.write_bytes(payload)
        paths.append(str(path))
    return paths


def make_simulation(traces, *, mode="standard", expected_insts=64):
    translation = ramulator.translation.FirstTouchPageColoring(
        max_addr=2 * 1024**3,
        page_size=4096,
        ba_bit_offset=13,
        reduced_ba=0,
        num_cores=4,
        tolerant_cores=[0, 1],
    )
    frontend = ramulator.frontend.ChampSimO3(
        clock_ratio=4,
        num_expected_insts=expected_insts,
        traces=traces,
        ipc=4,
        inst_window_depth=128,
        llc_capacity_per_core="64KB",
        translation=translation,
    )
    dram = ramulator.dram.LPDDR6(
        org_preset="LPDDR6_16Gb_x12",
        timing_preset="LPDDR6_10667_BL24",
        rank=1,
    )
    if mode == "standard":
        refresh_manager = ramulator.refresh_manager.LPDDR6DualBank(
            refresh_multiplier=0.125
        )
    else:
        refresh_manager = ramulator.refresh_manager.LPDDR6LetheM2(reduced_ba=0)
    controller = ramulator.controller.LPDDR6(
        dram=dram,
        scheduler=ramulator.scheduler.FRFCFS(),
        refresh_manager=refresh_manager,
        row_policy=ramulator.row_policy.Open(),
        addr_mapper=ramulator.addr_mapper.RoBaRaCoCh(),
        tolerant_cores=[0, 1],
        wck_sync_mode="always_on",
        refdb_mode=mode,
    )
    memory = ramulator.memory_system.GenericDRAM(
        clock_ratio=3,
        controllers=[controller],
        channel_mapper=ramulator.channel_mapper.CacheLineInterleave(),
    )
    return ramulator.Simulation(frontend, memory)


def test_champsim_frontend_streams_plain_xz_gzip_and_loops(tmp_path):
    sim = make_simulation(write_traces(tmp_path, records=4), expected_insts=40)
    sim.run()
    stats = sim.stats

    frontend = stats["frontend"]
    controller = stats["memory_system"]["controller"]
    assert frontend["trace_abi"] == "ChampSim input_instr 64-byte"
    assert all(value >= 40 for value in frontend["insts_retired_per_core"])
    assert all(value > 0 for value in frontend["trace_passes_completed_per_core"])
    assert frontend["llc_read_access"] > 0
    assert frontend["llc_write_access"] > 0
    assert all(controller[f"demand_reads_served_core_{core}"] > 0 for core in range(4))


def test_first_touch_page_coloring_keeps_reliable_pages_out_of_reduced_ba(tmp_path):
    sim = make_simulation(write_traces(tmp_path), expected_insts=48)
    sim.run()
    coloring = sim.stats["frontend"]["translation"]

    assert coloring["pages_tolerant"] > 0
    assert coloring["pages_reliable"] > 0
    assert coloring["pages_borrowed"] == 0
    assert coloring["pages_ba_0"] == coloring["pages_tolerant"]
    assert sum(coloring[f"pages_ba_{ba}"] for ba in (1, 2, 3)) == coloring[
        "pages_reliable"
    ]


@pytest.mark.parametrize("mode", ["standard", "lethe"])
def test_four_core_pilot_modes_report_latency_and_refresh_occupancy(tmp_path, mode):
    sim = make_simulation(write_traces(tmp_path), mode=mode, expected_insts=128)
    sim.run()
    controller = sim.stats["memory_system"]["controller"]

    assert controller["refdb_mode_lethe"] == (mode == "lethe")
    assert controller["refdb_issued"] > 0
    assert controller["refdb_bank_busy_cycles"] > 0
    assert all(controller[f"avg_read_latency_core_{core}"] > 0 for core in range(4))
    assert all(controller[f"p99_read_latency_core_{core}"] > 0 for core in range(4))
    assert controller["tolerant_avg_read_latency"] > 0
    assert controller["tolerant_p99_read_latency"] > 0
    assert controller["reliable_avg_read_latency"] > 0
    assert controller["reliable_p99_read_latency"] > 0


def test_champsim_frontend_rejects_truncated_record(tmp_path):
    paths = write_traces(tmp_path)
    truncated = tmp_path / "truncated.champsimtrace"
    truncated.write_bytes(b"not-64-bytes")
    paths[0] = str(truncated)

    with pytest.raises(RuntimeError, match="truncated"):
        make_simulation(paths)
