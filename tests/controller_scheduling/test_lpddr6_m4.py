import pytest

import ramulator
import tests.controller_scheduling.harness as cs


pytestmark = pytest.mark.controller_scheduling


def make_dut(*, manager, mode, density=16):
    dram = ramulator.dram.LPDDR6(
        org_preset=f"LPDDR6_{density}Gb_x12",
        timing_preset="LPDDR6_10667_BL24",
        rank=1,
    )
    controller = ramulator.controller.LPDDR6(
        dram=dram,
        scheduler=ramulator.scheduler.FRFCFS(),
        refresh_manager=manager,
        row_policy=ramulator.row_policy.Open(),
        addr_mapper=ramulator.addr_mapper.PassThroughAddrMapper(),
        refdb_mode=mode,
        wck_sync_mode="always_on",
    )
    return cs.ControllerUnderTest(controller)


def ba_counts(stats):
    return [stats[f"refdb_ba_{ba}_issued"] for ba in range(4)]


@pytest.mark.parametrize(
    ("ratios", "expected"),
    [
        ([0.5, 1, 1, 1], [8, 16, 16, 16]),
        ([0.25, 1, 1, 1], [4, 16, 16, 16]),
        ([0.25, 0.25, 1, 1], [4, 4, 16, 16]),
        ([0, 1, 1, 1], [0, 16, 16, 16]),
    ],
)
def test_m4_round_robin_ratios_are_relative_to_mr4(ratios, expected):
    dut = make_dut(
        manager=ramulator.refresh_manager.LPDDR6LetheM4(
            refresh_multiplier=0.125,
            ba_ratios=ratios,
            schedule="round_robin",
        ),
        mode="lethe",
    )
    base_interval = round(dut.timing("nREFIdb") * 0.125)
    dut.tick_many(base_interval * 64)

    stats = dut.stats()
    assert ba_counts(stats) == expected
    assert stats["refdb_issued"] == sum(expected)
    assert stats["refresh_manager"]["slots"] == sum(expected)


@pytest.mark.parametrize("schedule", ["round_robin", "darp"])
def test_m4_standard_and_lethe_schedules_preserve_expected_idle_rates(schedule):
    baseline = make_dut(
        manager=ramulator.refresh_manager.LPDDR6DualBankM4(
            refresh_multiplier=0.125, schedule=schedule
        ),
        mode="standard",
    )
    lethe = make_dut(
        manager=ramulator.refresh_manager.LPDDR6LetheM4(
            refresh_multiplier=0.125,
            ba_ratios=[0.25, 1, 1, 1],
            schedule=schedule,
        ),
        mode="lethe",
    )
    base_interval = round(baseline.timing("nREFIdb") * 0.125)
    baseline.tick_many(base_interval * 64)
    lethe.tick_many(base_interval * 64)

    baseline_stats = baseline.stats()
    lethe_stats = lethe.stats()
    assert baseline_stats["refdb_issued"] == 64
    assert ba_counts(baseline_stats) == [16, 16, 16, 16]
    assert lethe_stats["refdb_issued"] == 52
    assert ba_counts(lethe_stats) == [4, 16, 16, 16]
    assert lethe_stats["refdb_issued"] / baseline_stats["refdb_issued"] == 0.8125


def test_m4_32gb_uses_table_302_210ns_refdb_timing():
    dut = make_dut(
        manager=ramulator.refresh_manager.LPDDR6DualBankM4(),
        mode="standard",
        density=32,
    )
    assert dut.org["row"] == 1 << 17
    assert dut.timing("nRFCdb") == 560


def test_m4_rejects_invalid_ratios_schedules_and_mode_pairings():
    with pytest.raises(RuntimeError, match="exactly four"):
        make_dut(
            manager=ramulator.refresh_manager.LPDDR6LetheM4(ba_ratios=[0.25]),
            mode="lethe",
        )
    with pytest.raises(RuntimeError, match="ratios must be"):
        make_dut(
            manager=ramulator.refresh_manager.LPDDR6LetheM4(
                ba_ratios=[0.75, 1, 1, 1]
            ),
            mode="lethe",
        )
    with pytest.raises(RuntimeError, match="schedule must be"):
        make_dut(
            manager=ramulator.refresh_manager.LPDDR6DualBankM4(schedule="unknown"),
            mode="standard",
        )
    with pytest.raises(RuntimeError, match="requires refdb_mode='lethe'"):
        make_dut(
            manager=ramulator.refresh_manager.LPDDR6LetheM4(
                ba_ratios=[0.25, 1, 1, 1]
            ),
            mode="standard",
        )
