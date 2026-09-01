import pytest

import ramulator
from tests.controller_scheduling.test_lpddr6_bl24 import make_lpddr6_dut


pytestmark = pytest.mark.controller_scheduling


def make_lethe_dut(*, reduced_ba=0, refresh=True, dram_kwargs=None):
    manager = (
        ramulator.refresh_manager.LPDDR6LetheM2(reduced_ba=reduced_ba)
        if refresh
        else ramulator.refresh_manager.NoRefresh()
    )
    return make_lpddr6_dut(
        refresh_manager=manager,
        refdb_mode="lethe",
        wck_sync_mode="always_on",
        dram_kwargs=dram_kwargs,
    )


def refdb_pair(dut, ba, pair):
    bankgroups = (0, 1) if pair == 0 else (2, 3)
    return tuple(
        dut.addr_vec(Rank=0, BankGroup=bg, Bank=ba, Row=0, Column=0)
        for bg in bankgroups
    )


def issue_pair(dut, ba, pair):
    dut.priority_send_pair("REFdb", *refdb_pair(dut, ba, pair))
    history = dut.run_until_idle(max_ticks=2048)
    refs = [item for item in history if item.command == "REFdb"]
    assert len(refs) == 1
    return refs[0]


def ba_counts(stats):
    return [stats[f"refdb_ba_{ba}_issued"] for ba in range(4)]


def test_lethe_m2_configuration_rejects_out_of_scope_combinations():
    with pytest.raises(RuntimeError, match="reduced_ba must be in"):
        make_lethe_dut(reduced_ba=4)

    with pytest.raises(RuntimeError, match="requires refdb_mode='lethe'"):
        make_lpddr6_dut(
            refresh_manager=ramulator.refresh_manager.LPDDR6LetheM2(),
            refdb_mode="standard",
        )

    with pytest.raises(RuntimeError, match="requires refdb_mode='standard'"):
        make_lpddr6_dut(
            refresh_manager=ramulator.refresh_manager.LPDDR6DualBank(
                refresh_multiplier=0.125
            ),
            refdb_mode="lethe",
        )

    with pytest.raises(RuntimeError, match="16 Gb nRFCdb"):
        make_lethe_dut(dram_kwargs={"nRFCdb": 560})


def test_lethe_ba_coverage_advances_independently():
    dut = make_lethe_dut(refresh=False)

    issue_pair(dut, ba=0, pair=0)
    issue_pair(dut, ba=1, pair=0)
    issue_pair(dut, ba=0, pair=1)

    stats = dut.stats()
    assert ba_counts(stats) == [2, 1, 0, 0]
    assert stats["refdb_ba_0_rounds_completed"] == 1
    assert stats["refdb_ba_0_row"] == 1
    assert stats["refdb_ba_1_rounds_completed"] == 0
    assert stats["refdb_ba_1_row"] == 0
    assert stats["refdb_rounds_completed"] == 0


def test_lethe_rejects_duplicate_bank_before_ba_round_completion():
    dut = make_lethe_dut(refresh=False)
    issue_pair(dut, ba=0, pair=0)

    dut.priority_send_pair("REFdb", *refdb_pair(dut, ba=0, pair=0))
    dut.tick_many(dut.timing("ndbR2dbR_L") * 2)

    stats = dut.stats()
    assert stats["refdb_issued"] == 1
    assert stats["refdb_ba_0_issued"] == 1
    assert stats["refdb_ba_0_rounds_completed"] == 0


def test_lethe_uses_short_cross_ba_and_long_same_ba_round_gaps():
    same_ba = make_lethe_dut(refresh=False)
    first = issue_pair(same_ba, ba=0, pair=0)
    second = issue_pair(same_ba, ba=0, pair=1)
    third = issue_pair(same_ba, ba=0, pair=0)

    assert second.clk - first.clk == same_ba.timing("ndbR2dbR_S")
    assert third.clk - second.clk == same_ba.timing("ndbR2dbR_L")

    cross_ba = make_lethe_dut(refresh=False)
    issue_pair(cross_ba, ba=0, pair=0)
    completed = issue_pair(cross_ba, ba=0, pair=1)
    different = issue_pair(cross_ba, ba=1, pair=0)

    assert different.clk - completed.clk == cross_ba.timing("ndbR2dbR_S")


@pytest.mark.parametrize("reduced_ba", [0, 3])
def test_lethe_manager_rotates_all_bas_and_skips_three_of_four_reduced_turns(
    reduced_ba,
):
    dut = make_lethe_dut(reduced_ba=reduced_ba)
    slot_interval = round(dut.timing("nREFIdb") * 0.125)
    dut.tick_many(slot_interval * 16)

    stats = dut.stats()
    expected = [4, 4, 4, 4]
    expected[reduced_ba] = 1
    assert ba_counts(stats) == expected
    assert stats["refresh_manager"]["slots"] == 16
    assert stats["refresh_manager"]["skipped_reduced_ba_slots"] == 3


def test_full_refresh_window_is_81_25_percent_and_per_bank_quarter_rate():
    reduced_ba = 0
    baseline = make_lpddr6_dut(
        refresh_manager=ramulator.refresh_manager.LPDDR6DualBank(
            refresh_multiplier=0.125
        ),
        refdb_mode="standard",
        wck_sync_mode="always_on",
    )
    lethe = make_lethe_dut(reduced_ba=reduced_ba)

    slot_interval = round(baseline.timing("nREFIdb") * 0.125)
    standard_commands_per_window = 8192 * 8
    window_cycles = slot_interval * standard_commands_per_window
    baseline.tick_many(window_cycles)
    lethe.tick_many(window_cycles)

    baseline_stats = baseline.stats()
    lethe_stats = lethe.stats()

    assert baseline_stats["refdb_issued"] == 65536
    assert baseline_stats["refdb_rounds_completed"] == 8192
    assert baseline_stats["refdb_shared_row"] == 0
    assert ba_counts(baseline_stats) == [16384] * 4
    assert [baseline_stats[f"refdb_bank_{bank}"] for bank in range(16)] == [
        8192
    ] * 16

    assert lethe_stats["refdb_issued"] == 53248
    assert lethe_stats["refdb_issued"] / baseline_stats["refdb_issued"] == 0.8125
    assert ba_counts(lethe_stats) == [4096, 16384, 16384, 16384]

    for ba in range(4):
        expected_rounds = 2048 if ba == reduced_ba else 8192
        expected_row = 2048 if ba == reduced_ba else 0
        assert lethe_stats[f"refdb_ba_{ba}_rounds_completed"] == expected_rounds
        assert lethe_stats[f"refdb_ba_{ba}_row"] == expected_row
        for bg in range(4):
            flat_bank = bg * 4 + ba
            assert lethe_stats[f"refdb_bank_{flat_bank}"] == expected_rounds

    assert lethe_stats["refresh_manager"]["slots"] == 65536
    assert lethe_stats["refresh_manager"]["skipped_reduced_ba_slots"] == 12288
