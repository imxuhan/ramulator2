import pytest

import ramulator
import tests.device_timings.harness as device_timings


pytestmark = pytest.mark.device_timings


def make_dut():
    dram = ramulator.dram.LPDDR6(
        org_preset="LPDDR6_16Gb_x12",
        timing_preset="LPDDR6_10667_BL24",
    )
    return device_timings.DeviceUnderTest(dram)


def addr(dut, bankgroup, bank):
    return dut.addr_vec(Rank=0, BankGroup=bankgroup, Bank=bank, Row=0, Column=0)


def test_refdb_targets_exactly_two_different_bankgroups_with_same_ba():
    dut = make_dut()
    first = addr(dut, 0, 2)
    second = addr(dut, 3, 2)

    probe = dut.probe_pair("REFdb", first, second, clk=0)

    assert probe.ready
    assert probe.target_banks == [2, 14]


def test_refdb_rejects_same_bankgroup_or_different_ba():
    dut = make_dut()

    with pytest.raises(RuntimeError, match="different bank groups"):
        dut.probe_pair("REFdb", addr(dut, 0, 1), addr(dut, 0, 1), clk=0)

    with pytest.raises(RuntimeError, match="same channel, rank, and bank"):
        dut.probe_pair("REFdb", addr(dut, 0, 1), addr(dut, 1, 2), clk=0)


def test_refdb_blocks_only_two_target_banks_for_nrfcdb():
    dut = make_dut()
    first = addr(dut, 0, 2)
    second = addr(dut, 3, 2)
    other = addr(dut, 1, 2)
    nrfcdb = dut.timings["nRFCdb"]

    dut.issue_pair("REFdb", first, second, clk=0)

    assert not dut.probe("ACT2", first, clk=nrfcdb - 1).timing_OK
    assert not dut.probe("ACT2", second, clk=nrfcdb - 1).timing_OK
    assert dut.probe("ACT2", first, clk=nrfcdb).timing_OK
    assert dut.probe("ACT2", second, clk=nrfcdb).timing_OK
    # REFdb occupies the command bus for 2 CK, but must not impose nRFCdb on
    # any of the other 14 banks.
    assert dut.probe("ACT2", other, clk=2).timing_OK


def test_refdb_short_gap_is_tight():
    dut = make_dut()
    first = (addr(dut, 0, 0), addr(dut, 1, 0))
    second = (addr(dut, 2, 0), addr(dut, 3, 0))
    short = dut.timings["ndbR2dbR_S"]

    dut.issue_pair("REFdb", *first, clk=0)

    assert not dut.probe_pair("REFdb", *second, clk=short - 1).timing_OK
    assert dut.probe_pair("REFdb", *second, clk=short).timing_OK


def test_lpddr6_refdb_table_302_timings():
    dut = make_dut()

    assert dut.timings["nRFCdb"] == 427  # ceil(160 ns / 0.375 ns)
    assert dut.timings["ndbR2dbR_S"] == 126  # ceil(47 ns / 0.375 ns)
    assert dut.timings["ndbR2dbR_L"] == 240  # 90 ns / 0.375 ns
    assert ramulator.dram.LPDDR6._resolve_nRFCdb(32_768, 375) == 560
