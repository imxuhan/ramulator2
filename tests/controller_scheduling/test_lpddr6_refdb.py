import ramulator

from tests.controller_scheduling.test_lpddr6_bl24 import make_lpddr6_dut


def refdb_key(dut, command):
    bg_level = dut.level_names.index("BankGroup")
    bank_level = dut.level_names.index("Bank")
    return {
        (command.addr_vec[bg_level], command.addr_vec[bank_level]),
        (command.secondary_addr_vec[bg_level], command.secondary_addr_vec[bank_level]),
    }


def test_standard_refdb_manager_covers_all_banks_and_uses_long_round_gap():
    dut = make_lpddr6_dut(
        refresh_manager=ramulator.refresh_manager.LPDDR6DualBank(refresh_multiplier=0.125),
        wck_sync_mode="always_on",
    )

    refs = []
    for _ in range(5000):
        refs.extend(item for item in dut.tick() if item.command == "REFdb")
        if len(refs) >= 9:
            break

    assert len(refs) >= 9
    covered = set().union(*(refdb_key(dut, command) for command in refs[:8]))
    assert covered == {(bankgroup, bank) for bankgroup in range(4) for bank in range(4)}

    short = dut.timing("ndbR2dbR_S")
    long = dut.timing("ndbR2dbR_L")
    for previous, current in zip(refs[:7], refs[1:8]):
        assert current.clk - previous.clk >= short
    assert refs[8].clk - refs[7].clk >= long

    stats = dut.stats()
    assert stats["refdb_issued"] >= 9
    assert stats["refdb_rounds_completed"] >= 1


def test_refdb_rate_scales_eightfold_at_point_125x():
    def count_refs(multiplier):
        dut = make_lpddr6_dut(
            refresh_manager=ramulator.refresh_manager.LPDDR6DualBank(
                refresh_multiplier=multiplier
            ),
            wck_sync_mode="always_on",
        )
        refs = 0
        for _ in range(20_000):
            refs += sum(item.command == "REFdb" for item in dut.tick())
        return refs

    one_x = count_refs(1.0)
    point_125x = count_refs(0.125)

    assert one_x > 0
    assert 7.5 <= point_125x / one_x <= 8.5
