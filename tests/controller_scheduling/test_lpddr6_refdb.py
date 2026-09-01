import ramulator

from tests.controller_scheduling.test_lpddr6_bl24 import (
    lpddr6_addr,
    make_lpddr6_dut,
    open_row,
)


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


def test_refdb_consumes_two_tfaw_activation_credits():
    dut = make_lpddr6_dut(wck_sync_mode="always_on")
    first = dut.addr_vec(Rank=0, BankGroup=0, Bank=0, Row=0, Column=0)
    second = dut.addr_vec(Rank=0, BankGroup=1, Bank=0, Row=0, Column=0)
    dut.priority_send_pair("REFdb", first, second)
    ref_history = dut.run_until_idle(max_ticks=256)
    ref = next(item for item in ref_history if item.command == "REFdb")

    for bankgroup, bank in ((2, 0), (3, 0), (2, 1)):
        target = dut.addr_vec(
            Rank=0, BankGroup=bankgroup, Bank=bank, Row=0, Column=0
        )
        dut.send_request("Read", target)

    history = dut.run_until_idle(max_ticks=2048)
    activations = [item for item in history if item.command == "ACT2"]

    assert len(activations) == 3
    assert activations[1].clk - ref.clk < dut.timing("nFAW")
    assert activations[2].clk - ref.clk >= dut.timing("nFAW")


def test_refdb_manager_precharges_only_its_two_open_target_banks():
    dut = make_lpddr6_dut(
        refresh_manager=ramulator.refresh_manager.LPDDR6DualBank(),
        wck_sync_mode="always_on",
    )
    open_row(dut, lpddr6_addr(dut, bankgroup=0, bank=0))
    open_row(dut, lpddr6_addr(dut, bankgroup=1, bank=0))
    start = len(dut.history)

    for _ in range(2000):
        issued = dut.tick()
        if any(item.command == "REFdb" for item in issued):
            break

    maintenance = [
        item.command
        for item in dut.history[start:]
        if item.command in {"PREpb", "PREab", "REFdb"}
    ]
    assert maintenance[:3] == ["PREpb", "PREpb", "REFdb"]
    assert "PREab" not in maintenance
