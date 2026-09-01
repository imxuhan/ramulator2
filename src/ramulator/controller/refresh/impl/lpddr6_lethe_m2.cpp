#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/refresh/i_refresh_manager.h"
#include "ramulator/dram/dram_spec.h"
#include "ramulator/dram/node.h"

namespace Ramulator {

// Minimal Lethe vertical slice: LPDDR6 16 Gb, MR4 0.125x, deterministic
// round-robin scheduling, and exactly one BA refreshed at one-quarter rate.
class LPDDR6LetheM2Refresh final : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, LPDDR6LetheM2Refresh, "LPDDR6LetheM2")

 private:
  static constexpr int kBaCount = 4;
  static constexpr int kReducedBaDivisor = 4;

  ControllerBase* m_ctrl = nullptr;
  int m_cmd_prepb = -1;
  int m_cmd_refdb = -1;
  int m_state_closed = -1;
  int m_rank_level = -1;
  int m_bankgroup_level = -1;
  int m_bank_level = -1;
  int m_reduced_ba = 0;
  int m_slot_interval = -1;
  int m_next_ba = 0;
  int m_reduced_ba_turn = 0;
  Clk_t m_next_due = -1;
  bool m_debug = false;

  std::array<int, kBaCount> m_next_pair{};
  size_t s_slots = 0;
  size_t s_skipped_reduced_ba_slots = 0;

  void init() override;
  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override;
  void reset_stats() override;
  void tick() override;

  AddrVec_t make_addr(int bankgroup, int bank) const;
  void enqueue_command(const AddrVec_t& addr_vec, int command);
  void enqueue_refdb(int ba);
};

void LPDDR6LetheM2Refresh::init() {
  m_ctrl = cast_parent<ControllerBase>();
  RAMULATOR_PARSE_PARAM(m_reduced_ba, int, "reduced_ba").default_val(0);
  RAMULATOR_PARSE_PARAM(m_debug, bool, "debug").default_val(false);

  if (m_reduced_ba < 0 || m_reduced_ba >= kBaCount) {
    throw std::runtime_error("LPDDR6LetheM2 reduced_ba must be in [0, 3]");
  }

  const auto& spec = *m_ctrl->m_device.m_spec;
  if (spec.standard_name != "LPDDR6") {
    throw std::runtime_error("LPDDR6LetheM2 refresh manager requires LPDDR6 DRAM");
  }
  if (spec.get_level_size("Rank") != 1 || spec.get_level_size("BankGroup") != 4 ||
      spec.get_level_size("Bank") != 4) {
    throw std::runtime_error("LPDDR6LetheM2 requires one rank with four bank groups and four banks");
  }

  const int tck_ps = spec.get_timing_value("tCK_ps");
  const int expected_nrfcdb_16gb = (160000 + tck_ps - 1) / tck_ps;
  if (spec.get_timing_value("nRFCdb") != expected_nrfcdb_16gb) {
    throw std::runtime_error("LPDDR6LetheM2 requires the 16 Gb nRFCdb timing");
  }

  m_cmd_prepb = spec.get_command_id("PREpb");
  m_cmd_refdb = spec.get_command_id("REFdb");
  m_state_closed = spec.get_state_id("Closed");
  m_rank_level = spec.get_level_id("Rank");
  m_bankgroup_level = spec.get_level_id("BankGroup");
  m_bank_level = spec.get_level_id("Bank");

  // MR4 0.125x: the standard REFdb slot interval is nREFIdb / 8. Use
  // integer round-to-nearest to match LPDDR6DualBank's llround behavior.
  const int nrefidb = spec.get_timing_value("nREFIdb");
  m_slot_interval = std::max(1, (nrefidb + 4) / 8);
  m_next_due = m_slot_interval;
}

void LPDDR6LetheM2Refresh::setup(IFrontEnd*, IMemorySystem*) {
  m_stats.add("slots", s_slots);
  m_stats.add("skipped_reduced_ba_slots", s_skipped_reduced_ba_slots);
}

void LPDDR6LetheM2Refresh::reset_stats() {
  s_slots = 0;
  s_skipped_reduced_ba_slots = 0;
}

AddrVec_t LPDDR6LetheM2Refresh::make_addr(int bankgroup, int bank) const {
  AddrVec_t addr_vec(m_ctrl->m_device.m_spec->level_count, -1);
  addr_vec[0] = m_ctrl->m_channel_id;
  addr_vec[m_rank_level] = 0;
  addr_vec[m_bankgroup_level] = bankgroup;
  addr_vec[m_bank_level] = bank;
  return addr_vec;
}

void LPDDR6LetheM2Refresh::enqueue_command(const AddrVec_t& addr_vec, int command) {
  Request req(addr_vec, Request::Cmd, command);
  if (!m_ctrl->priority_send(req)) {
    throw std::runtime_error("LPDDR6LetheM2 failed to enqueue maintenance command");
  }
}

void LPDDR6LetheM2Refresh::enqueue_refdb(int ba) {
  const int pair = m_next_pair[ba];
  const int first_bg = pair == 0 ? 0 : 2;
  const int second_bg = first_bg + 1;
  AddrVec_t first = make_addr(first_bg, ba);
  AddrVec_t second = make_addr(second_bg, ba);

  const int first_id = m_ctrl->m_device.get_flat_bank_id(first);
  const int second_id = m_ctrl->m_device.get_flat_bank_id(second);
  if (m_ctrl->m_device.m_bank_nodes[first_id]->m_state != m_state_closed) {
    enqueue_command(first, m_cmd_prepb);
  }
  if (m_ctrl->m_device.m_bank_nodes[second_id]->m_state != m_state_closed) {
    enqueue_command(second, m_cmd_prepb);
  }

  Request ref(first, Request::Cmd, m_cmd_refdb);
  ref.secondary_addr_vec = second;
  if (!m_ctrl->priority_send(ref)) {
    throw std::runtime_error("LPDDR6LetheM2 failed to enqueue REFdb");
  }

  if (m_debug) {
    std::cout << "[LPDDR6LetheM2] due=" << m_ctrl->m_clk << " BA" << ba
              << " pair=(BG" << first_bg << ",BG" << second_bg << ")\n";
  }
  m_next_pair[ba] = (pair + 1) % 2;
}

void LPDDR6LetheM2Refresh::tick() {
  if (m_ctrl->m_clk < m_next_due) return;
  m_next_due += m_slot_interval;

  const int ba = m_next_ba;
  m_next_ba = (m_next_ba + 1) % kBaCount;
  s_slots++;

  if (ba == m_reduced_ba) {
    const bool issue_this_turn = (m_reduced_ba_turn % kReducedBaDivisor) == 0;
    m_reduced_ba_turn++;
    if (!issue_this_turn) {
      s_skipped_reduced_ba_slots++;
      return;
    }
  }

  enqueue_refdb(ba);
}

}  // namespace Ramulator
