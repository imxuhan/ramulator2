#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/refresh/i_refresh_manager.h"
#include "ramulator/dram/dram_spec.h"
#include "ramulator/dram/node.h"

namespace Ramulator {

// JESD209-6 dual-bank refresh. Each rank uses one shared row counter: eight
// REFdb commands cover all 16 banks before the sequence repeats.
class LPDDR6DualBankRefresh final : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, LPDDR6DualBankRefresh, "LPDDR6DualBank")

 private:
  ControllerBase* m_ctrl = nullptr;
  int m_cmd_prepb = -1;
  int m_cmd_refdb = -1;
  int m_state_closed = -1;
  int m_rank_level = -1;
  int m_bankgroup_level = -1;
  int m_bank_level = -1;
  int m_rank_count = -1;
  int m_bankgroup_count = -1;
  int m_bank_count = -1;
  int m_interval = -1;
  float m_refresh_multiplier = 1.0f;
  bool m_debug = false;

  std::vector<Clk_t> m_next_due;
  std::vector<int> m_next_pair;

  void init() override;
  void tick() override;

  AddrVec_t make_addr(int rank, int bankgroup, int bank) const;
  void enqueue_command(const AddrVec_t& addr_vec, int command);
  void enqueue_refdb(int rank);
};

void LPDDR6DualBankRefresh::init() {
  m_ctrl = cast_parent<ControllerBase>();
  RAMULATOR_PARSE_PARAM(m_refresh_multiplier, float, "refresh_multiplier").default_val(1.0f);
  RAMULATOR_PARSE_PARAM(m_debug, bool, "debug").default_val(false);

  if (m_refresh_multiplier <= 0.0f || m_refresh_multiplier > 1.0f) {
    throw std::runtime_error("LPDDR6DualBank refresh_multiplier must be in (0, 1]");
  }

  const auto& spec = *m_ctrl->m_device.m_spec;
  if (spec.standard_name != "LPDDR6") {
    throw std::runtime_error("LPDDR6DualBank refresh manager requires LPDDR6 DRAM");
  }

  m_cmd_prepb = spec.get_command_id("PREpb");
  m_cmd_refdb = spec.get_command_id("REFdb");
  m_state_closed = spec.get_state_id("Closed");
  m_rank_level = spec.get_level_id("Rank");
  m_bankgroup_level = spec.get_level_id("BankGroup");
  m_bank_level = spec.get_level_id("Bank");
  m_rank_count = spec.get_level_size("Rank");
  m_bankgroup_count = spec.get_level_size("BankGroup");
  m_bank_count = spec.get_level_size("Bank");

  if (m_bankgroup_count != 4 || m_bank_count != 4) {
    throw std::runtime_error("LPDDR6DualBank requires four bank groups with four banks each");
  }

  int nrefidb = spec.get_timing_value("nREFIdb");
  m_interval = std::max(1, static_cast<int>(std::llround(nrefidb * m_refresh_multiplier)));
  m_next_due.resize(m_rank_count);
  m_next_pair.assign(m_rank_count, 0);
  for (int rank = 0; rank < m_rank_count; rank++) {
    m_next_due[rank] = static_cast<Clk_t>(rank + 1) * m_interval;
  }
}

AddrVec_t LPDDR6DualBankRefresh::make_addr(int rank, int bankgroup, int bank) const {
  AddrVec_t addr_vec(m_ctrl->m_device.m_spec->level_count, -1);
  addr_vec[0] = m_ctrl->m_channel_id;
  addr_vec[m_rank_level] = rank;
  addr_vec[m_bankgroup_level] = bankgroup;
  addr_vec[m_bank_level] = bank;
  return addr_vec;
}

void LPDDR6DualBankRefresh::enqueue_command(const AddrVec_t& addr_vec, int command) {
  Request req(addr_vec, Request::Cmd, command);
  if (!m_ctrl->priority_send(req)) {
    throw std::runtime_error("LPDDR6DualBank failed to enqueue maintenance command");
  }
}

void LPDDR6DualBankRefresh::enqueue_refdb(int rank) {
  int pair_index = m_next_pair[rank];
  int bank = pair_index / 2;
  int first_bg = pair_index % 2 == 0 ? 0 : 2;
  int second_bg = first_bg + 1;
  AddrVec_t first = make_addr(rank, first_bg, bank);
  AddrVec_t second = make_addr(rank, second_bg, bank);

  int first_id = m_ctrl->m_device.get_flat_bank_id(first);
  int second_id = m_ctrl->m_device.get_flat_bank_id(second);
  if (m_ctrl->m_device.m_bank_nodes[first_id]->m_state != m_state_closed) {
    enqueue_command(first, m_cmd_prepb);
  }
  if (m_ctrl->m_device.m_bank_nodes[second_id]->m_state != m_state_closed) {
    enqueue_command(second, m_cmd_prepb);
  }

  Request ref(first, Request::Cmd, m_cmd_refdb);
  ref.secondary_addr_vec = second;
  if (!m_ctrl->priority_send(ref)) {
    throw std::runtime_error("LPDDR6DualBank failed to enqueue REFdb");
  }

  if (m_debug) {
    std::cout << "[LPDDR6DualBank] due=" << m_ctrl->m_clk << " rank=" << rank
              << " pair=(BG" << first_bg << ",BG" << second_bg << ",BA" << bank << ")\n";
  }
  m_next_pair[rank] = (pair_index + 1) % 8;
}

void LPDDR6DualBankRefresh::tick() {
  for (int rank = 0; rank < m_rank_count; rank++) {
    if (m_ctrl->m_clk < m_next_due[rank]) continue;
    m_next_due[rank] += m_interval;
    enqueue_refdb(rank);
  }
}

}  // namespace Ramulator
