#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/refresh/i_refresh_manager.h"
#include "ramulator/dram/dram_spec.h"
#include "ramulator/dram/node.h"

namespace Ramulator {

// M4 standard REFdb manager. It retains the shared counter in the LPDDR6
// controller and adds a traffic-aware, deterministic DARP-style schedule.
class LPDDR6DualBankM4Refresh final : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, LPDDR6DualBankM4Refresh, "LPDDR6DualBankM4")

 private:
  static constexpr int kBaCount = 4;
  static constexpr int kMaxDebt = 64;
  static constexpr std::array<std::array<int, 2>, 6> kBgPairs{{
      {{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}},
  }};

  enum class Schedule { RoundRobin, Darp };
  struct PairCandidate {
    bool valid = false;
    int ba = -1;
    int first_bg = -1;
    int second_bg = -1;
    size_t score = 0;
  };

  ControllerBase* m_ctrl = nullptr;
  int m_cmd_prepb = -1;
  int m_cmd_refdb = -1;
  int m_state_closed = -1;
  int m_rank_level = -1;
  int m_bankgroup_level = -1;
  int m_bank_level = -1;
  int m_rank_count = -1;
  int m_interval = -1;
  float m_refresh_multiplier = 1.0f;
  Schedule m_schedule = Schedule::RoundRobin;
  bool m_debug = false;

  std::vector<Clk_t> m_next_due;
  std::vector<int> m_next_pair;
  std::vector<int> m_debt;
  std::vector<std::array<uint8_t, kBaCount>> m_scheduled_coverage;
  std::vector<uint8_t> m_completed_bas;

  size_t s_slots = 0;
  size_t s_deferred_cycles = 0;
  size_t s_forced_at_debt_limit = 0;
  size_t s_refresh_prepb = 0;
  size_t s_selected_queue_depth = 0;
  size_t s_debt_peak = 0;

  void init() override;
  void setup(IFrontEnd*, IMemorySystem*) override;
  void reset_stats() override;
  void tick() override;

  AddrVec_t make_addr(int rank, int bankgroup, int ba) const;
  void enqueue_command(const AddrVec_t& addr_vec, int command);
  void enqueue_refdb(int rank, int ba, int first_bg, int second_bg);
  PairCandidate select_darp_candidate(int rank) const;
  void record_scheduled_pair(int rank, const PairCandidate& candidate);
};

void LPDDR6DualBankM4Refresh::init() {
  m_ctrl = cast_parent<ControllerBase>();
  RAMULATOR_PARSE_PARAM(m_refresh_multiplier, float, "refresh_multiplier").default_val(1.0f);
  std::string schedule;
  RAMULATOR_PARSE_PARAM(schedule, std::string, "schedule").default_val("round_robin");
  RAMULATOR_PARSE_PARAM(m_debug, bool, "debug").default_val(false);

  if (m_refresh_multiplier <= 0.0f || m_refresh_multiplier > 1.0f) {
    throw std::runtime_error("LPDDR6DualBankM4 refresh_multiplier must be in (0, 1]");
  }
  if (schedule == "round_robin") {
    m_schedule = Schedule::RoundRobin;
  } else if (schedule == "darp") {
    m_schedule = Schedule::Darp;
  } else {
    throw std::runtime_error("LPDDR6DualBankM4 schedule must be 'round_robin' or 'darp'");
  }

  const auto& spec = *m_ctrl->m_device.m_spec;
  if (spec.standard_name != "LPDDR6" || spec.get_level_size("Rank") != 1 ||
      spec.get_level_size("BankGroup") != 4 || spec.get_level_size("Bank") != 4) {
    throw std::runtime_error("LPDDR6DualBankM4 requires one-rank LPDDR6 with 4x4 banks");
  }

  m_cmd_prepb = spec.get_command_id("PREpb");
  m_cmd_refdb = spec.get_command_id("REFdb");
  m_state_closed = spec.get_state_id("Closed");
  m_rank_level = spec.get_level_id("Rank");
  m_bankgroup_level = spec.get_level_id("BankGroup");
  m_bank_level = spec.get_level_id("Bank");
  m_rank_count = spec.get_level_size("Rank");
  const int nrefidb = spec.get_timing_value("nREFIdb");
  m_interval = std::max(1, static_cast<int>(std::llround(nrefidb * m_refresh_multiplier)));

  m_next_due.resize(m_rank_count);
  m_next_pair.assign(m_rank_count, 0);
  m_debt.assign(m_rank_count, 0);
  m_scheduled_coverage.resize(m_rank_count);
  m_completed_bas.assign(m_rank_count, 0);
  for (int rank = 0; rank < m_rank_count; rank++) {
    m_next_due[rank] = static_cast<Clk_t>(rank + 1) * m_interval;
    m_scheduled_coverage[rank].fill(0);
  }
}

void LPDDR6DualBankM4Refresh::setup(IFrontEnd*, IMemorySystem*) {
  m_stats.add("slots", s_slots);
  m_stats.add("darp_deferred_cycles", s_deferred_cycles);
  m_stats.add("darp_forced_at_debt_limit", s_forced_at_debt_limit);
  m_stats.add("refresh_prepb", s_refresh_prepb);
  m_stats.add("darp_selected_queue_depth", s_selected_queue_depth);
  m_stats.add("darp_debt_peak", s_debt_peak);
}

void LPDDR6DualBankM4Refresh::reset_stats() {
  s_slots = 0;
  s_deferred_cycles = 0;
  s_forced_at_debt_limit = 0;
  s_refresh_prepb = 0;
  s_selected_queue_depth = 0;
  s_debt_peak = 0;
}

AddrVec_t LPDDR6DualBankM4Refresh::make_addr(int rank, int bankgroup, int ba) const {
  AddrVec_t addr_vec(m_ctrl->m_device.m_spec->level_count, -1);
  addr_vec[0] = m_ctrl->m_channel_id;
  addr_vec[m_rank_level] = rank;
  addr_vec[m_bankgroup_level] = bankgroup;
  addr_vec[m_bank_level] = ba;
  return addr_vec;
}

void LPDDR6DualBankM4Refresh::enqueue_command(const AddrVec_t& addr_vec, int command) {
  Request req(addr_vec, Request::Cmd, command);
  if (!m_ctrl->priority_send(req)) {
    throw std::runtime_error("LPDDR6DualBankM4 failed to enqueue maintenance command");
  }
}

void LPDDR6DualBankM4Refresh::enqueue_refdb(
    int rank, int ba, int first_bg, int second_bg) {
  AddrVec_t first = make_addr(rank, first_bg, ba);
  AddrVec_t second = make_addr(rank, second_bg, ba);
  const int first_id = m_ctrl->m_device.get_flat_bank_id(first);
  const int second_id = m_ctrl->m_device.get_flat_bank_id(second);
  if (m_ctrl->m_device.m_bank_nodes[first_id]->m_state != m_state_closed) {
    enqueue_command(first, m_cmd_prepb);
    s_refresh_prepb++;
  }
  if (m_ctrl->m_device.m_bank_nodes[second_id]->m_state != m_state_closed) {
    enqueue_command(second, m_cmd_prepb);
    s_refresh_prepb++;
  }

  Request ref(first, Request::Cmd, m_cmd_refdb);
  ref.secondary_addr_vec = second;
  if (!m_ctrl->priority_send(ref)) {
    throw std::runtime_error("LPDDR6DualBankM4 failed to enqueue REFdb");
  }
  if (m_debug) {
    std::cout << "[LPDDR6DualBankM4] rank=" << rank << " BA" << ba
              << " pair=(BG" << first_bg << ",BG" << second_bg << ")\n";
  }
}

LPDDR6DualBankM4Refresh::PairCandidate
LPDDR6DualBankM4Refresh::select_darp_candidate(int rank) const {
  PairCandidate best;
  for (int ba = 0; ba < kBaCount; ba++) {
    if ((m_completed_bas[rank] & (1u << ba)) != 0) continue;
    const uint8_t coverage = m_scheduled_coverage[rank][ba];
    for (const auto& pair : kBgPairs) {
      const uint8_t bits = static_cast<uint8_t>((1u << pair[0]) | (1u << pair[1]));
      if ((coverage & bits) != 0) continue;
      if (coverage != 0 && std::popcount(static_cast<unsigned>(coverage | bits)) != 4) continue;
      const AddrVec_t first = make_addr(rank, pair[0], ba);
      const AddrVec_t second = make_addr(rank, pair[1], ba);
      const size_t score = m_ctrl->count_pending_requests_for_banks({
          m_ctrl->m_device.get_flat_bank_id(first),
          m_ctrl->m_device.get_flat_bank_id(second),
      });
      if (!best.valid || score < best.score) {
        best = {true, ba, pair[0], pair[1], score};
      }
    }
  }
  return best;
}

void LPDDR6DualBankM4Refresh::record_scheduled_pair(
    int rank, const PairCandidate& candidate) {
  const uint8_t bits = static_cast<uint8_t>(
      (1u << candidate.first_bg) | (1u << candidate.second_bg));
  auto& coverage = m_scheduled_coverage[rank][candidate.ba];
  coverage |= bits;
  if (std::popcount(static_cast<unsigned>(coverage)) == 4) {
    m_completed_bas[rank] |= static_cast<uint8_t>(1u << candidate.ba);
  }
  if (std::popcount(static_cast<unsigned>(m_completed_bas[rank])) == kBaCount) {
    m_scheduled_coverage[rank].fill(0);
    m_completed_bas[rank] = 0;
  }
}

void LPDDR6DualBankM4Refresh::tick() {
  for (int rank = 0; rank < m_rank_count; rank++) {
    if (m_schedule == Schedule::RoundRobin) {
      if (m_ctrl->m_clk < m_next_due[rank]) continue;
      m_next_due[rank] += m_interval;
      const int pair_index = m_next_pair[rank];
      const int ba = pair_index / 2;
      const int first_bg = pair_index % 2 == 0 ? 0 : 2;
      enqueue_refdb(rank, ba, first_bg, first_bg + 1);
      m_next_pair[rank] = (pair_index + 1) % 8;
      s_slots++;
      continue;
    }

    while (m_ctrl->m_clk >= m_next_due[rank]) {
      m_next_due[rank] += m_interval;
      m_debt[rank]++;
      s_slots++;
      s_debt_peak = std::max(s_debt_peak, static_cast<size_t>(m_debt[rank]));
    }
    if (m_debt[rank] == 0) continue;

    PairCandidate candidate = select_darp_candidate(rank);
    if (!candidate.valid) {
      throw std::runtime_error("LPDDR6DualBankM4 DARP found no legal REFdb pair");
    }
    if (candidate.score > 0 && m_debt[rank] < kMaxDebt) {
      s_deferred_cycles++;
      continue;
    }
    if (candidate.score > 0) s_forced_at_debt_limit++;
    enqueue_refdb(rank, candidate.ba, candidate.first_bg, candidate.second_bg);
    record_scheduled_pair(rank, candidate);
    s_selected_queue_depth += candidate.score;
    m_debt[rank]--;
  }
}

}  // namespace Ramulator
