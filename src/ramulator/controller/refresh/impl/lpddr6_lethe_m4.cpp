#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "fmt/format.h"
#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/refresh/i_refresh_manager.h"
#include "ramulator/dram/dram_spec.h"
#include "ramulator/dram/node.h"

namespace Ramulator {

// General M4 Lethe manager: four independent BA streams, relative rates
// {1, 1/2, 1/4, 0}, and symmetric round-robin/DARP-style schedules.
class LPDDR6LetheM4Refresh final : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, LPDDR6LetheM4Refresh, "LPDDR6LetheM4")

 private:
  static constexpr int kBaCount = 4;
  static constexpr int kMaxDebt = 64;
  static constexpr std::array<std::array<int, 2>, 6> kBgPairs{{
      {{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}},
  }};

  enum class Schedule { RoundRobin, Darp };
  struct PairCandidate {
    bool valid = false;
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
  int m_base_interval = -1;
  float m_refresh_multiplier = 1.0f;
  std::vector<double> m_ba_ratios;
  Schedule m_schedule = Schedule::RoundRobin;
  bool m_debug = false;

  std::array<Clk_t, kBaCount> m_next_due{};
  std::array<int, kBaCount> m_ba_interval{};
  std::array<int, kBaCount> m_next_pair{};
  std::array<int, kBaCount> m_debt{};
  std::array<uint8_t, kBaCount> m_scheduled_coverage{};

  size_t s_slots = 0;
  size_t s_deferred_cycles = 0;
  size_t s_forced_at_debt_limit = 0;
  size_t s_refresh_prepb = 0;
  size_t s_selected_queue_depth = 0;
  size_t s_debt_peak = 0;
  std::array<size_t, kBaCount> s_due_per_ba{};
  std::array<size_t, kBaCount> s_deferred_per_ba{};

  void init() override;
  void setup(IFrontEnd*, IMemorySystem*) override;
  void reset_stats() override;
  void tick() override;

  static bool valid_ratio(double ratio);
  AddrVec_t make_addr(int bankgroup, int ba) const;
  void enqueue_command(const AddrVec_t& addr_vec, int command);
  void enqueue_refdb(int ba, int first_bg, int second_bg);
  PairCandidate select_darp_candidate(int ba) const;
  void record_scheduled_pair(int ba, int first_bg, int second_bg);
};

bool LPDDR6LetheM4Refresh::valid_ratio(double ratio) {
  constexpr double epsilon = 1e-9;
  for (double allowed : {0.0, 0.25, 0.5, 1.0}) {
    if (std::abs(ratio - allowed) < epsilon) return true;
  }
  return false;
}

void LPDDR6LetheM4Refresh::init() {
  m_ctrl = cast_parent<ControllerBase>();
  RAMULATOR_PARSE_PARAM(m_refresh_multiplier, float, "refresh_multiplier").default_val(1.0f);
  RAMULATOR_PARSE_PARAM(m_ba_ratios, std::vector<double>, "ba_ratios").required();
  std::string schedule;
  RAMULATOR_PARSE_PARAM(schedule, std::string, "schedule").default_val("round_robin");
  RAMULATOR_PARSE_PARAM(m_debug, bool, "debug").default_val(false);

  if (m_refresh_multiplier <= 0.0f || m_refresh_multiplier > 1.0f) {
    throw std::runtime_error("LPDDR6LetheM4 refresh_multiplier must be in (0, 1]");
  }
  if (m_ba_ratios.size() != kBaCount) {
    throw std::runtime_error("LPDDR6LetheM4 ba_ratios must contain exactly four values");
  }
  for (double ratio : m_ba_ratios) {
    if (!valid_ratio(ratio)) {
      throw std::runtime_error("LPDDR6LetheM4 ratios must be one of 0, 0.25, 0.5, 1");
    }
  }
  if (schedule == "round_robin") {
    m_schedule = Schedule::RoundRobin;
  } else if (schedule == "darp") {
    m_schedule = Schedule::Darp;
  } else {
    throw std::runtime_error("LPDDR6LetheM4 schedule must be 'round_robin' or 'darp'");
  }

  const auto& spec = *m_ctrl->m_device.m_spec;
  if (spec.standard_name != "LPDDR6" || spec.get_level_size("Rank") != 1 ||
      spec.get_level_size("BankGroup") != 4 || spec.get_level_size("Bank") != 4) {
    throw std::runtime_error("LPDDR6LetheM4 requires one-rank LPDDR6 with 4x4 banks");
  }
  m_cmd_prepb = spec.get_command_id("PREpb");
  m_cmd_refdb = spec.get_command_id("REFdb");
  m_state_closed = spec.get_state_id("Closed");
  m_rank_level = spec.get_level_id("Rank");
  m_bankgroup_level = spec.get_level_id("BankGroup");
  m_bank_level = spec.get_level_id("Bank");
  const int nrefidb = spec.get_timing_value("nREFIdb");
  m_base_interval = std::max(
      1, static_cast<int>(std::llround(nrefidb * m_refresh_multiplier)));

  for (int ba = 0; ba < kBaCount; ba++) {
    if (m_ba_ratios[ba] == 0.0) {
      m_ba_interval[ba] = 0;
      m_next_due[ba] = -1;
    } else {
      m_ba_interval[ba] = std::max(
          1, static_cast<int>(std::llround(4.0 * m_base_interval / m_ba_ratios[ba])));
      m_next_due[ba] = static_cast<Clk_t>(ba + 1) * m_base_interval;
    }
  }
}

void LPDDR6LetheM4Refresh::setup(IFrontEnd*, IMemorySystem*) {
  m_stats.add("slots", s_slots);
  m_stats.add("darp_deferred_cycles", s_deferred_cycles);
  m_stats.add("darp_forced_at_debt_limit", s_forced_at_debt_limit);
  m_stats.add("refresh_prepb", s_refresh_prepb);
  m_stats.add("darp_selected_queue_depth", s_selected_queue_depth);
  m_stats.add("darp_debt_peak", s_debt_peak);
  for (int ba = 0; ba < kBaCount; ba++) {
    m_stats.add(fmt::format("due_ba_{}", ba), s_due_per_ba[ba]);
    m_stats.add(fmt::format("darp_deferred_ba_{}", ba), s_deferred_per_ba[ba]);
  }
}

void LPDDR6LetheM4Refresh::reset_stats() {
  s_slots = 0;
  s_deferred_cycles = 0;
  s_forced_at_debt_limit = 0;
  s_refresh_prepb = 0;
  s_selected_queue_depth = 0;
  s_debt_peak = 0;
  s_due_per_ba.fill(0);
  s_deferred_per_ba.fill(0);
}

AddrVec_t LPDDR6LetheM4Refresh::make_addr(int bankgroup, int ba) const {
  AddrVec_t addr_vec(m_ctrl->m_device.m_spec->level_count, -1);
  addr_vec[0] = m_ctrl->m_channel_id;
  addr_vec[m_rank_level] = 0;
  addr_vec[m_bankgroup_level] = bankgroup;
  addr_vec[m_bank_level] = ba;
  return addr_vec;
}

void LPDDR6LetheM4Refresh::enqueue_command(const AddrVec_t& addr_vec, int command) {
  Request req(addr_vec, Request::Cmd, command);
  if (!m_ctrl->priority_send(req)) {
    throw std::runtime_error("LPDDR6LetheM4 failed to enqueue maintenance command");
  }
}

void LPDDR6LetheM4Refresh::enqueue_refdb(int ba, int first_bg, int second_bg) {
  AddrVec_t first = make_addr(first_bg, ba);
  AddrVec_t second = make_addr(second_bg, ba);
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
    throw std::runtime_error("LPDDR6LetheM4 failed to enqueue REFdb");
  }
  if (m_debug) {
    std::cout << "[LPDDR6LetheM4] BA" << ba << " pair=(BG" << first_bg
              << ",BG" << second_bg << ")\n";
  }
}

LPDDR6LetheM4Refresh::PairCandidate
LPDDR6LetheM4Refresh::select_darp_candidate(int ba) const {
  PairCandidate best;
  const uint8_t coverage = m_scheduled_coverage[ba];
  for (const auto& pair : kBgPairs) {
    const uint8_t bits = static_cast<uint8_t>((1u << pair[0]) | (1u << pair[1]));
    if ((coverage & bits) != 0) continue;
    if (coverage != 0 && std::popcount(static_cast<unsigned>(coverage | bits)) != 4) continue;
    const AddrVec_t first = make_addr(pair[0], ba);
    const AddrVec_t second = make_addr(pair[1], ba);
    const size_t score = m_ctrl->count_pending_requests_for_banks({
        m_ctrl->m_device.get_flat_bank_id(first),
        m_ctrl->m_device.get_flat_bank_id(second),
    });
    if (!best.valid || score < best.score) {
      best = {true, pair[0], pair[1], score};
    }
  }
  return best;
}

void LPDDR6LetheM4Refresh::record_scheduled_pair(
    int ba, int first_bg, int second_bg) {
  m_scheduled_coverage[ba] |= static_cast<uint8_t>(
      (1u << first_bg) | (1u << second_bg));
  if (std::popcount(static_cast<unsigned>(m_scheduled_coverage[ba])) == 4) {
    m_scheduled_coverage[ba] = 0;
  }
}

void LPDDR6LetheM4Refresh::tick() {
  for (int ba = 0; ba < kBaCount; ba++) {
    if (m_ba_interval[ba] == 0) continue;
    while (m_ctrl->m_clk >= m_next_due[ba]) {
      m_next_due[ba] += m_ba_interval[ba];
      m_debt[ba]++;
      s_slots++;
      s_due_per_ba[ba]++;
      s_debt_peak = std::max(s_debt_peak, static_cast<size_t>(m_debt[ba]));
    }
    if (m_debt[ba] == 0) continue;

    if (m_schedule == Schedule::RoundRobin) {
      const int first_bg = m_next_pair[ba] == 0 ? 0 : 2;
      enqueue_refdb(ba, first_bg, first_bg + 1);
      m_next_pair[ba] = (m_next_pair[ba] + 1) % 2;
      m_debt[ba]--;
      continue;
    }

    PairCandidate candidate = select_darp_candidate(ba);
    if (!candidate.valid) {
      throw std::runtime_error("LPDDR6LetheM4 DARP found no legal REFdb pair");
    }
    if (candidate.score > 0 && m_debt[ba] < kMaxDebt) {
      s_deferred_cycles++;
      s_deferred_per_ba[ba]++;
      continue;
    }
    if (candidate.score > 0) s_forced_at_debt_limit++;
    enqueue_refdb(ba, candidate.first_bg, candidate.second_bg);
    record_scheduled_pair(ba, candidate.first_bg, candidate.second_bg);
    s_selected_queue_depth += candidate.score;
    m_debt[ba]--;
  }
}

}  // namespace Ramulator
