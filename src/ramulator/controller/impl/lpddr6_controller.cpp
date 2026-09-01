#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/base.h"
#include "ramulator/controller/impl/lpddr_controller_base.h"
#include "ramulator/dram/dram_spec.h"

namespace Ramulator {

class LPDDR6Controller final : public LPDDRControllerBase {
  RAMULATOR_REGISTER_IMPLEMENTATION_DERIVED(IController, LPDDR6Controller, LPDDRControllerBase, "LPDDR6")

 public:
  void init() override {
    LPDDRControllerBase::init();

    const auto& spec = *m_device.m_spec;
    m_cmd_act1 = spec.get_command_id("ACT1");
    m_cmd_act2 = spec.get_command_id("ACT2");
    m_cmd_cas = spec.get_command_id("CAS");
    m_cmd_rd = spec.get_command_id("RD_S");
    m_cmd_wr = spec.get_command_id("WR_S");
    m_cmd_rda = spec.get_command_id("RDA_S");
    m_cmd_wra = spec.get_command_id("WRA_S");
    m_cmd_rd_l = spec.get_command_id("RD_L");
    m_cmd_wr_l = spec.get_command_id("WR_L");
    m_cmd_rda_l = spec.get_command_id("RDA_L");
    m_cmd_wra_l = spec.get_command_id("WRA_L");
    m_cmd_refdb = spec.get_command_id("REFdb");

    m_bankgroup_level = spec.get_level_id("BankGroup");
    m_bank_level_local = spec.get_level_id("Bank");
    m_bank_count = spec.get_level_size("Bank");
    if (spec.get_level_size("Rank") != 1 || spec.get_level_size("BankGroup") != 4 || m_bank_count != 4) {
      throw std::runtime_error("LPDDR6 REFdb baseline currently requires one rank with 4x4 banks");
    }

    m_nAAD = spec.get_timing_value("nAAD");
    m_read_latency = spec.get_timing_value("nRL");
    m_write_latency = spec.get_timing_value("nWL");
    m_burst_cycles = spec.get_timing_value("nBL_min");
    // BL48 WCK toggles through both 24-beat segments and the gap between them,
    // so the long-burst WCK-expiry window is BL/n_min(BL48), not BL/n_max.
    m_burst_cycles_long = spec.get_timing_value("nBL_min_L");
    m_nWCKPST = spec.get_timing_value("nWCKPST");
    m_cas_deadline_guard = std::max(2, spec.get_timing_value("nCAS"));
    m_nFAW = spec.get_timing_value("nFAW");
    m_nR2R_short = spec.get_timing_value("ndbR2dbR_S");
    m_nR2R_long = spec.get_timing_value("ndbR2dbR_L");

    std::string refdb_mode;
    RAMULATOR_PARSE_PARAM(refdb_mode, std::string, "refdb_mode").default_val("standard");
    if (refdb_mode == "standard") {
      m_refdb_mode = RefdbMode::Standard;
    } else if (refdb_mode == "lethe") {
      m_refdb_mode = RefdbMode::Lethe;
    } else {
      throw std::runtime_error("LPDDR6 refdb_mode must be 'standard' or 'lethe'");
    }

    const std::string refresh_impl =
        m_config["refresh_manager"]["impl"].as<std::string>("");
    if (m_refdb_mode == RefdbMode::Standard && refresh_impl == "LPDDR6LetheM2") {
      throw std::runtime_error("LPDDR6LetheM2 refresh manager requires refdb_mode='lethe'");
    }
    if (m_refdb_mode == RefdbMode::Lethe && refresh_impl == "LPDDR6DualBank") {
      throw std::runtime_error("LPDDR6DualBank refresh manager requires refdb_mode='standard'");
    }

    s_refdb_mode_lethe = m_refdb_mode == RefdbMode::Lethe ? 1 : 0;
    s_refdb_per_bank.assign(m_device.m_bank_nodes.size(), 0);
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    LPDDRControllerBase::setup(frontend, memory_system);
    m_stats.add("refdb_mode_lethe", s_refdb_mode_lethe);
    m_stats.add("refdb_issued", s_refdb_issued);
    m_stats.add("refdb_rounds_completed", s_refdb_rounds_completed);
    m_stats.add("refdb_shared_row", m_refdb_shared_row);
    for (size_t ba = 0; ba < kBaCount; ba++) {
      m_stats.add(fmt::format("refdb_ba_{}_issued", ba), s_refdb_per_ba[ba]);
      m_stats.add(fmt::format("refdb_ba_{}_rounds_completed", ba),
                  s_refdb_ba_rounds_completed[ba]);
      m_stats.add(fmt::format("refdb_ba_{}_row", ba), m_refdb_ba_row[ba]);
    }
    for (size_t bank = 0; bank < s_refdb_per_bank.size(); bank++) {
      m_stats.add(fmt::format("refdb_bank_{}", bank), s_refdb_per_bank[bank]);
    }
  }

  void reset_stats() override {
    LPDDRControllerBase::reset_stats();
    s_refdb_issued = 0;
    s_refdb_rounds_completed = 0;
    s_refdb_per_ba.fill(0);
    s_refdb_ba_rounds_completed.fill(0);
    std::fill(s_refdb_per_bank.begin(), s_refdb_per_bank.end(), 0);
  }

 protected:
  bool protocol_allows(const Request& req, Clk_t issue_clk) const override {
    int activation_credits = req.command == m_cmd_act2 ? 1 : (req.command == m_cmd_refdb ? 2 : 0);
    if (activation_credits > 0) {
      int live_credits = 0;
      for (Clk_t issued_at : m_activation_credits) {
        if (issued_at + m_nFAW > issue_clk) live_credits++;
      }
      if (live_credits + activation_credits > 4) return false;
    }

    if (req.command != m_cmd_refdb) return true;
    if (req.secondary_addr_vec.empty()) return false;

    if (m_refdb_mode == RefdbMode::Standard) {
      if (issue_clk < m_next_refdb_cycle) return false;

      int first = coverage_bit(req.addr_vec);
      int second = coverage_bit(req.secondary_addr_vec);
      uint16_t pair = static_cast<uint16_t>((1u << first) | (1u << second));
      return (m_refdb_coverage & pair) == 0;
    }

    int ba = bank_address(req.addr_vec);
    int second_ba = bank_address(req.secondary_addr_vec);
    if (ba != second_ba) return false;

    uint8_t pair = lethe_coverage_pair(req);
    if ((m_refdb_ba_coverage[ba] & pair) != 0) return false;

    if (m_last_refdb_cycle >= 0) {
      Clk_t gap = m_nR2R_short;
      if (m_last_refdb_ba == ba && m_last_refdb_advanced) {
        gap = m_nR2R_long;
      }
      if (issue_clk < m_last_refdb_cycle + gap) return false;
    }
    return true;
  }

  void protocol_on_issue(const Request& req) override {
    while (!m_activation_credits.empty() && m_activation_credits.front() + m_nFAW <= m_clk) {
      m_activation_credits.pop_front();
    }

    int activation_credits = req.command == m_cmd_act2 ? 1 : (req.command == m_cmd_refdb ? 2 : 0);
    for (int i = 0; i < activation_credits; i++) m_activation_credits.push_back(m_clk);

    if (req.command != m_cmd_refdb) return;

    int first = coverage_bit(req.addr_vec);
    int second = coverage_bit(req.secondary_addr_vec);
    s_refdb_issued++;
    s_refdb_per_bank[first]++;
    s_refdb_per_bank[second]++;
    int ba = bank_address(req.addr_vec);
    s_refdb_per_ba[ba]++;

    if (m_refdb_mode == RefdbMode::Standard) {
      uint16_t pair = static_cast<uint16_t>((1u << first) | (1u << second));
      if ((m_refdb_coverage & pair) != 0) {
        throw std::runtime_error("LPDDR6 REFdb repeated a bank before completing the shared-counter round");
      }

      m_refdb_coverage |= pair;
      if (std::popcount(m_refdb_coverage) == 16) {
        m_refdb_coverage = 0;
        m_refdb_shared_row = (m_refdb_shared_row + 1) % kRefreshRows;
        s_refdb_rounds_completed++;
        m_next_refdb_cycle = m_clk + m_nR2R_long;
      } else {
        m_next_refdb_cycle = m_clk + m_nR2R_short;
      }
      return;
    }

    uint8_t pair = lethe_coverage_pair(req);
    if ((m_refdb_ba_coverage[ba] & pair) != 0) {
      throw std::runtime_error("LPDDR6 Lethe REFdb repeated a bank before completing the BA round");
    }

    m_refdb_ba_coverage[ba] |= pair;
    bool advanced = std::popcount(m_refdb_ba_coverage[ba]) == 4;
    if (advanced) {
      m_refdb_ba_coverage[ba] = 0;
      m_refdb_ba_row[ba] = (m_refdb_ba_row[ba] + 1) % kRefreshRows;
      s_refdb_ba_rounds_completed[ba]++;
    }
    m_last_refdb_cycle = m_clk;
    m_last_refdb_ba = ba;
    m_last_refdb_advanced = advanced;
  }

 private:
  enum class RefdbMode {
    Standard,
    Lethe,
  };

  static constexpr size_t kBaCount = 4;
  static constexpr size_t kRefreshRows = 8192;

  int m_cmd_refdb = -1;
  int m_bankgroup_level = -1;
  int m_bank_level_local = -1;
  int m_bank_count = -1;
  int m_nFAW = -1;
  int m_nR2R_short = -1;
  int m_nR2R_long = -1;
  RefdbMode m_refdb_mode = RefdbMode::Standard;
  Clk_t m_next_refdb_cycle = 0;
  uint16_t m_refdb_coverage = 0;
  size_t m_refdb_shared_row = 0;
  std::array<uint8_t, kBaCount> m_refdb_ba_coverage{};
  std::array<size_t, kBaCount> m_refdb_ba_row{};
  Clk_t m_last_refdb_cycle = -1;
  int m_last_refdb_ba = -1;
  bool m_last_refdb_advanced = false;
  std::deque<Clk_t> m_activation_credits;

  size_t s_refdb_mode_lethe = 0;
  size_t s_refdb_issued = 0;
  size_t s_refdb_rounds_completed = 0;
  std::array<size_t, kBaCount> s_refdb_per_ba{};
  std::array<size_t, kBaCount> s_refdb_ba_rounds_completed{};
  std::vector<size_t> s_refdb_per_bank;

  int bank_address(const AddrVec_t& addr_vec) const {
    int ba = addr_vec[m_bank_level_local];
    if (ba < 0 || ba >= static_cast<int>(kBaCount)) {
      throw std::runtime_error("LPDDR6 REFdb requires a concrete in-range bank address");
    }
    return ba;
  }

  uint8_t lethe_coverage_pair(const Request& req) const {
    int first_bg = req.addr_vec[m_bankgroup_level];
    int second_bg = req.secondary_addr_vec[m_bankgroup_level];
    if (first_bg < 0 || first_bg >= 4 || second_bg < 0 || second_bg >= 4 ||
        first_bg == second_bg) {
      throw std::runtime_error("LPDDR6 Lethe REFdb requires two different in-range bank groups");
    }
    return static_cast<uint8_t>((1u << first_bg) | (1u << second_bg));
  }

  int coverage_bit(const AddrVec_t& addr_vec) const {
    int bankgroup = addr_vec[m_bankgroup_level];
    int bank = addr_vec[m_bank_level_local];
    if (bankgroup < 0 || bankgroup >= 4 || bank < 0 || bank >= m_bank_count) {
      throw std::runtime_error("LPDDR6 REFdb requires concrete in-range bank-group and bank addresses");
    }
    return bankgroup * m_bank_count + bank;
  }
};

}  // namespace Ramulator
