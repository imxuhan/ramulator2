#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "ramulator/base/param.h"
#include "ramulator/frontend/i_frontend.h"
#include "ramulator/frontend/impl/processor/champsimO3/core.h"
#include "ramulator/frontend/impl/processor/simpleO3/llc.h"
#include "ramulator/translation/i_translation.h"

namespace Ramulator {

class ChampSimO3 final : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, ChampSimO3, "ChampSimO3");

  ITranslation* m_translation = nullptr;
  int m_num_expected_insts = 0;
  std::vector<std::string> m_traces;
  int m_ipc = 4;
  int m_depth = 128;
  int m_llc_latency = 47;
  int m_llc_linesize_bytes = 64;
  int m_llc_associativity = 8;
  int m_llc_num_mshr_per_core = 16;
  std::string m_llc_capacity_str = "2MB";
  std::string m_trace_abi = "ChampSim input_instr 64-byte";

  std::vector<std::unique_ptr<ChampSimO3Core>> m_cores;
  std::unique_ptr<SimpleO3LLC> m_llc;
  std::vector<size_t> s_insts_retired;
  std::vector<size_t> s_cycles_recorded;
  std::vector<size_t> s_trace_records;
  std::vector<size_t> s_trace_passes_completed;

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_clock_ratio, unsigned int, "clock_ratio").required();
    RAMULATOR_PARSE_PARAM(m_num_expected_insts, int, "num_expected_insts").required();
    RAMULATOR_PARSE_PARAM(m_traces, std::vector<std::string>, "traces").required();
    RAMULATOR_PARSE_PARAM(m_ipc, int, "ipc").default_val(4);
    RAMULATOR_PARSE_PARAM(m_depth, int, "inst_window_depth").default_val(128);
    RAMULATOR_PARSE_PARAM(m_llc_latency, int, "llc_latency").default_val(47);
    RAMULATOR_PARSE_PARAM(m_llc_linesize_bytes, int, "llc_linesize").default_val(64);
    RAMULATOR_PARSE_PARAM(m_llc_associativity, int, "llc_associativity").default_val(8);
    RAMULATOR_PARSE_PARAM(m_llc_capacity_str, std::string, "llc_capacity_per_core").default_val("2MB");
    RAMULATOR_PARSE_PARAM(m_llc_num_mshr_per_core, int, "llc_num_mshr_per_core").default_val(16);

    if (m_traces.size() != 4) {
      throw std::runtime_error("ChampSimO3 M3 frontend requires exactly four traces");
    }
    if (m_num_expected_insts <= 0) {
      throw std::runtime_error("ChampSimO3 num_expected_insts must be positive");
    }
    if (m_num_expected_insts > std::numeric_limits<int>::max() - m_depth) {
      throw std::runtime_error("ChampSimO3 num_expected_insts exceeds request tag capacity");
    }

    RAMULATOR_CREATE_CHILD(m_translation, ITranslation);
    int llc_capacity = parse_capacity_str(m_llc_capacity_str) * static_cast<int>(m_traces.size());
    m_llc = std::make_unique<SimpleO3LLC>(
        m_clk, m_llc_latency, llc_capacity, m_llc_linesize_bytes, m_llc_associativity,
        m_llc_num_mshr_per_core * static_cast<int>(m_traces.size()));

    for (int core_id = 0; core_id < static_cast<int>(m_traces.size()); core_id++) {
      auto core = std::make_unique<ChampSimO3Core>(
          m_clk, core_id, m_ipc, m_depth, static_cast<size_t>(m_num_expected_insts),
          m_traces.at(core_id), m_translation, m_llc.get());
      core->set_callback([this](Request& req) { receive(req); });
      m_cores.push_back(std::move(core));
    }

    s_insts_retired.resize(m_cores.size());
    s_cycles_recorded.resize(m_cores.size());
    s_trace_records.resize(m_cores.size());
    s_trace_passes_completed.resize(m_cores.size());

    m_stats.add("trace_abi", m_trace_abi);
    m_stats.add("num_expected_insts", m_num_expected_insts);
    m_stats.add("insts_retired_per_core", s_insts_retired);
    m_stats.add("cycles_recorded_per_core", s_cycles_recorded);
    m_stats.add("trace_records_per_core", s_trace_records);
    m_stats.add("trace_passes_completed_per_core", s_trace_passes_completed);
    m_stats.add("llc_eviction", m_llc->s_llc_eviction);
    m_stats.add("llc_read_access", m_llc->s_llc_read_access);
    m_stats.add("llc_write_access", m_llc->s_llc_write_access);
    m_stats.add("llc_read_misses", m_llc->s_llc_read_misses);
    m_stats.add("llc_write_misses", m_llc->s_llc_write_misses);
    m_stats.add("llc_mshr_unavailable", m_llc->s_llc_mshr_unavailable);
  }

  void tick() override {
    m_clk++;
    m_llc->tick();
    for (auto& core : m_cores) {
      core->tick();
    }
  }

  void receive(Request& req) {
    m_llc->receive(req);
    auto& waiting = m_llc->m_receive_requests[req.addr];
    for (auto& original : waiting) {
      if (original.source_id >= 0 && original.source_id < static_cast<int>(m_cores.size())) {
        m_cores.at(original.source_id)->receive(original);
      }
    }
    waiting.clear();
  }

  bool is_finished() override {
    for (const auto& core : m_cores) {
      if (!core->reached_expected_num_insts) {
        return false;
      }
    }
    return true;
  }

  void connect_memory_system(IMemorySystem* memory_system) override {
    IFrontEnd::connect_memory_system(memory_system);
    m_llc->connect_memory_system(memory_system);
  }

  int get_num_cores() override { return static_cast<int>(m_cores.size()); }

  void update_stats() override {
    for (size_t core_id = 0; core_id < m_cores.size(); core_id++) {
      s_insts_retired.at(core_id) = m_cores.at(core_id)->s_insts_retired;
      s_cycles_recorded.at(core_id) = m_cores.at(core_id)->s_cycles_recorded;
      s_trace_records.at(core_id) = m_cores.at(core_id)->s_trace_records;
      s_trace_passes_completed.at(core_id) = m_cores.at(core_id)->trace_passes_completed();
    }
  }
};

}  // namespace Ramulator
