#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <unordered_set>

#include <fmt/format.h>

#include "ramulator/base/param.h"
#include "ramulator/controller/i_controller.h"
#include "ramulator/memory_system/channel_mapper/i_channel_mapper.h"
#include "ramulator/memory_system/i_memory_system.h"
#include "ramulator/translation/i_translation.h"

namespace Ramulator {

class GenericDRAMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, GenericDRAMSystem, "GenericDRAM");

 protected:
  IChannelMapper* m_channel_mapper;
  std::vector<IController*> m_controllers;
  unsigned int m_clock_ratio = 1;
  int m_tx_bytes = 0;
  size_t m_num_cores = 0;
  std::vector<int> m_tolerant_cores;
  std::unordered_set<int> m_tolerant_core_set;

  std::vector<size_t> s_demand_reads_served_per_core;
  std::vector<size_t> s_read_latency_per_core;
  std::vector<float> s_avg_read_latency_per_core;
  std::vector<size_t> s_p99_read_latency_per_core;
  std::vector<std::map<int, size_t>> m_read_latency_histogram_per_core;
  std::vector<std::vector<size_t>> s_read_latency_histogram_bins_per_core;
  std::vector<std::vector<size_t>> s_read_latency_histogram_counts_per_core;
  std::array<size_t, 2> s_class_demand_reads_served{};
  std::array<size_t, 2> s_class_read_latency{};
  std::array<float, 2> s_class_avg_read_latency{};
  std::array<size_t, 2> s_class_p99_read_latency{};
  std::array<std::map<int, size_t>, 2> m_class_read_latency_histogram;
  std::array<std::vector<size_t>, 2> s_class_read_latency_histogram_bins;
  std::array<std::vector<size_t>, 2> s_class_read_latency_histogram_counts;

  static size_t percentile99(const std::map<int, size_t>& histogram, size_t count) {
    if (count == 0) return 0;
    const size_t target = (count * 99 + 99) / 100;
    size_t cumulative = 0;
    for (const auto& [latency, occurrences] : histogram) {
      cumulative += occurrences;
      if (cumulative >= target) return static_cast<size_t>(latency);
    }
    throw std::runtime_error("GenericDRAM pooled latency histogram is incomplete");
  }

  static void sparsify(
      const std::map<int, size_t>& histogram, std::vector<size_t>& bins,
      std::vector<size_t>& counts) {
    bins.clear();
    counts.clear();
    bins.reserve(histogram.size());
    counts.reserve(histogram.size());
    for (const auto& [latency, occurrences] : histogram) {
      bins.push_back(static_cast<size_t>(latency));
      counts.push_back(occurrences);
    }
  }

  void record_read_completion(const Request& req) {
    if (req.ingress_id < 0 || req.source_id < 0 ||
        req.source_id >= static_cast<int>(m_num_cores)) return;
    if (req.arrive < 0 || req.depart < req.arrive) {
      throw std::runtime_error("GenericDRAM received an invalid completed-read latency");
    }
    const int latency = static_cast<int>(req.depart - req.arrive);
    const size_t core = static_cast<size_t>(req.source_id);
    s_demand_reads_served_per_core.at(core)++;
    s_read_latency_per_core.at(core) += latency;
    m_read_latency_histogram_per_core.at(core)[latency]++;
    const bool tolerant = req.memory_class >= 0
                              ? req.memory_class == 1
                              : m_tolerant_core_set.count(req.source_id) != 0;
    const size_t request_class = tolerant ? 0 : 1;
    s_class_demand_reads_served.at(request_class)++;
    s_class_read_latency.at(request_class) += latency;
    m_class_read_latency_histogram.at(request_class)[latency]++;
  }

 public:
  int s_num_read_requests = 0;
  int s_num_write_requests = 0;

 public:
  void init() override {
    RAMULATOR_PARSE_PARAM(m_clock_ratio, unsigned int, "clock_ratio").required();
    RAMULATOR_PARSE_PARAM(m_tolerant_cores, std::vector<int>, "tolerant_cores").default_val({});
    RAMULATOR_CREATE_CHILD(m_channel_mapper, IChannelMapper);

    // Each controller = one channel. DRAM config lives inside each controller.
    RAMULATOR_CREATE_CHILD_LIST(m_controllers, IController);
    if (m_controllers.empty()) {
      throw std::runtime_error("GenericDRAM requires at least one controller");
    }
    for (size_t i = 0; i < m_controllers.size(); i++) {
      dynamic_cast<Implementation*>(m_controllers[i])->set_id(fmt::format("Channel {}", i));
      m_controllers[i]->set_channel_id(static_cast<int>(i));
      m_controllers[i]->m_clock_ratio = m_clock_ratio;
    }

    // Setup channel mapper with controller info
    m_tx_bytes = m_controllers[0]->get_tx_bytes();
    const float reference_tck = m_controllers[0]->get_tCK();
    for (size_t i = 1; i < m_controllers.size(); i++) {
      if (m_controllers[i]->get_tx_bytes() != m_tx_bytes ||
          std::fabs(m_controllers[i]->get_tCK() - reference_tck) > 1e-6f) {
        throw std::runtime_error(
            "GenericDRAM requires identical transaction sizes and clocks across channels");
      }
    }
    m_channel_mapper->setup(static_cast<int>(m_controllers.size()), calc_log2(m_tx_bytes));

    m_stats.add("total_num_read_requests", s_num_read_requests);
    m_stats.add("total_num_write_requests", s_num_write_requests);
  };

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_num_cores = static_cast<size_t>(frontend->get_num_cores());
    s_demand_reads_served_per_core.assign(m_num_cores, 0);
    s_read_latency_per_core.assign(m_num_cores, 0);
    s_avg_read_latency_per_core.assign(m_num_cores, 0);
    s_p99_read_latency_per_core.assign(m_num_cores, 0);
    m_read_latency_histogram_per_core.resize(m_num_cores);
    s_read_latency_histogram_bins_per_core.resize(m_num_cores);
    s_read_latency_histogram_counts_per_core.resize(m_num_cores);
    for (int core : m_tolerant_cores) {
      if (core < 0 || core >= static_cast<int>(m_num_cores) ||
          !m_tolerant_core_set.insert(core).second) {
        throw std::runtime_error(
            "GenericDRAM tolerant_cores must be unique valid core ids");
      }
    }
    for (size_t core = 0; core < m_num_cores; core++) {
      m_stats.add(fmt::format("system_demand_reads_served_core_{}", core),
                  s_demand_reads_served_per_core.at(core));
      m_stats.add(fmt::format("system_read_latency_core_{}", core),
                  s_read_latency_per_core.at(core));
      m_stats.add(fmt::format("system_avg_read_latency_core_{}", core),
                  s_avg_read_latency_per_core.at(core));
      m_stats.add(fmt::format("system_p99_read_latency_core_{}", core),
                  s_p99_read_latency_per_core.at(core));
      m_stats.add(fmt::format("system_read_latency_histogram_bins_core_{}", core),
                  s_read_latency_histogram_bins_per_core.at(core));
      m_stats.add(fmt::format("system_read_latency_histogram_counts_core_{}", core),
                  s_read_latency_histogram_counts_per_core.at(core));
    }
    m_stats.add("system_tolerant_demand_reads_served", s_class_demand_reads_served[0]);
    m_stats.add("system_tolerant_read_latency", s_class_read_latency[0]);
    m_stats.add("system_tolerant_avg_read_latency", s_class_avg_read_latency[0]);
    m_stats.add("system_tolerant_p99_read_latency", s_class_p99_read_latency[0]);
    m_stats.add("system_tolerant_read_latency_histogram_bins", s_class_read_latency_histogram_bins[0]);
    m_stats.add("system_tolerant_read_latency_histogram_counts", s_class_read_latency_histogram_counts[0]);
    m_stats.add("system_reliable_demand_reads_served", s_class_demand_reads_served[1]);
    m_stats.add("system_reliable_read_latency", s_class_read_latency[1]);
    m_stats.add("system_reliable_avg_read_latency", s_class_avg_read_latency[1]);
    m_stats.add("system_reliable_p99_read_latency", s_class_p99_read_latency[1]);
    m_stats.add("system_reliable_read_latency_histogram_bins", s_class_read_latency_histogram_bins[1]);
    m_stats.add("system_reliable_read_latency_histogram_counts", s_class_read_latency_histogram_counts[1]);
  }

  bool send(Request& req) override {
    // Validate request size: must be set and fit within one transaction.
    if (req.size_bytes <= 0 || req.size_bytes > m_tx_bytes) {
      throw std::runtime_error(fmt::format(
          "Request size_bytes must be set by the frontend (got {}, tx_bytes = {}).",
          req.size_bytes, m_tx_bytes));
    }

    // Channel mapper sets req.addr_vec[0] and req.intra_channel_addr.
    // Controller::send() handles address mapping internally.
    m_channel_mapper->apply(req);
    int channel_id = req.addr_vec[0];
    if (channel_id < 0 || channel_id >= static_cast<int>(m_controllers.size())) {
      throw std::runtime_error("Channel mapper selected an out-of-range controller");
    }
    const auto original_callback = req.callback;
    if (req.type_id == Request::Type::Read) {
      req.callback = [this, original_callback](Request& completed) {
        record_read_completion(completed);
        if (original_callback) original_callback(completed);
      };
    }
    bool is_success = m_controllers[channel_id]->send(req);
    if (!is_success) req.callback = original_callback;

    if (is_success) {
      switch (req.type_id) {
        case Request::Type::Read: {
          s_num_read_requests++;
          break;
        }
        case Request::Type::Write: {
          s_num_write_requests++;
          break;
        }
      }
    }
    return is_success;
  };

  void tick() override {
    for (auto controller : m_controllers) {
      controller->tick();
    }
  };

  void reset_stats() override {
    s_num_read_requests = 0;
    s_num_write_requests = 0;
    std::fill(s_demand_reads_served_per_core.begin(), s_demand_reads_served_per_core.end(), 0);
    std::fill(s_read_latency_per_core.begin(), s_read_latency_per_core.end(), 0);
    std::fill(s_avg_read_latency_per_core.begin(), s_avg_read_latency_per_core.end(), 0);
    std::fill(s_p99_read_latency_per_core.begin(), s_p99_read_latency_per_core.end(), 0);
    for (auto& histogram : m_read_latency_histogram_per_core) histogram.clear();
    for (auto& histogram : s_read_latency_histogram_bins_per_core) histogram.clear();
    for (auto& histogram : s_read_latency_histogram_counts_per_core) histogram.clear();
    s_class_demand_reads_served.fill(0);
    s_class_read_latency.fill(0);
    s_class_avg_read_latency.fill(0);
    s_class_p99_read_latency.fill(0);
    for (auto& histogram : m_class_read_latency_histogram) histogram.clear();
    for (auto& histogram : s_class_read_latency_histogram_bins) histogram.clear();
    for (auto& histogram : s_class_read_latency_histogram_counts) histogram.clear();
  }

  void update_stats() override {
    for (size_t core = 0; core < m_num_cores; core++) {
      const size_t count = s_demand_reads_served_per_core.at(core);
      s_avg_read_latency_per_core.at(core) = count > 0
          ? static_cast<float>(s_read_latency_per_core.at(core)) / static_cast<float>(count)
          : 0;
      s_p99_read_latency_per_core.at(core) =
          percentile99(m_read_latency_histogram_per_core.at(core), count);
      sparsify(m_read_latency_histogram_per_core.at(core),
               s_read_latency_histogram_bins_per_core.at(core),
               s_read_latency_histogram_counts_per_core.at(core));
    }
    for (size_t request_class = 0; request_class < 2; request_class++) {
      const size_t count = s_class_demand_reads_served.at(request_class);
      s_class_avg_read_latency.at(request_class) = count > 0
          ? static_cast<float>(s_class_read_latency.at(request_class)) / static_cast<float>(count)
          : 0;
      s_class_p99_read_latency.at(request_class) =
          percentile99(m_class_read_latency_histogram.at(request_class), count);
      sparsify(m_class_read_latency_histogram.at(request_class),
               s_class_read_latency_histogram_bins.at(request_class),
               s_class_read_latency_histogram_counts.at(request_class));
    }
  }

  void finalize() override {
    update_stats();
  }

  int get_clock_ratio() override {
    return m_clock_ratio;
  }

  float get_tCK() override {
    if (!m_controllers.empty()) {
      return m_controllers[0]->get_tCK();
    }
    return -1.0f;
  }

  int get_tx_bytes() override {
    return m_tx_bytes;
  }

  bool is_idle() const override {
    return std::all_of(
        m_controllers.begin(), m_controllers.end(),
        [](const IController* controller) { return controller->is_idle(); });
  }
};

}  // namespace Ramulator
