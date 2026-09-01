#ifndef RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_TRACE_H
#define RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_TRACE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace Ramulator {

// ChampSim input_instr ABI, verified against ChampSim/ChampSim commit
// 51588e1d6f97875fe8de1a3621d28668bff83fcf.
struct ChampSimTraceRecord {
  uint64_t ip = 0;
  uint8_t is_branch = 0;
  uint8_t branch_taken = 0;
  std::array<uint8_t, 2> destination_registers{};
  std::array<uint8_t, 4> source_registers{};
  std::array<uint64_t, 2> destination_memory{};
  std::array<uint64_t, 4> source_memory{};
};

static_assert(sizeof(ChampSimTraceRecord) == 64, "ChampSim input_instr must be 64 bytes");

class ChampSimTrace {
 private:
  std::string m_path;
  FILE* m_stream = nullptr;
  bool m_is_pipe = false;
  size_t m_records_this_pass = 0;
  size_t m_records_read = 0;
  size_t m_passes_completed = 0;

  static std::string shell_quote(const std::string& value);
  void open_stream();
  void close_stream(bool check_status);

 public:
  explicit ChampSimTrace(std::string path);
  ChampSimTrace(const ChampSimTrace&) = delete;
  ChampSimTrace& operator=(const ChampSimTrace&) = delete;
  ~ChampSimTrace();

  ChampSimTraceRecord next();
  size_t records_read() const { return m_records_read; }
  size_t passes_completed() const { return m_passes_completed; }
};

}  // namespace Ramulator

#endif  // RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_TRACE_H
