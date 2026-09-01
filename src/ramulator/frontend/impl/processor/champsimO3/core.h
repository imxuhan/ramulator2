#ifndef RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_CORE_H
#define RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_CORE_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ramulator/base/request.h"
#include "ramulator/base/type.h"
#include "ramulator/frontend/impl/processor/champsimO3/object_sidecar.h"
#include "ramulator/frontend/impl/processor/champsimO3/trace.h"
#include "ramulator/translation/i_translation.h"

namespace Ramulator {

class SimpleO3LLC;

class ChampSimO3Core {
  class InstWindow {
    struct Entry {
      uint64_t instruction_id = 0;
      int pending_memory = 0;
      bool valid = false;
    };

    int m_ipc;
    int m_depth;
    int m_load = 0;
    int m_head_idx = 0;
    int m_tail_idx = 0;
    std::vector<Entry> m_entries;

   public:
    InstWindow(int ipc, int depth);
    bool is_full() const { return m_load == m_depth; }
    void insert(uint64_t instruction_id, int pending_memory);
    void complete(uint64_t instruction_id);
    int retire();
  };

  const Clk_t& m_clk;
  int m_id;
  int m_ipc;
  size_t m_num_expected_insts;
  size_t m_warmup_insts = 0;
  size_t m_roi_insts = 0;
  size_t m_roi_start_insts = 0;
  size_t m_roi_start_clk = 0;
  bool m_roi_started = false;
  ChampSimTrace m_trace;
  std::unique_ptr<ObjectSidecar> m_sidecar;
  InstWindow m_window;
  ITranslation* m_translation;
  SimpleO3LLC* m_llc;
  std::function<void(Request&)> m_callback;

  ChampSimTraceRecord m_current;
  uint64_t m_current_id = 0;
  uint64_t m_next_instruction_id = 0;
  size_t m_next_load = 0;
  size_t m_next_store = 0;
  bool m_current_inserted = false;

  int current_memory_count() const;
  void advance_trace();

 public:
  bool reached_expected_num_insts = false;
  bool reached_warmup = false;
  size_t s_insts_retired = 0;
  size_t s_cycles_recorded = 0;
  size_t s_trace_records = 0;

  ChampSimO3Core(const Clk_t& clk, int id, int ipc, int depth, size_t warmup_insts,
                 size_t num_expected_insts,
                 const std::string& trace_path, const std::string& sidecar_path,
                 ITranslation* translation, SimpleO3LLC* llc);
  void set_callback(std::function<void(Request&)> callback) { m_callback = std::move(callback); }
  void tick();
  void receive(Request& req);
  void begin_roi();
  size_t roi_insts_retired() const;
  size_t trace_passes_completed() const { return m_trace.passes_completed(); }
};

}  // namespace Ramulator

#endif  // RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_CORE_H
