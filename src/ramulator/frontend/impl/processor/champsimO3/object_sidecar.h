#ifndef RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_OBJECT_SIDECAR_H
#define RAMULATOR_FRONTEND_PROCESSOR_CHAMPSIMO3_OBJECT_SIDECAR_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace Ramulator {

class ObjectSidecar {
  struct Event {
    char op = 0;
    uint64_t sequence = 0;
    uint64_t object_id = 0;
    uint64_t address = 0;
    uint64_t size = 0;
    int memory_class = 0;
  };
  struct Object {
    uint64_t id = 0;
    uint64_t address = 0;
    uint64_t size = 0;
    int memory_class = 0;
  };

  std::string m_path;
  std::vector<Event> m_events;
  std::unordered_map<uint64_t, Object> m_active;
  std::vector<uint64_t> m_active_order;
  size_t m_next_event = 0;
  size_t m_current_pass = static_cast<size_t>(-1);
  uint64_t m_trace_records = 0;

  static std::string shell_quote(const std::string& value);
  void load();
  void reset_pass(size_t pass);
  void apply(const Event& event);

 public:
  explicit ObjectSidecar(std::string path);
  void advance(size_t pass, size_t record_index);
  int classify(uint64_t address) const;
  uint64_t trace_records() const { return m_trace_records; }
};

}  // namespace Ramulator

#endif
