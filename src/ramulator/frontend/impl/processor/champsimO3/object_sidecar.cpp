#include "ramulator/frontend/impl/processor/champsimO3/object_sidecar.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>

namespace Ramulator {

namespace fs = std::filesystem;

std::string ObjectSidecar::shell_quote(const std::string& value) {
  std::string quoted = "'";
  for (char ch : value) quoted += ch == '\'' ? "'\\''" : std::string(1, ch);
  return quoted + "'";
}

ObjectSidecar::ObjectSidecar(std::string path) : m_path(std::move(path)) {
  if (!fs::is_regular_file(m_path)) {
    throw std::runtime_error(fmt::format("object sidecar {} is not a regular file", m_path));
  }
  load();
}

void ObjectSidecar::load() {
  const bool compressed = m_path.ends_with(".xz");
  FILE* stream = compressed ? popen(("xz -dc -- " + shell_quote(m_path)).c_str(), "r")
                            : std::fopen(m_path.c_str(), "r");
  if (stream == nullptr) throw std::runtime_error("could not open object sidecar");
  char* line = nullptr;
  size_t capacity = 0;
  ssize_t length = 0;
  size_t line_number = 0;
  uint64_t previous_sequence = 0;
  bool header_seen = false;
  bool end_seen = false;
  while ((length = getline(&line, &capacity, stream)) >= 0) {
    line_number++;
    std::string text(line, static_cast<size_t>(length));
    if (!text.empty() && text.back() == '\n') text.pop_back();
    if (text.empty()) continue;
    std::vector<std::string> fields;
    std::stringstream parser(text);
    std::string field;
    while (std::getline(parser, field, '\t')) fields.push_back(field);
    if (fields.empty()) continue;
    if (fields[0] == "H") {
      if (line_number != 1 || header_seen || fields.size() != 5 || fields[1] != "1" ||
          fields[2] != "champsim-input-instr-64" ||
          fields[3] != "global-record-order" || fields[4] != "reliable-default") {
        throw std::runtime_error("unsupported or malformed object sidecar header");
      }
      header_seen = true;
      continue;
    }
    if (!header_seen || end_seen) throw std::runtime_error("malformed object sidecar ordering");
    if (fields[0] == "E") {
      if (fields.size() != 5) throw std::runtime_error("malformed object sidecar E record");
      m_trace_records = std::stoull(fields[1]);
      end_seen = true;
      continue;
    }
    if (fields[0] == "B" || fields[0] == "D") {
      if (fields.size() != 3) throw std::runtime_error("malformed object sidecar ROI record");
      Event event;
      event.op = fields[0][0];
      event.sequence = std::stoull(fields[1]);
      if (event.sequence < previous_sequence)
        throw std::runtime_error("object sidecar sequence regressed");
      previous_sequence = event.sequence;
      m_events.push_back(event);
      continue;
    }
    Event event;
    if (fields[0] == "A") {
      if (fields.size() != 8) throw std::runtime_error("malformed object sidecar A record");
      event.op = 'A';
      event.sequence = std::stoull(fields[1]);
      event.object_id = std::stoull(fields[3]);
      event.address = std::stoull(fields[4], nullptr, 16);
      event.size = std::stoull(fields[5]);
      event.memory_class = fields[6] == "tolerant" ? 1 : fields[6] == "reliable" ? 0 : -1;
      if (event.object_id == 0 || event.address == 0 || event.size == 0 ||
          event.memory_class < 0 || event.address > std::numeric_limits<uint64_t>::max() - event.size)
        throw std::runtime_error("invalid object sidecar allocation");
    } else if (fields[0] == "F") {
      if (fields.size() != 5) throw std::runtime_error("malformed object sidecar F record");
      event.op = 'F';
      event.sequence = std::stoull(fields[1]);
      event.object_id = std::stoull(fields[3]);
      event.address = std::stoull(fields[4], nullptr, 16);
    } else {
      throw std::runtime_error("unknown object sidecar record");
    }
    if (event.sequence < previous_sequence)
      throw std::runtime_error("object sidecar sequence regressed");
    previous_sequence = event.sequence;
    m_events.push_back(event);
  }
  std::free(line);
  int status = compressed ? pclose(stream) : std::fclose(stream);
  if (status != 0) throw std::runtime_error("object sidecar reader failed");
  if (!header_seen || !end_seen || m_trace_records == 0)
    throw std::runtime_error("object sidecar is incomplete or empty");
  for (const auto& event : m_events)
    if (event.sequence > m_trace_records)
      throw std::runtime_error("object sidecar event exceeds trace length");
}

void ObjectSidecar::reset_pass(size_t pass) {
  m_current_pass = pass;
  m_next_event = 0;
  m_active.clear();
  m_active_order.clear();
}

void ObjectSidecar::apply(const Event& event) {
  if (event.op == 'A') {
    if (m_active.count(event.object_id))
      throw std::runtime_error("object sidecar allocates an active object twice");
    m_active.emplace(event.object_id,
                     Object{event.object_id, event.address, event.size, event.memory_class});
    m_active_order.push_back(event.object_id);
  } else if (event.op == 'F') {
    auto found = m_active.find(event.object_id);
    if (found == m_active.end() || found->second.address != event.address)
      throw std::runtime_error("object sidecar frees an inactive or mismatched object");
    m_active.erase(found);
  }
}

void ObjectSidecar::advance(size_t pass, size_t record_index) {
  if (pass != m_current_pass) reset_pass(pass);
  while (m_next_event < m_events.size() &&
         m_events[m_next_event].sequence <= record_index) {
    apply(m_events[m_next_event++]);
  }
}

int ObjectSidecar::classify(uint64_t address) const {
  for (auto iterator = m_active_order.rbegin(); iterator != m_active_order.rend(); ++iterator) {
    auto found = m_active.find(*iterator);
    if (found == m_active.end()) continue;
    const auto& object = found->second;
    if (object.address <= address && address < object.address + object.size)
      return object.memory_class;
  }
  return 0;
}

}  // namespace Ramulator
