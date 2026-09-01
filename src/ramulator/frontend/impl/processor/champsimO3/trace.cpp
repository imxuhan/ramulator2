#include "ramulator/frontend/impl/processor/champsimO3/trace.h"

#include <filesystem>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>

namespace Ramulator {

namespace fs = std::filesystem;

std::string ChampSimTrace::shell_quote(const std::string& value) {
  std::string quoted = "'";
  for (char ch : value) {
    if (ch == '\'') {
      quoted += "'\\''";
    } else {
      quoted += ch;
    }
  }
  quoted += "'";
  return quoted;
}

ChampSimTrace::ChampSimTrace(std::string path) : m_path(std::move(path)) {
  if (!fs::is_regular_file(m_path)) {
    throw std::runtime_error(fmt::format("ChampSim trace {} does not exist or is not a regular file", m_path));
  }
  open_stream();
}

ChampSimTrace::~ChampSimTrace() {
  try {
    close_stream(false);
  } catch (...) {
  }
}

void ChampSimTrace::open_stream() {
  std::string command;
  if (m_path.ends_with(".xz")) {
    command = "xz -dc -- " + shell_quote(m_path);
  } else if (m_path.ends_with(".gz")) {
    command = "gzip -dc -- " + shell_quote(m_path);
  }

  if (command.empty()) {
    m_stream = std::fopen(m_path.c_str(), "rb");
    m_is_pipe = false;
  } else {
    m_stream = popen(command.c_str(), "r");
    m_is_pipe = true;
  }
  if (m_stream == nullptr) {
    throw std::runtime_error(fmt::format("Could not open ChampSim trace {}", m_path));
  }
  m_records_this_pass = 0;
}

void ChampSimTrace::close_stream(bool check_status) {
  if (m_stream == nullptr) {
    return;
  }
  int status = m_is_pipe ? pclose(m_stream) : std::fclose(m_stream);
  m_stream = nullptr;
  if (check_status && status != 0) {
    throw std::runtime_error(fmt::format("Decompressor failed for ChampSim trace {}", m_path));
  }
}

ChampSimTraceRecord ChampSimTrace::next() {
  ChampSimTraceRecord record;
  size_t bytes = std::fread(&record, 1, sizeof(record), m_stream);
  if (bytes == sizeof(record)) {
    m_records_this_pass++;
    m_records_read++;
    return record;
  }
  if (bytes != 0) {
    throw std::runtime_error(fmt::format(
        "ChampSim trace {} is truncated: final record has {} of 64 bytes", m_path, bytes));
  }
  if (std::ferror(m_stream)) {
    throw std::runtime_error(fmt::format("Failed while reading ChampSim trace {}", m_path));
  }
  if (m_records_this_pass == 0) {
    throw std::runtime_error(fmt::format("ChampSim trace {} is empty", m_path));
  }

  close_stream(true);
  m_passes_completed++;
  open_stream();
  bytes = std::fread(&record, 1, sizeof(record), m_stream);
  if (bytes != sizeof(record)) {
    throw std::runtime_error(fmt::format("Could not restart ChampSim trace {}", m_path));
  }
  m_records_this_pass++;
  m_records_read++;
  return record;
}

}  // namespace Ramulator
