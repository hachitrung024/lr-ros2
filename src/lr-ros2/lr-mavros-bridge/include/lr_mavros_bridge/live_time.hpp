#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>
#include <nlohmann/json.hpp>

namespace lr_mavros_bridge {
using Json = nlohmann::json;

struct SourceSession {
  std::string id;
  uint64_t sequence = 0;
  int64_t record_ns = 0, boot_ns = 0;
  std::unordered_set<std::string> retired;

  bool accept(const uint8_t *data, size_t size) {
    if (size != 48 || std::string(reinterpret_cast<const char *>(data), 4) != "LRST" ||
        data[4] != 1 || data[5] || data[6] || data[7]) return false;
    auto read = [data](int start) {
      uint64_t value = 0;
      for (int i = 0; i < 8; ++i) value |= uint64_t(data[start + i]) << (8 * i);
      return value;
    };
    std::ostringstream name;
    for (int i = 8; i < 24; ++i) name << std::hex << std::setw(2) << std::setfill('0') << int(data[i]);
    const auto incoming = name.str();
    const auto seq = read(24), record = read(32), boot = read(40);
    if (!record || record >= (uint64_t(1) << 63) || !boot ||
        boot / 1000000 >= UINT32_MAX || retired.count(incoming)) return false;
    if (incoming == id) {
      if (seq <= sequence || int64_t(record) != record_ns || int64_t(boot) != boot_ns) return false;
    } else {
      if (!id.empty()) retired.insert(id);
      id = incoming;
    }
    sequence = seq;
    record_ns = int64_t(record);
    boot_ns = int64_t(boot);
    return true;
  }
};
}  // namespace lr_mavros_bridge
