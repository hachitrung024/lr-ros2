#pragma once
#include <cmath>
#include <cstdint>

namespace lr_mavros_bridge {
// Status is published even for rejected observations. Count only low-RTT
// observations agreeing with the estimate actually used by MAVROS.
struct SyncHealth {
  unsigned samples = 0;
  uint64_t epoch = 0, remote = 0;
  int64_t seen = 0, wall = 0, steady = 0, offset = 0;
  double rtt = 0;
  void reset() { samples = 0; seen = 0; remote = 0; ++epoch; }
  bool ready(int64_t now) const { return samples >= 30 && seen > 0 && now - seen < 2000000000LL; }
  void expire(int64_t now) { if (seen && now - seen >= 2000000000LL) reset(); }
  void observe(int64_t now, int64_t wall_now, uint64_t remote_now,
               int64_t observed, int64_t estimated, int64_t applied, double round_trip) {
    expire(now);
    if ((remote && remote_now < remote) ||
        (steady && std::abs(double(wall_now - wall) - double(now - steady)) > 100000000.) ||
        (samples && (std::abs(double(observed) - double(offset)) > 100000000. ||
                     std::abs(double(applied) - double(offset)) > 100000000.))) reset();
    remote = remote_now; wall = wall_now; steady = now; rtt = round_trip;
    if (!remote_now || !std::isfinite(rtt) || rtt < 0 || rtt >= 20. ||
        std::abs(double(observed) - double(estimated)) > 10000000. ||
        std::abs(double(applied) - double(estimated)) > 1000000.) return;
    offset = applied; seen = now; if (samples < 30) ++samples;
  }
};

// SYSTEM_TIME explicitly relates the FCU's UTC and boot clocks. It does not
// prove that a camera shares that UTC clock; that is an operator assertion.
struct UtcHealth {
  unsigned samples = 0;
  int64_t utc_minus_boot = 0, seen = 0;
  uint32_t boot_ms = 0;
  void reset() { samples = 0; seen = 0; }
  bool ready(int64_t now) const { return samples >= 5 && seen && now-seen < 2000000000LL; }
  void observe(int64_t now, uint64_t unix_us, uint32_t boot) {
    if (!unix_us || unix_us > uint64_t(INT64_MAX / 1000)) { reset(); return; }
    const int64_t relation = int64_t(unix_us * 1000) - int64_t(boot) * 1000000;
    if (seen && (now-seen >= 2000000000LL || boot < boot_ms ||
        std::abs(double(relation)-double(utc_minus_boot)) > 5000000.)) reset();
    utc_minus_boot = relation; seen = now; boot_ms = boot;
    if (samples < 5) ++samples;
  }
};
}
