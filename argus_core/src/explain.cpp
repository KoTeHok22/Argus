// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0

#include "argus_core/explain.hpp"

#include <cstdio>
#include <cstdint>

namespace argus {

std::string format_alert(const Track& track, const AlertReason& r) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "ALERT track=%u votes: free_space=%u no_return=%u "
                "geometry=%u (%u/3)\n"
                "  position    : x=%.2f y=%.2f z=%.2f (lidar frame)\n"
                "  range       : nearest=%.1fm\n"
                "  extent      : %.2f m3\n"
                "  points persistence: hits=%u misses=%u\n"
                "  ttc         : %.1f s\n",
                track.id, r.votes_free_space, r.votes_no_return,
                r.votes_geometry, r.total_votes, track.state(0),
                track.state(1), track.state(2), track.nearest_range,
                track.volume, track.hits, track.misses, track.ttc);
  return std::string(buf);
}

std::string format_clear(uint64_t frames_processed, float fps) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "CLEAR frames=%llu fps=%.1f",
                static_cast<unsigned long long>(frames_processed),
                static_cast<double>(fps));
  return std::string(buf);
}

}  // namespace argus
