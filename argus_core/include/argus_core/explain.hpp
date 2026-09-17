
#pragma once

#include <string>

#include "argus_core/clustering.hpp"
#include "argus_core/tracking.hpp"

namespace argus {

struct AlertReason {
    float free_space_confidence = -1.0f;
    float geometry_deviation_m = -1.0f;
    float geometry_sigma_m = -1.0f;
    uint8_t votes_free_space = 0;
    uint8_t votes_no_return = 0;
    uint8_t votes_geometry = 0;
    uint8_t total_votes = 0;
};

std::string format_alert(const Track& track, const AlertReason& reason);

std::string format_clear(uint64_t frames_processed, float fps);

} // namespace argus
