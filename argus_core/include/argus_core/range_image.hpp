
#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "argus_core/types.hpp"

namespace argus {

struct RangeImageParams {
    uint32_t rings_fallback = 128;
    uint32_t az_steps_fallback = 2400;
    bool prefer_ring_field = true;
    uint32_t max_rings = 256;
};

RangeImage build_range_image(const CleanCloud& cloud, const RangeImageParams& params);

float sample(const RangeImage& ri, int az, int ring);

uint32_t neighbors8(const RangeImage& ri, uint32_t az, uint32_t ring,
                    std::array<std::pair<uint32_t, uint32_t>, 8>& out);

RangeImage azimuth_median_filter(const RangeImage& ri, uint32_t half_window);

uint32_t detect_rings(const CleanCloud& cloud, uint32_t max_rings);

} // namespace argus
