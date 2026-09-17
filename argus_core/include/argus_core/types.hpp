
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace argus {

struct CleanCloud {
    std::vector<float> x, y, z;
    std::vector<float> intensity;
    std::vector<uint16_t> ring;
    std::vector<uint32_t> azimuth_idx;

    std::vector<uint32_t> raw_idx;
    std::vector<uint32_t> no_return_raw;
    std::vector<uint8_t> ground_mask;

    uint32_t n_raw = 0;
    uint32_t rings = 0;
    uint32_t az_steps = 0;
    uint8_t echo_multiplier = 1;
    double stamp_s = 0.0;
    std::string frame_id;

    size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
};

struct RangeImage {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<float> range;
    std::vector<float> intensity;
    std::vector<uint8_t> no_return_mask;

    size_t idx(uint32_t az, uint32_t ring) const { return static_cast<size_t>(az) * height + ring; }

    bool valid() const {
        return width > 0 && height > 0 && range.size() == static_cast<size_t>(width) * height &&
               no_return_mask.size() == range.size();
    }
};

struct AnomalySet {
    std::vector<uint32_t> indices;
    std::vector<float> score;
    std::vector<uint32_t> cells;
    std::vector<float> cells_score;
    std::string source;
};

} // namespace argus
