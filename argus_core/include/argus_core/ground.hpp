#pragma once

#include <cstdint>
#include <memory>

#include "argus_core/gauge.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct GroundParams {
    bool enabled = true;
    float sensor_height = 1.20f;
    float rail_zone_m = 2.0f;
    float rail_max_height = 0.45f;
    float max_ground_z_rel = 0.60f;
    float min_range = 1.0f;
    float max_range = 80.0f;
    bool enable_RNR = true;
    bool enable_TGR = true;
    ForwardAxis forward_axis = ForwardAxis::NegY;
};

struct GroundStats {
    uint32_t n_ground = 0;
    uint32_t n_nonground = 0;
    uint32_t n_wall_guarded = 0;
    uint32_t n_rail_fallback = 0;
};

class GroundSegmenter {
public:
    explicit GroundSegmenter(const GroundParams& p);
    ~GroundSegmenter();
    GroundSegmenter(GroundSegmenter&&) noexcept;
    GroundSegmenter& operator=(GroundSegmenter&&) noexcept;
    GroundSegmenter(const GroundSegmenter&) = delete;
    GroundSegmenter& operator=(const GroundSegmenter&) = delete;

    void apply(CleanCloud& cloud, GroundStats* stats = nullptr);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace argus
