
#pragma once

#include <cstdint>
#include <vector>

#include "argus_core/range_image.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct TunnelProfileParams {
    bool enabled = true;
    float alpha_initial = 0.30f;
    float alpha_steady = 0.02f;
    uint32_t warmup_frames = 200;
    float max_range = 150.0f;
    float min_range = 2.0f;
};

class TunnelProfile {
public:
    explicit TunnelProfile(const TunnelProfileParams& p);

    void update(const RangeImage& ri, bool clean_frame);

    void reset();

    bool valid() const { return valid_; }

    uint32_t updates() const { return updates_; }

    float expected_range(uint32_t az, uint32_t ring) const;

    float deviation(const RangeImage& ri, uint32_t az, uint32_t ring) const;

    float median_deviation(const RangeImage& ri) const;

    float p95_deviation(const RangeImage& ri) const;

    uint32_t width() const { return az_steps_; }

    uint32_t height() const { return rings_; }

private:
    static float blend(float previous, float observed, float alpha);

    TunnelProfileParams p_;
    std::vector<float> expected_;
    std::vector<float> variance_;
    uint32_t az_steps_ = 0;
    uint32_t rings_ = 0;
    uint32_t updates_ = 0;
    bool valid_ = false;
};

} // namespace argus
