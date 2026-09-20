
#pragma once

#include <cstdint>
#include <deque>

namespace argus {

struct SensorHealthParams {
    float max_no_return_ratio = 0.6f;
    float min_valid_ratio = 0.10f;
    float valid_drop_factor = 0.5f;
    uint32_t warmup_frames = 30;
    uint32_t bad_frames_to_raise = 20;
    uint32_t good_frames_to_clear = 20;
};

struct SensorHealthStatus {
    bool ok = true;
    float no_return_ratio = 0.0f;
    float valid_ratio = 0.0f;
    float baseline_valid = 0.0f;
    uint32_t bad_streak = 0;
    bool baseline_ready = false;
};

class SensorHealthMonitor {
public:
    explicit SensorHealthMonitor(const SensorHealthParams& p);

    SensorHealthStatus update(uint64_t n_raw, uint64_t n_valid, uint64_t n_no_return);

private:
    SensorHealthParams p_;
    std::deque<float> baseline_window_;
    float baseline_valid_ = 0.0f;
    uint32_t bad_streak_ = 0;
    uint32_t good_streak_ = 0;
    uint32_t frames_ = 0;
    bool raised_ = false;
};

} // namespace argus
