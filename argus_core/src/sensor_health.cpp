
#include "argus_core/sensor_health.hpp"

#include <algorithm>

namespace argus {

namespace {
constexpr size_t kBaselineWindow = 100;
}

SensorHealthMonitor::SensorHealthMonitor(const SensorHealthParams& p) : p_(p) {}

SensorHealthStatus SensorHealthMonitor::update(uint64_t n_raw, uint64_t n_valid,
                                               uint64_t n_no_return) {
    SensorHealthStatus st;
    ++frames_;

    const uint64_t rays = n_valid + n_no_return;
    const uint64_t expected = rays > 0 ? rays : (n_raw > 0 ? n_raw : 1);
    st.no_return_ratio = static_cast<float>(n_no_return) / static_cast<float>(expected);
    st.valid_ratio = static_cast<float>(n_valid) / static_cast<float>(expected);

    if (!raised_ && bad_streak_ == 0 && n_valid > 0) {
        baseline_window_.push_back(static_cast<float>(n_valid));
        if (baseline_window_.size() > kBaselineWindow) {
            baseline_window_.pop_front();
        }
        float sum = 0.0f;
        for (float v : baseline_window_) {
            sum += v;
        }
        baseline_valid_ = sum / static_cast<float>(baseline_window_.size());
    }
    st.baseline_valid = baseline_valid_;
    st.baseline_ready = frames_ >= p_.warmup_frames && !baseline_window_.empty();

    bool bad = false;
    if (st.no_return_ratio > p_.max_no_return_ratio) {
        bad = true;
    }
    if (st.valid_ratio < p_.min_valid_ratio) {
        bad = true;
    }
    if (st.baseline_ready && baseline_valid_ > 0.0f &&
        static_cast<float>(n_valid) < baseline_valid_ * p_.valid_drop_factor) {
        bad = true;
    }

    if (bad) {
        good_streak_ = 0;
        ++bad_streak_;
        if (!raised_ && bad_streak_ >= p_.bad_frames_to_raise) {
            raised_ = true;
        }
    } else {
        bad_streak_ = 0;
        ++good_streak_;
        if (raised_ && good_streak_ >= p_.good_frames_to_clear) {
            raised_ = false;
            good_streak_ = 0;
        }
    }

    st.ok = !raised_;
    st.bad_streak = bad_streak_;
    return st;
}

} // namespace argus
