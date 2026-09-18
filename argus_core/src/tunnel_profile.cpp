
#include "argus_core/tunnel_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus {

TunnelProfile::TunnelProfile(const TunnelProfileParams& p) : p_(p) {}

void TunnelProfile::reset() {
    expected_.clear();
    variance_.clear();
    az_steps_ = 0;
    rings_ = 0;
    updates_ = 0;
    valid_ = false;
}

float TunnelProfile::blend(float previous, float observed, float alpha) {
    return previous + alpha * (observed - previous);
}

void TunnelProfile::update(const RangeImage& ri, bool clean_frame) {
    if (!p_.enabled || !ri.valid()) {
        return;
    }
    if (az_steps_ != ri.width || rings_ != ri.height) {
        reset();
        az_steps_ = ri.width;
        rings_ = ri.height;
        expected_.assign(static_cast<size_t>(az_steps_) * rings_,
                         std::numeric_limits<float>::quiet_NaN());
        variance_.assign(expected_.size(), 0.0f);
    }
    if (!clean_frame) {
        return;
    }

    const float alpha =
        p_.alpha_steady + (p_.alpha_initial - p_.alpha_steady) *
                              std::exp(-static_cast<float>(updates_) /
                                       static_cast<float>(std::max<uint32_t>(1, p_.warmup_frames)));
    const float adaptive = std::clamp(alpha, 0.0f, 1.0f);

    bool any_observation = false;
    for (size_t i = 0; i < expected_.size(); ++i) {
        const float v = ri.range[i];
        if (!std::isfinite(v) || v < p_.min_range || v > p_.max_range) {
            continue;
        }
        any_observation = true;
        if (!std::isfinite(expected_[i])) {
            expected_[i] = v;
            variance_[i] = 0.0f;
            continue;
        }
        const float diff = v - expected_[i];
        expected_[i] = blend(expected_[i], v, adaptive);
        variance_[i] = blend(variance_[i], diff * diff, adaptive);
    }
    if (!any_observation) {
        return;
    }
    ++updates_;
    valid_ = true;
}

float TunnelProfile::expected_range(uint32_t az, uint32_t ring) const {
    if (az >= az_steps_ || ring >= rings_) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return expected_[static_cast<size_t>(az) * rings_ + ring];
}

float TunnelProfile::deviation(const RangeImage& ri, uint32_t az, uint32_t ring) const {
    if (!valid_ || !ri.valid() || az >= az_steps_ || ring >= rings_) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const size_t idx = static_cast<size_t>(az) * rings_ + ring;
    const float observed = ri.range[idx];
    const float expected = expected_[idx];
    if (!std::isfinite(observed) || !std::isfinite(expected)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return expected - observed;
}

float TunnelProfile::median_deviation(const RangeImage& ri) const {
    if (!valid_ || !ri.valid() || az_steps_ != ri.width || rings_ != ri.height) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    std::vector<float> deviations;
    deviations.reserve(expected_.size());
    for (size_t i = 0; i < expected_.size(); ++i) {
        const float observed = ri.range[i];
        const float expected = expected_[i];
        if (!std::isfinite(observed) || !std::isfinite(expected)) {
            continue;
        }
        deviations.push_back(std::fabs(expected - observed));
    }
    if (deviations.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const size_t mid = deviations.size() / 2;
    std::sort(deviations.begin(), deviations.end());
    return deviations[mid];
}

float TunnelProfile::p95_deviation(const RangeImage& ri) const {
    if (!valid_ || !ri.valid() || az_steps_ != ri.width || rings_ != ri.height) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    std::vector<float> deviations;
    deviations.reserve(expected_.size());
    for (size_t i = 0; i < expected_.size(); ++i) {
        const float observed = ri.range[i];
        const float expected = expected_[i];
        if (!std::isfinite(observed) || !std::isfinite(expected)) {
            continue;
        }
        deviations.push_back(std::fabs(expected - observed));
    }
    if (deviations.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const size_t idx = (deviations.size() * 95) / 100;
    std::sort(deviations.begin(), deviations.end());
    return deviations[std::min(idx, deviations.size() - 1)];
}

} // namespace argus
