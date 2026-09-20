
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "argus_core/anomaly.hpp"

namespace argus {

TemporalResidualDetector::TemporalResidualDetector(const TemporalResidualParams& params)
    : params_(params) {}

void TemporalResidualDetector::reset() {
    history_.clear();
    width_ = 0;
    height_ = 0;
}

AnomalySet TemporalResidualDetector::detect(const CleanCloud& cloud, const RangeImage& ri) {
    AnomalySet out;
    out.source = name();
    if (!ri.valid() || params_.window_frames == 0) {
        reset();
        return out;
    }
    if (width_ != ri.width || height_ != ri.height) {
        history_.clear();
        width_ = ri.width;
        height_ = ri.height;
    }
    if (history_.size() >= 2) {
        std::vector<uint32_t> cell_to_cloud(ri.range.size(), std::numeric_limits<uint32_t>::max());
        for (size_t index = 0; index < cloud.size(); ++index) {
            const uint32_t cell =
                cloud.raw_idx.empty() ? static_cast<uint32_t>(index) : cloud.raw_idx[index];
            if (cell < cell_to_cloud.size()) {
                cell_to_cloud[cell] = static_cast<uint32_t>(index);
            }
        }
        std::vector<float> values;
        values.reserve(history_.size());
        for (size_t cell = 0; cell < ri.range.size(); ++cell) {
            const float current = ri.range[cell];
            const uint32_t cloud_index = cell_to_cloud[cell];
            if (!std::isfinite(current) || cloud_index == std::numeric_limits<uint32_t>::max() ||
                current < params_.min_range_m || current > params_.max_range_m) {
                continue;
            }
            values.clear();
            for (const auto& frame : history_) {
                if (std::isfinite(frame[cell])) {
                    values.push_back(frame[cell]);
                }
            }
            if (values.size() < 2) {
                continue;
            }
            std::sort(values.begin(), values.end());
            const float baseline = values[values.size() / 2];
            const float residual = baseline - current;
            if (residual <= params_.residual_threshold_m) {
                continue;
            }
            out.indices.push_back(cloud_index);
            out.score.push_back(std::min(1.0f, residual / (2.0f * params_.residual_threshold_m)));
        }
    }
    history_.push_back(ri.range);
    if (history_.size() > params_.window_frames) {
        history_.erase(history_.begin());
    }
    return out;
}

} // namespace argus
