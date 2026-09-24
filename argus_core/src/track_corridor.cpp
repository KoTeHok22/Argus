#include "argus_core/track_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus {

TrackCorridor::TrackCorridor(TrackCorridorParams params) : params_(params) {}

bool TrackCorridor::set_centerline(const std::vector<Eigen::Vector2f>& centerline) {
    segments_.clear();
    if (centerline.size() < 2 || !std::isfinite(params_.half_width_m) ||
        !std::isfinite(params_.max_segment_m) || !std::isfinite(params_.max_pose_offset_m) ||
        params_.half_width_m <= 0.0f || params_.max_segment_m <= 0.0f ||
        params_.max_pose_offset_m < 0.0f) {
        return false;
    }
    float arc = 0.0f;
    for (size_t i = 1; i < centerline.size(); ++i) {
        const Eigen::Vector2f delta = centerline[i] - centerline[i - 1];
        const float length = delta.norm();
        if (!centerline[i - 1].allFinite() || !centerline[i].allFinite() ||
            !std::isfinite(length) || length < 0.01f || length > params_.max_segment_m) {
            segments_.clear();
            return false;
        }
        segments_.push_back({centerline[i - 1], delta / length, length, arc});
        arc += length;
    }
    return true;
}

bool TrackCorridor::project(const Eigen::Vector2f& point, float& arc, float& offset) const {
    if (!point.allFinite() || segments_.empty()) {
        return false;
    }
    float best = std::numeric_limits<float>::infinity();
    for (const auto& segment : segments_) {
        const float along =
            std::clamp((point - segment.start).dot(segment.direction), 0.0f, segment.length);
        const float distance = (point - segment.start - along * segment.direction).norm();
        if (distance < best) {
            best = distance;
            arc = segment.arc_start + along;
        }
    }
    offset = best;
    return true;
}

bool TrackCorridor::contains(const Eigen::Vector2f& sensor, const Eigen::Vector2f& candidate,
                             float& distance_ahead) const {
    float sensor_arc = 0.0f;
    float sensor_offset = 0.0f;
    float candidate_arc = 0.0f;
    float candidate_offset = 0.0f;
    if (!project(sensor, sensor_arc, sensor_offset) ||
        !project(candidate, candidate_arc, candidate_offset) ||
        sensor_offset > params_.max_pose_offset_m || candidate_offset > params_.half_width_m) {
        return false;
    }
    const Segment& first = segments_.front();
    const Segment& last = segments_.back();
    if (candidate_arc <= sensor_arc) {
        return false;
    }
    if ((candidate - first.start).dot(first.direction) < 0.0f ||
        (candidate - last.start).dot(last.direction) > last.length) {
        return false;
    }
    for (const auto& segment : segments_) {
        const float along = (candidate - segment.start).dot(segment.direction);
        if (along >= 0.0f && along <= segment.length &&
            (candidate - segment.start - along * segment.direction).norm() <=
                params_.half_width_m) {
            distance_ahead = segment.arc_start + along - sensor_arc;
            return distance_ahead > 0.0f;
        }
    }
    return false;
}

} // namespace argus
