#pragma once

#include <Eigen/Core>

#include <vector>

namespace argus {

struct TrackCorridorParams {
    float half_width_m = 1.5f;
    float max_segment_m = 5.0f;
    float max_pose_offset_m = 1.0f;
};

class TrackCorridor {
public:
    explicit TrackCorridor(TrackCorridorParams params = {});

    bool set_centerline(const std::vector<Eigen::Vector2f>& centerline);
    bool contains(const Eigen::Vector2f& sensor, const Eigen::Vector2f& candidate,
                  float& distance_ahead) const;
    bool contains(const Eigen::Vector2f& sensor, const Eigen::Vector2f& candidate,
                  float& distance_ahead, float& lateral_offset) const;

private:
    struct Segment {
        Eigen::Vector2f start;
        Eigen::Vector2f direction;
        float length;
        float arc_start;
    };

    bool project(const Eigen::Vector2f& point, float& arc, float& offset) const;

    TrackCorridorParams params_;
    std::vector<Segment> segments_;
};

} // namespace argus
