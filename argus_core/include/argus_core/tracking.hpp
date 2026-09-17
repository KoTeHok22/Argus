
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "argus_core/clustering.hpp"

namespace argus {

struct TrackingParams {
    float max_association_distance = 2.0f;
    uint32_t min_hits_to_confirm = 3;
    uint32_t max_misses_to_keep = 4;
    float process_noise = 0.5f;
    float measurement_noise = 0.3f;
};

struct Track {
    uint32_t id = 0;
    Eigen::Matrix<float, 6, 1> state = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 6> covariance = Eigen::Matrix<float, 6, 6>::Identity();
    uint32_t hits = 0;
    uint32_t misses = 0;
    bool confirmed = false;

    float nearest_range = 0.0f;
    float ttc = -1.0f;
    float volume = 0.0f;
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    uint32_t point_count = 0;
    float score = 0.0f;
    uint8_t votes_free_space = 0;
    uint8_t votes_no_return = 0;
    uint8_t votes_geometry = 0;
};

class ObstacleTracker {
public:
    explicit ObstacleTracker(const TrackingParams& p);

    std::vector<Track> update(const std::vector<Cluster>& clusters, float dt,
                              float train_speed_mps);

    const std::vector<Track>& all_tracks() const { return tracks_; }

private:
    TrackingParams p_;
    std::vector<Track> tracks_;
    uint32_t next_id_ = 1;
};

float compute_ttc(const Track& t, float train_speed_mps);

} // namespace argus
