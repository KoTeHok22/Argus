
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "argus_core/range_image.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct ClusteringParams {
    float neighbor_distance = 0.35f;
    bool adaptive_scaling = true;
    float reference_range = 50.0f;
    uint32_t min_cluster_size = 15;
    uint32_t max_cluster_size = 20000;
    float min_extent_m = 0.20f;
    float max_extent_m = 5.00f;
};

struct Cluster {
    std::vector<uint32_t> indices;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_corner = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_corner = Eigen::Vector3f::Zero();
    float nearest_range = 0.0f;
    float forward_distance = -1.0f;
    float volume = 0.0f;
    float score = 0.0f;
    uint32_t point_count = 0;

    uint8_t votes_free_space = 0;
    uint8_t votes_no_return = 0;
    uint8_t votes_geometry = 0;
};

float neighbor_distance_at(float base, float d, float ref, bool adaptive);

std::vector<Cluster> cluster_anomalies(const CleanCloud& cloud, const RangeImage& ri,
                                       const AnomalySet& merged, const ClusteringParams& p);

} // namespace argus
