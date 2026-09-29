#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "argus_core/gauge.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct OdometryParams {
    bool enabled = true;
    bool lock_lateral = true;
    bool lock_roll = true;
    bool lock_vertical = false;
    float max_range = 120.0f;
    float min_range = 1.0f;
    float voxel_size = 0.5f;
    float max_speed_mps = 30.0f;
    float max_accel_mps2 = 2.0f;
    uint32_t max_iterations = 4;
    float max_correspondence_m = 2.0f;
    float rest_translation_m = 0.20f;
    bool keep_ground = true;
    ForwardAxis forward_axis = ForwardAxis::NegY;
    float max_fitness = 0.05f;
    uint32_t min_correspondences_quality = 100;
};

struct OdometryResult {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double stamp_s = 0.0;
    float speed_mps = 0.0f;
    float accel_mps2 = 0.0f;
    float fitness = 0.0f;
    uint32_t n_correspondences = 0;
    uint32_t n_downsampled = 0;
    bool valid = false;
    bool quality_ok = false;
    uint32_t frames = 0;
};

class TunnelOdometry {
public:
    explicit TunnelOdometry(const OdometryParams& p);

    OdometryResult update(const CleanCloud& cloud, double stamp_s);

    const OdometryResult& last() const { return last_; }

private:
    OdometryParams p_;
    ClearanceGauge gauge_;
    std::vector<Eigen::Vector3f> prev_;
    double prev_stamp_s_ = 0.0;
    Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
    float prev_speed_mps_ = 0.0f;
    OdometryResult last_;
    bool have_prev_ = false;
};

bool icp_find_nearest(const std::vector<Eigen::Vector3f>& pts, const Eigen::Vector3f& q,
                      float voxel_size, float max_correspondence_m, Eigen::Vector3f& out);

} // namespace argus
