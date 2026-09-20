
#include <cmath>

#include "argus_core/anomaly.hpp"
#include "argus_core/tunnel_model.hpp"

namespace argus {

FreeSpaceDetector::FreeSpaceDetector(TunnelModel* model, const FreeSpaceParams& p)
    : model_(model), p_(p) {}

AnomalySet FreeSpaceDetector::detect(const CleanCloud& cloud, const RangeImage& ri) {
    return detect(cloud, ri, Eigen::Isometry3d::Identity());
}

AnomalySet FreeSpaceDetector::detect(const CleanCloud& cloud, const RangeImage& ri,
                                     const Eigen::Isometry3d& pose) {
    AnomalySet out;
    out.source = name();
    if (!p_.enabled || model_ == nullptr || !model_->ready() || !ri.valid()) {
        return out;
    }

    const Eigen::Vector3f origin = (pose.inverse() * Eigen::Vector3d::Zero()).cast<float>();

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (!cloud.ground_mask.empty() && i < cloud.ground_mask.size() &&
            cloud.ground_mask[i] != 0) {
            continue;
        }
        const Eigen::Vector3f lidar(cloud.x[i], cloud.y[i], cloud.z[i]);
        const float range = (lidar - origin).norm();
        if (!std::isfinite(range) || range < p_.min_range || range > p_.max_range) {
            continue;
        }
        float confidence = 0.0f;
        if (!model_->contradicts_free_space(lidar, confidence, pose, p_.min_confidence,
                                            p_.min_free_observations)) {
            continue;
        }
        out.indices.push_back(static_cast<uint32_t>(i));
        out.score.push_back(confidence);
    }
    return out;
}

} // namespace argus
