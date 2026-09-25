#include "argus_core/verifier.hpp"

#include <algorithm>
#include <cmath>

namespace argus {

ClusterVerifier::ClusterVerifier(const VerifierParams& p) : p_(p) {}

VerifierVerdict ClusterVerifier::verify(const Cluster& cluster) const {
    VerifierVerdict verdict;
    if (!p_.enabled) {
        return verdict;
    }

    const float range = cluster.nearest_range;
    if (!std::isfinite(range) || range < p_.min_range_m) {
        verdict.reason = "too_close_for_verifier";
        return verdict;
    }
    if (range > p_.max_range_m) {
        verdict.reason = "beyond_verifier_range";
        return verdict;
    }

    const Eigen::Vector3f extent = cluster.max_corner - cluster.min_corner;
    const float span = std::max({extent.x(), extent.y(), extent.z()});
    const float thickness = std::min({extent.x(), extent.y(), extent.z()});
    const float volume = std::max(1e-3f, extent.x() * extent.y() * extent.z());
    const float density = static_cast<float>(cluster.point_count) / std::max(0.05f, volume);
    const float expected = p_.target_density_per_m * std::max(0.5f, range / 30.0f);
    const float density_term = std::clamp(density / std::max(1e-3f, expected), 0.0f, 1.0f);

    float score = 0.5f * density_term;
    score += 0.5f * std::clamp(cluster.score, 0.0f, 1.0f);

    const bool slab = thickness < p_.max_wall_thickness_m && span > 2.0f * thickness;
    if (slab) {
        score *= 0.5f;
    }
    if (cluster.point_count < p_.min_points) {
        score *= 0.6f;
    }
    if (span < p_.min_standalone_extent_m && !slab) {
        score *= 0.7f;
    }

    verdict.score = std::clamp(score, 0.0f, 1.0f);
    verdict.accepted = verdict.score >= p_.min_score;
    if (verdict.accepted) {
        verdict.reason = slab ? "accepted_dense_slab" : "accepted_standalone";
    } else if (slab) {
        verdict.reason = "rejected_wall_like";
    } else if (cluster.point_count < p_.min_points) {
        verdict.reason = "rejected_sparse";
    } else {
        verdict.reason = "rejected_low_score";
    }
    return verdict;
}

} // namespace argus
