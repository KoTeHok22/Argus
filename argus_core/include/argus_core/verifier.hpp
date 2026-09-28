#pragma once

#include <cstdint>
#include <string>

#include "argus_core/clustering.hpp"

namespace argus {

struct VerifierParams {
    bool enabled = false;
    float min_range_m = 15.0f;
    float max_range_m = 150.0f;
    float min_score = 0.35f;
    float target_density_per_m = 3.0f;
    float max_wall_thickness_m = 1.2f;
    float min_standalone_extent_m = 0.25f;
    uint32_t min_points = 3;
};

struct VerifierVerdict {
    bool accepted = true;
    float score = 1.0f;
    std::string reason;
};

class ClusterVerifier {
public:
    explicit ClusterVerifier(const VerifierParams& p);

    VerifierVerdict verify(const Cluster& cluster) const;

    const VerifierParams& params() const { return p_; }

private:
    VerifierParams p_;
};

} // namespace argus
