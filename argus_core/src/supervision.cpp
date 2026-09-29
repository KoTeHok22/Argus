#include "argus_core/supervision.hpp"

#include <cmath>

namespace argus {

float forward_visibility_m(const CleanCloud& cloud, const ClearanceGauge& gauge,
                           const VisibilityParams& p) {
    const float cos_limit = std::cos(p.half_angle_deg * static_cast<float>(M_PI) / 180.0f);
    float best = 0.0f;
    for (size_t i = 0; i < cloud.size(); ++i) {
        const float x = cloud.x[i], y = cloud.y[i], z = cloud.z[i];
        const float fwd = x * gauge.forward_x() + y * gauge.forward_y();
        if (fwd <= 0.0f || fwd > p.max_range_m) {
            continue;
        }
        const float z_rel = z + gauge.params().sensor_height;
        if (z_rel < p.min_z_rel_m) {
            continue;
        }
        const float lat = x * gauge.left_x() + y * gauge.left_y();
        const float dist = std::sqrt(fwd * fwd + lat * lat);
        if (dist <= 0.0f) {
            continue;
        }
        if (fwd / dist < cos_limit) {
            continue;
        }
        if (fwd > best) {
            best = fwd;
        }
    }
    return best;
}

float speed_limit_for_visibility_mps(float visibility_m, const BrakingParams& braking,
                                     float margin_m) {
    const float usable = visibility_m - std::max(0.0f, margin_m);
    if (usable <= 0.0f) {
        return 0.0f;
    }
    const float a = std::max(braking.decel_mps2, 1e-3f);
    const float reaction = std::max(0.0f, braking.reaction_s);
    const float v = -a * reaction + std::sqrt(a * a * reaction * reaction + 2.0f * a * usable);
    return v > 0.0f ? v : 0.0f;
}

} // namespace argus