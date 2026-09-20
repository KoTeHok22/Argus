#pragma once

#include "argus_core/gauge.hpp"
#include "argus_core/tracking.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct VisibilityParams {
    float min_z_rel_m = 0.30f;
    float half_angle_deg = 20.0f;
    float max_range_m = 300.0f;
};

float forward_visibility_m(const CleanCloud& cloud, const ClearanceGauge& gauge,
                           const VisibilityParams& p);

float speed_limit_for_visibility_mps(float visibility_m, const BrakingParams& braking,
                                     float margin_m);

} // namespace argus