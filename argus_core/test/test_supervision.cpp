#include <gtest/gtest.h>

#include <cmath>

#include "argus_core/supervision.hpp"

namespace {

argus::ClearanceGauge make_gauge() {
    argus::ClearanceGaugeParams p;
    p.forward_axis = argus::ForwardAxis::NegY;
    p.sensor_height = 1.075f;
    p.half_width = 1.5f;
    return argus::ClearanceGauge(p);
}

argus::CleanCloud make_cloud(float front_range, float lateral) {
    argus::CleanCloud c;
    for (int i = 0; i < 10; ++i) {
        c.x.push_back(lateral);
        c.y.push_back(-front_range);
        c.z.push_back(0.5f);
        c.intensity.push_back(1.0f);
    }
    return c;
}

TEST(Supervision, VisibilityFindsForwardReturn) {
    const auto gauge = make_gauge();
    argus::VisibilityParams p;
    const auto cloud = make_cloud(80.0f, 0.0f);
    EXPECT_NEAR(argus::forward_visibility_m(cloud, gauge, p), 80.0f, 0.5f);
}

TEST(Supervision, VisibilityIgnoresBehindAndGround) {
    const auto gauge = make_gauge();
    argus::VisibilityParams p;
    argus::CleanCloud cloud;
    cloud.x.push_back(0.0f);
    cloud.y.push_back(50.0f);
    cloud.z.push_back(0.5f);
    cloud.x.push_back(0.0f);
    cloud.y.push_back(-100.0f);
    cloud.z.push_back(-1.0f);
    EXPECT_FLOAT_EQ(argus::forward_visibility_m(cloud, gauge, p), 0.0f);
}

TEST(Supervision, VisibilityIgnoresWideOffAxis) {
    const auto gauge = make_gauge();
    argus::VisibilityParams p;
    p.half_angle_deg = 20.0f;
    const auto cloud = make_cloud(60.0f, 60.0f);
    EXPECT_FLOAT_EQ(argus::forward_visibility_m(cloud, gauge, p), 0.0f);
}

TEST(Supervision, VisibilityRespectsMaxRange) {
    const auto gauge = make_gauge();
    argus::VisibilityParams p;
    p.max_range_m = 50.0f;
    const auto cloud = make_cloud(200.0f, 0.0f);
    EXPECT_FLOAT_EQ(argus::forward_visibility_m(cloud, gauge, p), 0.0f);
}

TEST(Supervision, SpeedLimitZeroWhenVisibilityBelowMargin) {
    argus::BrakingParams b;
    b.decel_mps2 = 1.3f;
    b.reaction_s = 0.5f;
    EXPECT_FLOAT_EQ(argus::speed_limit_for_visibility_mps(5.0f, b, 10.0f), 0.0f);
}

TEST(Supervision, SpeedLimitStopsWithinVisibility) {
    argus::BrakingParams b;
    b.decel_mps2 = 1.3f;
    b.reaction_s = 0.5f;
    const float visibility = 200.0f;
    const float margin = 10.0f;
    const float v = argus::speed_limit_for_visibility_mps(visibility, b, margin);
    const float stop = v * b.reaction_s + v * v / (2.0f * b.decel_mps2);
    EXPECT_NEAR(stop, visibility - margin, 0.1f);
}

TEST(Supervision, SpeedLimitMonotonicInVisibility) {
    argus::BrakingParams b;
    float prev = -1.0f;
    for (float vis = 10.0f; vis <= 300.0f; vis += 10.0f) {
        const float v = argus::speed_limit_for_visibility_mps(vis, b, 5.0f);
        EXPECT_GT(v, prev);
        prev = v;
    }
}

TEST(Supervision, SpeedLimitMatchesMetroCase) {
    argus::BrakingParams b;
    b.decel_mps2 = 1.3f;
    b.reaction_s = 0.5f;
    const float v = argus::speed_limit_for_visibility_mps(200.0f, b, 10.0f);
    EXPECT_NEAR(v, 21.6f, 0.5f);
}

} // namespace