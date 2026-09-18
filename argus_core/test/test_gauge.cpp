
#include <gtest/gtest.h>

#include "argus_core/gauge.hpp"

namespace {

argus::ClearanceGauge default_gauge() {
    argus::ClearanceGaugeParams p;
    p.safety_margin = 0.0f;
    return argus::ClearanceGauge(p);
}

} // namespace

TEST(Gauge, CenterInside) {
    const auto g = default_gauge();
    EXPECT_TRUE(g.contains(50.0f, 0.0f, 2.0f));
}

TEST(Gauge, OutsideByWidth) {
    const auto g = default_gauge();
    EXPECT_FALSE(g.contains(50.0f, 1.451f, 2.0f));
    EXPECT_FALSE(g.contains(50.0f, -1.451f, 2.0f));
}

TEST(Gauge, SafetyMarginDoesNotWidenLateralContains) {
    argus::ClearanceGaugeParams p;
    p.safety_margin = 0.10f;
    const argus::ClearanceGauge g(p);
    EXPECT_TRUE(g.contains(50.0f, 1.45f, 2.0f));
    EXPECT_FALSE(g.contains(50.0f, 1.50f, 2.0f));
}

TEST(Gauge, BoundaryExact) {
    const auto g = default_gauge();
    EXPECT_TRUE(g.contains(50.0f, 1.45f, 2.0f));
    EXPECT_TRUE(g.contains(50.0f, 0.0f, 3.60f));
}

TEST(Gauge, ChamferCutsTopCorner) {
    const auto g = default_gauge();
    EXPECT_FALSE(g.contains(50.0f, 1.45f, 3.59f));
    EXPECT_TRUE(g.contains(50.0f, 1.20f, 3.59f));
}

TEST(Gauge, OutsideByHeight) {
    const auto g = default_gauge();
    EXPECT_FALSE(g.contains(50.0f, 0.0f, 0.15f));
    EXPECT_FALSE(g.contains(50.0f, 0.0f, 3.61f));
}

TEST(Gauge, OutsideByRange) {
    const auto g = default_gauge();
    EXPECT_FALSE(g.contains(-0.01f, 0.0f, 2.0f));
    EXPECT_TRUE(g.contains(299.0f, 0.0f, 2.0f));
}

TEST(Gauge, ForwardDistance) {
    const auto g = default_gauge();
    EXPECT_FLOAT_EQ(g.forward_distance(47.5f, 0.0f, 2.0f), 47.5f);
    EXPECT_FLOAT_EQ(g.forward_distance(47.5f, 5.0f, 2.0f), -1.0f);
}

TEST(Gauge, DistanceTo) {
    const auto g = default_gauge();
    EXPECT_FLOAT_EQ(g.distance_to(50.0f, 0.0f, 2.0f), 0.0f);
    EXPECT_FLOAT_EQ(g.distance_to(50.0f, 2.45f, 2.0f), 1.0f);
    EXPECT_FLOAT_EQ(g.distance_to(40.0f, 0.0f, 4.6f), 1.0f);
}
