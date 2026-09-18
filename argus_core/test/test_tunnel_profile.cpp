
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "argus_core/tunnel_profile.hpp"

namespace {

constexpr uint32_t kW = 120;
constexpr uint32_t kH = 8;
constexpr float kWallRange = 25.0f;

argus::CleanCloud make_wall(float range, uint32_t az0, uint32_t az1) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool bumped = az >= az0 && az <= az1;
            c.x.push_back(bumped ? range - 8.0f : range);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    return c;
}

argus::RangeImage ri_of(const argus::CleanCloud& cloud) {
    argus::RangeImageParams p;
    p.rings_fallback = kH;
    return argus::build_range_image(cloud, p);
}

argus::TunnelProfileParams make_params() {
    argus::TunnelProfileParams p;
    p.alpha_initial = 0.5f;
    p.alpha_steady = 0.05f;
    p.warmup_frames = 10;
    p.max_range = 100.0f;
    p.min_range = 1.0f;
    return p;
}

} // namespace

TEST(TunnelProfile, LearnsFromFirstCleanFrame) {
    argus::TunnelProfile profile(make_params());
    const argus::CleanCloud cloud = make_wall(kWallRange, 0, 0);
    const argus::RangeImage ri = ri_of(cloud);

    EXPECT_FALSE(profile.valid());
    profile.update(ri, true);
    EXPECT_TRUE(profile.valid());
    EXPECT_FLOAT_EQ(profile.expected_range(10, 3), kWallRange);
}

TEST(TunnelProfile, DirtyFramesDoNotUpdateTheModel) {
    argus::TunnelProfile profile(make_params());
    const argus::RangeImage ri = ri_of(make_wall(kWallRange, 0, 0));
    profile.update(ri, true);

    const argus::RangeImage dirty = ri_of(make_wall(kWallRange - 5.0f, 20, 40));
    profile.update(dirty, false);
    EXPECT_FLOAT_EQ(profile.expected_range(30, 3), kWallRange);
    EXPECT_EQ(profile.updates(), 1u);
}

TEST(TunnelProfile, MedianDeviationIsSmallOnEmptyWallAndPeaksOnObstacle) {
    argus::TunnelProfile profile(make_params());
    const argus::CleanCloud empty = make_wall(kWallRange, 0, 0);
    const argus::RangeImage empty_ri = ri_of(empty);
    for (int i = 0; i < 20; ++i) {
        profile.update(empty_ri, true);
    }
    EXPECT_LT(profile.median_deviation(empty_ri), 0.3f);

    const argus::RangeImage blocked_ri = ri_of(make_wall(kWallRange, 20, 40));
    EXPECT_GT(profile.deviation(blocked_ri, 30, 3), 5.0f);
    EXPECT_LT(profile.deviation(blocked_ri, 5, 3), 0.3f);
    EXPECT_LT(profile.median_deviation(empty_ri), 0.3f);
    EXPECT_GT(profile.p95_deviation(blocked_ri), 0.3f);
}

TEST(TunnelProfile, ShapeChangeResetsTheModel) {
    argus::TunnelProfile profile(make_params());
    profile.update(ri_of(make_wall(kWallRange, 0, 0)), true);
    EXPECT_EQ(profile.width(), kW);

    argus::CleanCloud other;
    other.rings = kH + 2;
    other.n_raw = (kW + 4) * (kH + 2);
    for (uint32_t az = 0; az < kW + 4; ++az) {
        for (uint32_t ring = 0; ring < kH + 2; ++ring) {
            other.x.push_back(kWallRange);
            other.y.push_back(0.0f);
            other.z.push_back(0.0f);
            other.intensity.push_back(1.0f);
            other.ring.push_back(static_cast<uint16_t>(ring));
            other.azimuth_idx.push_back(az);
        }
    }
    profile.update(ri_of(other), true);
    EXPECT_EQ(profile.width(), kW + 4);
    EXPECT_EQ(profile.updates(), 1u);
}

TEST(TunnelProfile, ResetClearsEverything) {
    argus::TunnelProfile profile(make_params());
    profile.update(ri_of(make_wall(kWallRange, 0, 0)), true);
    ASSERT_TRUE(profile.valid());
    profile.reset();
    EXPECT_FALSE(profile.valid());
    EXPECT_EQ(profile.width(), 0u);
    EXPECT_EQ(profile.updates(), 0u);
}

TEST(TunnelProfile, EmptyRangeImageProducesNoModel) {
    argus::RangeImage ri = ri_of(make_wall(kWallRange, 0, 0));
    ri.range.assign(ri.range.size(), std::numeric_limits<float>::quiet_NaN());
    argus::TunnelProfile nan_profile(make_params());
    nan_profile.update(ri, true);
    EXPECT_FALSE(nan_profile.valid());
    EXPECT_EQ(nan_profile.updates(), 0u);
}

TEST(TunnelProfile, OutOfRangeReturnsAreIgnored) {
    argus::TunnelProfileParams p = make_params();
    p.max_range = 10.0f;
    argus::TunnelProfile profile(p);
    profile.update(ri_of(make_wall(kWallRange, 0, 0)), true);
    EXPECT_FALSE(profile.valid());

    argus::TunnelProfileParams near = make_params();
    near.min_range = 40.0f;
    argus::TunnelProfile close_profile(near);
    close_profile.update(ri_of(make_wall(kWallRange, 0, 0)), true);
    EXPECT_FALSE(close_profile.valid());
}
