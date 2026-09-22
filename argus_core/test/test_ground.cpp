#include <gtest/gtest.h>

#include "argus_core/fusion.hpp"
#include "argus_core/ground.hpp"
#include "argus_core/range_image.hpp"

TEST(Ground, RailAndFloorMarkedObstacleKept) {
    argus::CleanCloud cloud;
    for (int i = 0; i < 80; ++i) {
        cloud.x.push_back(-1.4f + 0.03f * static_cast<float>(i % 8));
        cloud.y.push_back(-4.0f - 0.2f * static_cast<float>(i));
        cloud.z.push_back(-1.05f);
        cloud.intensity.push_back(8.0f);
    }
    const size_t n_rail = cloud.size();
    for (int i = 0; i < 40; ++i) {
        cloud.x.push_back(0.2f);
        cloud.y.push_back(-16.9f - 0.02f * static_cast<float>(i));
        cloud.z.push_back(0.4f);
        cloud.intensity.push_back(20.0f);
    }
    const size_t n_box = cloud.size() - n_rail;
    for (int i = 0; i < 40; ++i) {
        cloud.x.push_back(6.0f);
        cloud.y.push_back(-8.0f);
        cloud.z.push_back(-0.9f);
        cloud.intensity.push_back(5.0f);
    }

    argus::GroundParams gp;
    gp.enabled = true;
    gp.method = argus::GroundMethod::PatchworkPp;
    gp.sensor_height = 1.2f;
    gp.forward_axis = argus::ForwardAxis::NegY;
    gp.min_range = 0.5f;
    gp.max_range = 80.0f;
    argus::GroundSegmenter seg(gp);
    argus::GroundStats st;
    seg.apply(cloud, &st);

    ASSERT_EQ(cloud.ground_mask.size(), cloud.size());
    uint32_t rail_hit = 0;
    for (size_t i = 0; i < n_rail; ++i) {
        rail_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    uint32_t box_hit = 0;
    for (size_t i = n_rail; i < n_rail + n_box; ++i) {
        box_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    uint32_t wall_hit = 0;
    for (size_t i = n_rail + n_box; i < cloud.size(); ++i) {
        wall_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    EXPECT_GT(rail_hit, n_rail / 2);
    EXPECT_EQ(box_hit, 0u);
    EXPECT_EQ(wall_hit, 0u);
    EXPECT_GT(st.n_ground, 0u);
}

TEST(Ground, PlatformRailAtHalfMeterIsMasked) {
    argus::CleanCloud cloud;
    for (int i = 0; i < 40; ++i) {
        cloud.x.push_back(1.36f);
        cloud.y.push_back(-8.0f - 0.05f * static_cast<float>(i));
        cloud.z.push_back(-0.70f);
        cloud.intensity.push_back(4.0f);
    }
    for (int i = 0; i < 20; ++i) {
        cloud.x.push_back(0.2f);
        cloud.y.push_back(-16.9f);
        cloud.z.push_back(0.47f);
        cloud.intensity.push_back(20.0f);
    }

    argus::GroundParams gp;
    gp.enabled = true;
    gp.method = argus::GroundMethod::PatchworkPp;
    gp.sensor_height = 1.2f;
    gp.forward_axis = argus::ForwardAxis::NegY;
    argus::GroundSegmenter seg(gp);
    seg.apply(cloud);

    uint32_t rail_hit = 0;
    uint32_t box_hit = 0;
    for (size_t i = 0; i < 40; ++i) {
        rail_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    for (size_t i = 40; i < cloud.size(); ++i) {
        box_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    EXPECT_EQ(rail_hit, 40u);
    EXPECT_EQ(box_hit, 0u);
}

TEST(Ground, DisabledLeavesMaskZero) {
    argus::CleanCloud cloud;
    cloud.x = {0.0f, 1.0f};
    cloud.y = {-5.0f, -5.0f};
    cloud.z = {-1.0f, -1.0f};
    cloud.intensity = {1.0f, 1.0f};
    argus::GroundParams gp;
    gp.enabled = false;
    argus::GroundSegmenter seg(gp);
    argus::GroundStats st;
    seg.apply(cloud, &st);
    ASSERT_EQ(cloud.ground_mask.size(), 2u);
    EXPECT_EQ(cloud.ground_mask[0], 0);
    EXPECT_EQ(cloud.ground_mask[1], 0);
    EXPECT_EQ(st.n_nonground, 2u);
}

TEST(Ground, ZThresholdMasksRailKeepsObstacle) {
    argus::CleanCloud cloud;
    for (int i = 0; i < 40; ++i) {
        cloud.x.push_back(-1.4f);
        cloud.y.push_back(-4.0f - 0.1f * static_cast<float>(i));
        cloud.z.push_back(-1.05f);
        cloud.intensity.push_back(8.0f);
    }
    for (int i = 0; i < 20; ++i) {
        cloud.x.push_back(0.2f);
        cloud.y.push_back(-16.9f);
        cloud.z.push_back(0.47f);
        cloud.intensity.push_back(20.0f);
    }
    for (int i = 0; i < 10; ++i) {
        cloud.x.push_back(6.0f);
        cloud.y.push_back(-8.0f);
        cloud.z.push_back(-0.9f);
        cloud.intensity.push_back(5.0f);
    }

    argus::GroundParams gp;
    gp.enabled = true;
    gp.method = argus::GroundMethod::ZThreshold;
    gp.sensor_height = 1.2f;
    gp.forward_axis = argus::ForwardAxis::NegY;
    argus::GroundSegmenter seg(gp);
    argus::GroundStats st;
    seg.apply(cloud, &st);

    uint32_t rail_hit = 0;
    uint32_t box_hit = 0;
    uint32_t wall_hit = 0;
    for (size_t i = 0; i < 40; ++i) {
        rail_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    for (size_t i = 40; i < 60; ++i) {
        box_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    for (size_t i = 60; i < cloud.size(); ++i) {
        wall_hit += cloud.ground_mask[i] != 0 ? 1u : 0u;
    }
    EXPECT_EQ(rail_hit, 40u);
    EXPECT_EQ(box_hit, 0u);
    EXPECT_EQ(wall_hit, 0u);
    EXPECT_EQ(st.n_rail_fallback, 40u);
}

TEST(Fusion, GroundMaskDropsRailsWithoutFallbackFilter) {
    argus::CleanCloud cloud;
    std::vector<uint32_t> rail_idx, object_idx;
    for (uint32_t az = 0; az < 200; ++az) {
        cloud.x.push_back(-1.5f);
        cloud.y.push_back(-static_cast<float>(az) * 0.05f - 3.0f);
        cloud.z.push_back(-0.9f);
        cloud.intensity.push_back(1.0f);
        rail_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }
    for (uint32_t az = 200; az <= 249; ++az) {
        cloud.x.push_back(0.0f);
        cloud.y.push_back(-16.9f - static_cast<float>(az - 200) * 0.02f);
        cloud.z.push_back(-0.2f);
        cloud.intensity.push_back(1.0f);
        object_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }
    cloud.ground_mask.assign(cloud.size(), 0);
    for (uint32_t i : rail_idx) {
        cloud.ground_mask[i] = 1;
    }
    cloud.rings = 8;
    cloud.n_raw = static_cast<uint32_t>(cloud.size());

    argus::RangeImageParams rp;
    rp.rings_fallback = 8;
    const auto ri = argus::build_range_image(cloud, rp);
    argus::AnomalySet geometry;
    geometry.indices = rail_idx;
    geometry.indices.insert(geometry.indices.end(), object_idx.begin(), object_idx.end());
    geometry.score.assign(geometry.indices.size(), 0.8f);

    argus::FusionParams p;
    p.ground_filter = false;
    p.clustering.min_cluster_size_near = 5;
    p.clustering.min_extent_m = 0.1f;
    p.clustering.max_extent_m = 50.0f;
    p.tracking.min_hits_to_confirm = 1;
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    gp.sensor_height = 1.2f;
    argus::FusionPipeline pipe(argus::ClearanceGauge(gp), p);
    const argus::FusionResult r = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    ASSERT_EQ(r.clusters.size(), 1u);
    EXPECT_NEAR(r.clusters[0].forward_distance, 16.9f, 0.5f);
    EXPECT_EQ(r.clusters[0].point_count, object_idx.size());
}
