
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "argus_core/fusion.hpp"
#include "argus_core/range_image.hpp"

namespace {

constexpr uint32_t kW = 480;
constexpr uint32_t kH = 8;
constexpr float kWallRange = 25.5f;
constexpr float kBoxRange = 16.9f;

argus::CleanCloud make_scene(float box_lateral_x) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool in_box = az >= 200 && az <= 249 && ring >= 3 && ring <= 5;
            const float lateral = in_box ? box_lateral_x + ring * 0.2f : -0.35f + ring * 0.1f;
            const float along = in_box ? kBoxRange + (az - 200) * 0.01f : kWallRange;
            const float z = 0.4f + ring * 0.1f;
            c.x.push_back(lateral);
            c.y.push_back(-along);
            c.z.push_back(z);
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

argus::AnomalySet geometry_of_box(const argus::CleanCloud& cloud) {
    argus::AnomalySet set;
    set.source = "geometry";
    for (size_t i = 0; i < cloud.size(); ++i) {
        const bool in_box = cloud.y[i] <= -kBoxRange - 0.001f && cloud.y[i] > -18.0f;
        if (in_box) {
            set.indices.push_back(static_cast<uint32_t>(i));
            set.score.push_back(0.8f);
        }
    }
    return set;
}

argus::FusionParams fusion_params() {
    argus::FusionParams p;
    p.clustering.min_cluster_size_near = 15;
    p.clustering.min_extent_m = 0.1f;
    p.clustering.max_extent_m = 50.0f;
    p.tracking.min_hits_to_confirm = 2;
    return p;
}

argus::ClearanceGauge gauge_neg_y() {
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    return argus::ClearanceGauge(gp);
}

} // namespace

TEST(Fusion, ObstacleAheadTriggersConfirmedAlert) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);
    const auto geometry = geometry_of_box(cloud);
    ASSERT_FALSE(geometry.indices.empty());

    argus::FusionPipeline pipe(gauge_neg_y(), fusion_params());

    const argus::FusionResult f1 = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_FALSE(f1.alert);

    const argus::FusionResult f2 = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_TRUE(f2.alert);
    EXPECT_FALSE(f2.tracks.empty());
    EXPECT_TRUE(f2.tracks[0].confirmed);
    EXPECT_GT(f2.nearest_forward_m, 16.0f);
    EXPECT_LT(f2.nearest_forward_m, 17.5f);
    EXPECT_EQ(f2.n_filtered_out, 0u);
}

TEST(Fusion, ConfirmedFarTrackStopsAlertingAfterStaleGeometry) {
    argus::CleanCloud cloud;
    cloud.rings = 8;
    cloud.n_raw = 80;
    for (uint32_t az = 0; az < 10; ++az) {
        for (uint32_t ring = 0; ring < 8; ++ring) {
            cloud.x.push_back(0.1f * static_cast<float>(ring));
            cloud.y.push_back(-100.0f - 0.01f * static_cast<float>(az));
            cloud.z.push_back(0.2f * static_cast<float>(ring));
            cloud.intensity.push_back(1.0f);
            cloud.raw_idx.push_back(az * 8 + ring);
        }
    }
    argus::RangeImageParams range_params;
    range_params.rings_fallback = 8;
    const auto ri = argus::build_range_image(cloud, range_params);
    argus::AnomalySet geometry;
    for (uint32_t i = 0; i < cloud.size(); ++i) {
        geometry.indices.push_back(i);
        geometry.score.push_back(1.0f);
    }
    argus::FusionParams params;
    params.clustering.min_cluster_size_near = 1;
    params.clustering.min_cluster_size_mid = 1;
    params.clustering.min_cluster_size_far = 1;
    params.clustering.min_cluster_size_long = 1;
    params.clustering.min_extent_m = 0.0f;
    params.clustering.max_extent_m = 10.0f;
    params.tracking.min_hits_to_confirm = 1;
    params.tracking.min_hits_to_confirm_far = 1;
    params.strict_track_freshness_range_m = 90.0f;
    params.max_confirmed_track_misses = 2;
    argus::ClearanceGaugeParams gauge_params;
    gauge_params.forward_axis = argus::ForwardAxis::NegY;
    gauge_params.half_width = 2.0f;
    gauge_params.height = 5.0f;
    argus::FusionPipeline pipeline(argus::ClearanceGauge(gauge_params), params);
    ASSERT_TRUE(pipeline.update(cloud, ri, geometry, {}, 0.1f, 0.0f).alert);
    for (uint32_t i = 0; i < params.max_confirmed_track_misses; ++i) {
        EXPECT_TRUE(pipeline.update(cloud, ri, {}, {}, 0.1f, 0.0f).alert);
    }
    EXPECT_FALSE(pipeline.update(cloud, ri, {}, {}, 0.1f, 0.0f).alert);
}

TEST(Fusion, LateralWallFilteredByGauge) {
    const auto cloud = make_scene(4.8f);
    const auto ri = ri_of(cloud);
    const auto geometry = geometry_of_box(cloud);

    argus::FusionPipeline pipe(gauge_neg_y(), fusion_params());
    pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    const argus::FusionResult f2 = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_FALSE(f2.alert);
    EXPECT_TRUE(f2.clusters.empty());
    EXPECT_GT(f2.n_filtered_out, 0u);
}

TEST(Fusion, GaugeWidthControlsAlertWithoutIncludingSideWall) {
    const auto cloud = make_scene(1.0f);
    const auto ri = ri_of(cloud);
    const auto geometry = geometry_of_box(cloud);

    argus::ClearanceGaugeParams narrow;
    narrow.forward_axis = argus::ForwardAxis::NegY;
    narrow.half_width = 1.4f;
    narrow.safety_margin = 0.1f;
    argus::FusionPipeline narrow_pipe(argus::ClearanceGauge(narrow), fusion_params());
    narrow_pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    const auto excluded = narrow_pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_FALSE(excluded.alert);

    narrow.half_width = 1.7f;
    argus::FusionPipeline wide_pipe(argus::ClearanceGauge(narrow), fusion_params());
    wide_pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    const auto included = wide_pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_TRUE(included.alert);
}

TEST(Fusion, MinVotesGateRequiresSecondDetector) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);
    const auto geometry = geometry_of_box(cloud);

    argus::AnomalySet no_return;
    no_return.source = "no_return";
    for (uint32_t az = 200; az <= 249; ++az) {
        for (uint32_t ring = 3; ring <= 5; ++ring) {
            no_return.cells.push_back(ri.idx(az, ring));
        }
    }

    argus::FusionParams gated = fusion_params();
    gated.min_votes_for_alert = 2;
    argus::FusionPipeline gated_pipe(gauge_neg_y(), gated);
    gated_pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    const argus::FusionResult gated_f2 = gated_pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_FALSE(gated_f2.alert);
    EXPECT_FALSE(gated_f2.tracks.empty());

    argus::FusionPipeline voting_pipe(gauge_neg_y(), gated);
    voting_pipe.update(cloud, ri, geometry, no_return, 0.1f, 0.0f);
    const argus::FusionResult voting_f2 =
        voting_pipe.update(cloud, ri, geometry, no_return, 0.1f, 0.0f);
    EXPECT_TRUE(voting_f2.alert);
    EXPECT_EQ(voting_f2.tracks[0].votes_geometry, 1u);
    EXPECT_EQ(voting_f2.tracks[0].votes_no_return, 1u);

    argus::FusionParams strict = fusion_params();
    strict.min_votes_for_alert = 3;
    argus::FusionPipeline strict_pipe(gauge_neg_y(), strict);
    strict_pipe.update(cloud, ri, geometry, no_return, 0.1f, 0.0f);
    const argus::FusionResult strict_f2 =
        strict_pipe.update(cloud, ri, geometry, no_return, 0.1f, 0.0f);
    EXPECT_FALSE(strict_f2.alert);
}

TEST(Fusion, NoReturnCellsCountedNotClustered) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::AnomalySet no_return;
    no_return.source = "no_return";
    for (uint32_t az = 200; az <= 209; ++az) {
        no_return.cells.push_back(ri.idx(az, 4));
        no_return.cells_score.push_back(1.0f);
    }

    argus::FusionPipeline pipe(gauge_neg_y(), fusion_params());
    const argus::FusionResult r = pipe.update(cloud, ri, {}, no_return, 0.1f, 0.0f);
    EXPECT_EQ(r.n_anom_cells, 10u);
    EXPECT_TRUE(r.clusters.empty());
    EXPECT_FALSE(r.alert);
}

TEST(Fusion, GroundFilterDropsRailLikePoints) {
    argus::CleanCloud cloud;
    cloud.rings = kH;
    cloud.n_raw = kW * kH;
    std::vector<uint32_t> rail_idx, object_idx;
    for (uint32_t az = 0; az < 200; ++az) {
        cloud.x.push_back(-1.5f);
        cloud.y.push_back(-static_cast<float>(az) * 0.05f - 3.0f);
        cloud.z.push_back(-0.9f);
        cloud.intensity.push_back(1.0f);
        cloud.ring.push_back(static_cast<uint16_t>(az % kH));
        cloud.azimuth_idx.push_back(az);
        rail_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }
    for (uint32_t az = 200; az <= 249; ++az) {
        cloud.x.push_back(0.0f);
        cloud.y.push_back(-16.9f - static_cast<float>(az - 200) * 0.02f);
        cloud.z.push_back(-0.2f);
        cloud.intensity.push_back(1.0f);
        cloud.ring.push_back(static_cast<uint16_t>(az % kH));
        cloud.azimuth_idx.push_back(az);
        object_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }

    argus::RangeImageParams rp;
    rp.rings_fallback = kH;
    const auto ri = argus::build_range_image(cloud, rp);

    argus::AnomalySet geometry;
    geometry.source = "geometry";
    geometry.indices = rail_idx;
    geometry.indices.insert(geometry.indices.end(), object_idx.begin(), object_idx.end());
    geometry.score.assign(geometry.indices.size(), 0.8f);

    argus::FusionParams p = fusion_params();
    p.clustering.min_cluster_size_near = 5;
    p.clustering.min_extent_m = 0.1f;
    p.tracking.min_hits_to_confirm = 1;
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    gp.sensor_height = 1.2f;
    argus::FusionPipeline pipe(argus::ClearanceGauge(gp), p);
    const argus::FusionResult r = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);

    ASSERT_EQ(r.clusters.size(), 1u);
    EXPECT_NEAR(r.clusters[0].forward_distance, 16.9f, 0.5f);
    EXPECT_EQ(r.clusters[0].point_count, object_idx.size());
    EXPECT_TRUE(r.alert);
}

TEST(Fusion, GaugeAxisParsedAndApplied) {
    argus::ForwardAxis axis = argus::ForwardAxis::PosX;
    ASSERT_TRUE(argus::parse_forward_axis("-y", axis));
    EXPECT_EQ(axis, argus::ForwardAxis::NegY);
    ASSERT_TRUE(argus::parse_forward_axis("+x", axis));
    EXPECT_EQ(axis, argus::ForwardAxis::PosX);
    EXPECT_FALSE(argus::parse_forward_axis("diagonal", axis));

    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    const argus::ClearanceGauge g(gp);
    EXPECT_FLOAT_EQ(g.forward_distance(0.5f, -16.9f, 1.0f), 16.9f);
    EXPECT_FLOAT_EQ(g.distance_to(0.5f, -16.9f, 1.0f), 0.0f);
    argus::ClearanceGaugeParams gpx;
    const argus::ClearanceGauge gx(gpx);
    EXPECT_EQ(gx.forward_distance(0.5f, -16.9f, 1.0f), -1.0f);
}

TEST(Fusion, GaugeSensorHeightShiftsZBounds) {
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    gp.sensor_height = 1.2f;
    const argus::ClearanceGauge g(gp);
    EXPECT_FLOAT_EQ(g.forward_distance(0.0f, -16.9f, -0.7f), 16.9f);
    EXPECT_EQ(g.forward_distance(0.0f, -16.9f, -1.2f), -1.0f);
    argus::ClearanceGaugeParams g0;
    g0.forward_axis = argus::ForwardAxis::NegY;
    const argus::ClearanceGauge without_shift(g0);
    EXPECT_EQ(without_shift.forward_distance(0.0f, -16.9f, -1.2f), -1.0f);
    EXPECT_FLOAT_EQ(without_shift.forward_distance(0.0f, -16.9f, -0.7f), -1.0f);
}

TEST(Fusion, CleanFramesBuildTheTunnelModel) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::FusionParams p = fusion_params();
    p.use_free_space = true;
    p.model.voxel_size = 0.5f;
    p.model.min_observations = 2;

    argus::FusionPipeline pipe(gauge_neg_y(), p);
    const argus::FusionResult clean =
        pipe.update(cloud, ri, {}, {}, 0.1f, 0.0f, Eigen::Isometry3d::Identity());
    EXPECT_TRUE(clean.model_updated);
    EXPECT_TRUE(clean.pose_used);
    EXPECT_GT(pipe.model().voxel_count(), 0u);
}

TEST(Fusion, DirtyFramesStillCarveButDoNotRegisterHits) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::FusionParams p = fusion_params();
    p.use_free_space = true;
    p.model.voxel_size = 0.5f;
    p.model.min_observations = 2;
    p.tracking.min_hits_to_confirm = 1;

    argus::FusionPipeline pipe(gauge_neg_y(), p);
    const auto geometry = geometry_of_box(cloud);
    ASSERT_FALSE(geometry.indices.empty());
    const argus::FusionResult dirty =
        pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f, Eigen::Isometry3d::Identity());
    EXPECT_TRUE(dirty.alert);
    EXPECT_TRUE(dirty.model_updated);

    const argus::TunnelModelStats s = pipe.model().stats();
    EXPECT_EQ(s.n_occupied, 0u);
}

TEST(Fusion, FreeSpaceVoteCountsOnlyProvidedViolations) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::FusionParams p = fusion_params();
    p.use_free_space = true;
    p.model.voxel_size = 0.5f;
    p.model.min_observations = 1;
    p.free_space_warmup_frames = 2;
    p.free_space_min_points = 5;
    p.free_space_min_cells = 5;
    p.tracking.min_hits_to_confirm = 1;

    argus::FusionPipeline pipe(gauge_neg_y(), p);
    for (int i = 0; i < 3; ++i) {
        pipe.update(cloud, ri, {}, {}, 0.1f, 0.0f, Eigen::Isometry3d::Identity());
    }
    ASSERT_TRUE(pipe.model().ready());

    argus::AnomalySet free_space;
    free_space.source = "free_space";
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.y[i] <= -kBoxRange - 0.001f && cloud.y[i] > -18.0f) {
            free_space.indices.push_back(static_cast<uint32_t>(i));
            free_space.score.push_back(0.9f);
        }
    }
    ASSERT_FALSE(free_space.indices.empty());

    const argus::FusionResult r =
        pipe.update(cloud, ri, {}, {}, free_space, 0.1f, 0.0f, Eigen::Isometry3d::Identity());
    EXPECT_GT(r.n_free_space_points, 0u);
    ASSERT_FALSE(r.clusters.empty());
    EXPECT_EQ(r.clusters[0].votes_free_space, 1u);
}

TEST(Fusion, FreeSpaceVoteIgnoredDuringWarmup) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::FusionParams p = fusion_params();
    p.use_free_space = true;
    p.model.voxel_size = 0.5f;
    p.model.min_observations = 1;
    p.free_space_warmup_frames = 100;
    p.tracking.min_hits_to_confirm = 1;

    argus::FusionPipeline pipe(gauge_neg_y(), p);
    argus::AnomalySet free_space;
    free_space.source = "free_space";
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.y[i] <= -kBoxRange - 0.001f && cloud.y[i] > -18.0f) {
            free_space.indices.push_back(static_cast<uint32_t>(i));
            free_space.score.push_back(0.9f);
        }
    }

    const argus::FusionResult r =
        pipe.update(cloud, ri, {}, {}, free_space, 0.1f, 0.0f, Eigen::Isometry3d::Identity());
    EXPECT_EQ(r.n_free_space_points, 0u);
    EXPECT_TRUE(r.clusters.empty());
    EXPECT_FALSE(r.alert);
}

TEST(Fusion, MissingPoseSkipsFreeSpaceAndModel) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::FusionParams p = fusion_params();
    p.use_free_space = true;
    p.model.voxel_size = 0.5f;
    p.model.min_observations = 1;
    p.free_space_warmup_frames = 1;
    p.free_space_min_points = 5;
    p.free_space_min_cells = 5;

    argus::FusionPipeline pipe(gauge_neg_y(), p);
    argus::AnomalySet free_space;
    free_space.source = "free_space";
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.y[i] <= -kBoxRange - 0.001f && cloud.y[i] > -18.0f) {
            free_space.indices.push_back(static_cast<uint32_t>(i));
        }
    }

    const argus::FusionResult no_pose = pipe.update(cloud, ri, {}, {}, 0.1f, 0.0f);
    EXPECT_FALSE(no_pose.pose_used);
    EXPECT_FALSE(no_pose.model_updated);
    EXPECT_EQ(pipe.model().voxel_count(), 0u);

    const argus::FusionResult with_pose = pipe.update(cloud, ri, {}, {}, free_space, 0.1f, 0.0f,
                                                      Eigen::Isometry3d::Identity(), true, false);
    EXPECT_FALSE(with_pose.pose_used);
    EXPECT_EQ(with_pose.n_free_space_points, 0u);
    EXPECT_FALSE(with_pose.model_updated);
}
