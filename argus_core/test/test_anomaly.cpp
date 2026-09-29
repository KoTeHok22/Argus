
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "argus_core/anomaly.hpp"
#include "argus_core/range_image.hpp"

namespace {

constexpr uint32_t kW = 480;
constexpr uint32_t kH = 8;
constexpr float kWallRange = 25.5f;
constexpr float kBoxRange = 16.9f;

argus::CleanCloud make_wall_cloud(float box_range, uint32_t az0, uint32_t az1, uint32_t r0,
                                  uint32_t r1, float wall_range = kWallRange) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool in_box = az >= az0 && az <= az1 && ring >= r0 && ring <= r1;
            const float d = in_box ? box_range : wall_range;
            c.x.push_back(d);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    return c;
}

argus::GeometryResidualParams make_geom_params(uint32_t min_stable_frames) {
    argus::GeometryResidualParams p;
    p.median_half_window = 100;
    p.residual_threshold_m = 1.0f;
    p.residual_threshold_far_m = 1.0f;
    p.min_cells = 50;
    p.min_fill = 0.3f;
    p.min_range = 4.0f;
    p.max_target_range_m = 45.0f;
    p.far_range_m = 60.0f;
    p.min_cells_far = 8;
    p.min_fill_far = 0.12f;
    p.min_stable_frames = min_stable_frames;
    p.az_tolerance = 20;
    p.range_tolerance_m = 3.0f;
    return p;
}

argus::RangeImage ri_of(const argus::CleanCloud& cloud) {
    argus::RangeImageParams p;
    p.rings_fallback = kH;
    return argus::build_range_image(cloud, p);
}

} // namespace

TEST(GeometryDetector, DenseBoxOnWallDetected) {
    const auto cloud = make_wall_cloud(kBoxRange, 200, 249, 3, 5);
    const auto ri = ri_of(cloud);
    ASSERT_TRUE(ri.valid());

    argus::GeometryResidualDetector det(make_geom_params(1));
    const argus::AnomalySet set = det.detect(cloud, ri);

    ASSERT_FALSE(set.indices.empty());
    ASSERT_EQ(set.indices.size(), set.score.size());
    for (uint32_t idx : set.indices) {
        ASSERT_LT(idx, cloud.size());
        EXPECT_FLOAT_EQ(cloud.x[idx], kBoxRange);
    }
}

TEST(GeometryDetector, StableBoxNeedsTwoFrames) {
    const auto cloud = make_wall_cloud(kBoxRange, 200, 249, 3, 5);
    const auto ri = ri_of(cloud);

    argus::GeometryResidualDetector det(make_geom_params(2));
    const argus::AnomalySet first = det.detect(cloud, ri);
    EXPECT_TRUE(first.indices.empty());

    const argus::AnomalySet second = det.detect(cloud, ri);
    EXPECT_FALSE(second.indices.empty());
}

TEST(GeometryDetector, TransientBandDoesNotAlarm) {
    const auto cloud_f1 = make_wall_cloud(8.0f, 100, 199, 3, 3);
    const auto cloud_f2 = make_wall_cloud(24.0f, 100, 199, 3, 3);
    const auto ri_f1 = ri_of(cloud_f1);
    const auto ri_f2 = ri_of(cloud_f2);

    argus::GeometryResidualDetector det(make_geom_params(2));
    const argus::AnomalySet a1 = det.detect(cloud_f1, ri_f1);
    const argus::AnomalySet a2 = det.detect(cloud_f2, ri_f2);
    EXPECT_TRUE(a1.indices.empty());
    EXPECT_TRUE(a2.indices.empty());
}

TEST(GeometryDetector, SparseBandDoesNotAlarm) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool band = ring == 3 && az >= 150 && az <= 320 && az % 4 == 0;
            const float d = band ? 20.0f : kWallRange;
            c.x.push_back(d);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    const auto ri = ri_of(c);

    argus::GeometryResidualDetector det(make_geom_params(1));
    const argus::AnomalySet set = det.detect(c, ri);
    EXPECT_TRUE(set.indices.empty());
}

TEST(GeometryDetector, EmptyWallNoAlarm) {
    const auto cloud = make_wall_cloud(kWallRange, 1, 0, 0, 0);
    const auto ri = ri_of(cloud);

    argus::GeometryResidualDetector det(make_geom_params(1));
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.indices.empty());
}

namespace {

struct HoleSpec {
    uint32_t az0, az1, ring;
};

argus::CleanCloud make_cloud_with_holes(const std::vector<HoleSpec>& zero_holes,
                                        const std::vector<HoleSpec>& dropped) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    const auto is_hole = [&zero_holes, &dropped](uint32_t az, uint32_t ring, bool* zero) {
        for (const HoleSpec& h : zero_holes) {
            if (az >= h.az0 && az <= h.az1 && ring == h.ring) {
                *zero = true;
                return true;
            }
        }
        for (const HoleSpec& h : dropped) {
            if (az >= h.az0 && az <= h.az1 && ring == h.ring) {
                return true;
            }
        }
        return false;
    };

    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            bool zero = false;
            if (is_hole(az, ring, &zero)) {
                if (zero) {
                    c.no_return_raw.push_back(az * kH + ring);
                }
                continue;
            }
            c.x.push_back(kWallRange);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.raw_idx.push_back(az * kH + ring);
        }
    }
    return c;
}

} // namespace

TEST(NoReturnDetector, MissingRunWithLiveBaselineDetected) {
    const auto cloud = make_cloud_with_holes({{200, 209, 4}}, {});
    const auto ri = ri_of(cloud);
    ASSERT_TRUE(ri.valid());

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);

    ASSERT_EQ(set.cells.size(), 10u);
    ASSERT_EQ(set.cells_score.size(), 10u);
    for (size_t k = 0; k < set.cells.size(); ++k) {
        const uint32_t cell = set.cells[k];
        EXPECT_EQ(cell % ri.height, 4u);
        EXPECT_EQ(ri.no_return_mask[cell], 1u);
        EXPECT_GE(set.cells_score[k], 0.7f);
    }
}

TEST(NoReturnDetector, ShortRunNotFlagged) {
    const auto cloud = make_cloud_with_holes({{200, 202, 4}}, {});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}

TEST(NoReturnDetector, DeadRingNotFlagged) {
    const auto cloud = make_cloud_with_holes({{0, kW - 1, 4}}, {});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}

TEST(NoReturnDetector, HugeRunNotFlagged) {
    const auto cloud = make_cloud_with_holes({{0, 399, 4}}, {});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}

TEST(NoReturnDetector, DeadBaselineNotFlagged) {
    const auto cloud = make_cloud_with_holes({{200, 209, 4}}, {{195, 199, 4}, {210, 214, 4}});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}

TEST(GeometryResidualDetector, SparseFarCandidateUsesFarGate) {
    const auto cloud = make_wall_cloud(100.0f, 200, 203, 2, 4, 140.0f);
    const auto ri = ri_of(cloud);
    auto params = make_geom_params(1);
    params.max_target_range_m = 150.0f;
    params.far_range_m = 60.0f;
    params.min_cells = 50;
    params.min_cells_far = 8;
    params.min_fill = 0.3f;
    params.min_fill_far = 0.12f;
    argus::GeometryResidualDetector detector(params);
    const auto set = detector.detect(cloud, ri);
    EXPECT_EQ(set.indices.size(), 12u);

    params.far_range_m = 120.0f;
    argus::GeometryResidualDetector near_gate(params);
    EXPECT_TRUE(near_gate.detect(cloud, ri).indices.empty());

    params.far_range_m = 60.0f;
    params.max_target_range_m = 90.0f;
    argus::GeometryResidualDetector range_gate(params);
    EXPECT_TRUE(range_gate.detect(cloud, ri).indices.empty());
}

TEST(GeometryResidualDetector, FarResidualThresholdIsIndependent) {
    const auto cloud = make_wall_cloud(100.0f, 200, 203, 2, 4, 140.0f);
    const auto ri = ri_of(cloud);
    auto params = make_geom_params(1);
    params.max_target_range_m = 150.0f;
    params.far_range_m = 60.0f;
    params.min_cells_far = 8;
    params.min_fill_far = 0.12f;
    params.residual_threshold_far_m = 60.0f;
    argus::GeometryResidualDetector detector(params);
    EXPECT_TRUE(detector.detect(cloud, ri).indices.empty());
}

TEST(GeometryResidualDetector, NearRangeMedianOverrideRecoversWideTarget) {
    const auto cloud = make_wall_cloud(20.0f, 160, 320, 1, 6, 180.0f);
    const auto ri = ri_of(cloud);
    auto params = make_geom_params(1);
    params.max_target_range_m = 150.0f;
    params.median_window_override_min_range_m = 18.5f;
    params.median_window_override_max_range_m = 40.0f;
    params.median_half_window_override = 200;
    argus::GeometryResidualDetector detector(params);
    EXPECT_FALSE(detector.detect(cloud, ri).indices.empty());
}

TEST(GeometryResidualDetector, NearRangeMedianOverrideLeavesFarTargetUnchanged) {
    const auto cloud = make_wall_cloud(100.0f, 200, 203, 2, 4, 140.0f);
    const auto ri = ri_of(cloud);
    auto base = make_geom_params(1);
    base.max_target_range_m = 150.0f;
    auto overridden = base;
    overridden.median_window_override_min_range_m = 18.5f;
    overridden.median_window_override_max_range_m = 40.0f;
    overridden.median_half_window_override = 200;
    argus::GeometryResidualDetector base_detector(base);
    argus::GeometryResidualDetector override_detector(overridden);
    EXPECT_EQ(base_detector.detect(cloud, ri).indices, override_detector.detect(cloud, ri).indices);
}

TEST(GeometryResidualDetector, NearRangeMedianOverrideLeavesFdTargetUnchanged) {
    const auto cloud = make_wall_cloud(16.9f, 200, 203, 2, 4);
    const auto ri = ri_of(cloud);
    auto base = make_geom_params(1);
    auto overridden = base;
    overridden.median_window_override_min_range_m = 18.5f;
    overridden.median_window_override_max_range_m = 40.0f;
    overridden.median_half_window_override = 200;
    argus::GeometryResidualDetector base_detector(base);
    argus::GeometryResidualDetector override_detector(overridden);
    EXPECT_EQ(base_detector.detect(cloud, ri).indices, override_detector.detect(cloud, ri).indices);
}

TEST(GeometryResidualDetector, NearRangeMedianOverrideIsScoped) {
    const auto cloud = make_wall_cloud(20.0f, 194, 209, 1, 6, 180.0f);
    const auto ri = ri_of(cloud);
    auto params = make_geom_params(1);
    params.max_target_range_m = 150.0f;
    params.median_half_window = 100;
    params.median_window_override_min_range_m = 18.5f;
    params.median_window_override_max_range_m = 40.0f;
    params.median_half_window_override = 200;
    argus::GeometryResidualDetector detector(params);
    EXPECT_FALSE(detector.detect(cloud, ri).indices.empty());
}

TEST(TemporalResidualDetector, AppearingBoxProducesIndicesAfterHistoryWarmup) {
    argus::TemporalResidualParams params;
    params.window_frames = 5;
    params.residual_threshold_m = 0.5f;
    params.min_range_m = 4.0f;
    params.max_range_m = 45.0f;
    argus::TemporalResidualDetector detector(params);
    const auto wall = make_wall_cloud(25.5f, kW, kW, 0, 0);
    const auto wall_range = ri_of(wall);
    EXPECT_TRUE(detector.detect(wall, wall_range).indices.empty());
    EXPECT_TRUE(detector.detect(wall, wall_range).indices.empty());
    const auto box = make_wall_cloud(10.0f, 230, 250, 2, 5, 25.5f);
    const auto result = detector.detect(box, ri_of(box));
    EXPECT_FALSE(result.indices.empty());
}

TEST(TemporalResidualDetector, StableSceneProducesNoCandidates) {
    argus::TemporalResidualParams params;
    argus::TemporalResidualDetector detector(params);
    const auto wall = make_wall_cloud(25.5f, kW, kW, 0, 0);
    const auto range = ri_of(wall);
    for (int frame = 0; frame < 8; ++frame) {
        EXPECT_TRUE(detector.detect(wall, range).indices.empty());
    }
}
