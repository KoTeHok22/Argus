
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "argus_core/fusion.hpp"
#include "argus_core/tunnel_model.hpp"

namespace {

constexpr uint32_t kW = 240;
constexpr uint32_t kH = 12;
constexpr float kWallRange = 20.0f;

argus::CleanCloud make_tunnel(bool with_obstacle) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const float z = 0.6f + 0.12f * static_cast<float>(ring);
            const bool blocked = with_obstacle && ring < 4;
            const float range = blocked ? kWallRange - 6.0f : kWallRange;
            c.x.push_back(0.0f);
            c.y.push_back(-range);
            c.z.push_back(z);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    return c;
}

argus::TunnelModelParams make_params() {
    argus::TunnelModelParams p;
    p.voxel_size = 0.5f;
    p.min_observations = 3;
    p.free_space_margin = 0.15f;
    p.max_range = 60.0f;
    return p;
}

argus::RangeImage ri_of(const argus::CleanCloud& cloud) {
    argus::RangeImageParams rp;
    rp.rings_fallback = kH;
    return argus::build_range_image(cloud, rp);
}

} // namespace

TEST(TunnelModel, ObservingSameCloudCarvesTheRayPath) {
    argus::TunnelModel model(make_params());
    const argus::CleanCloud cloud = make_tunnel(false);
    EXPECT_FALSE(model.ready());

    for (int i = 0; i < 3; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    EXPECT_TRUE(model.ready());

    float confidence = 0.0f;
    EXPECT_TRUE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 0.6f), confidence));
    EXPECT_GT(confidence, 0.5f);

    confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(Eigen::Vector3f(0.0f, -25.0f, 0.6f), confidence));
}

TEST(TunnelModel, EmptyHistoryDoesNotFlagAnything) {
    argus::TunnelModelParams p = make_params();
    p.min_observations = 1;
    argus::TunnelModel model(p);
    float confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 0.6f), confidence));
    EXPECT_FLOAT_EQ(confidence, 0.0f);

    const argus::CleanCloud cloud = make_tunnel(false);
    model.integrate(cloud, Eigen::Isometry3d::Identity());
    const argus::FreeSpaceCheck check = model.check(cloud, Eigen::Isometry3d::Identity(), 0.5f);
    EXPECT_EQ(check.empty, 0u);
    EXPECT_EQ(check.observed, 0u);
    EXPECT_FLOAT_EQ(check.violation_rate(), 0.0f);
}

TEST(TunnelModel, SingleObservationIsNotEnough) {
    argus::TunnelModelParams p = make_params();
    p.min_observations = 3;
    argus::TunnelModel model(p);
    const argus::CleanCloud cloud = make_tunnel(false);
    for (int i = 0; i < 2; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    float confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 2.0f), confidence));
}

TEST(TunnelModel, ObstacleInsideFreeSpaceIsDetected) {
    argus::TunnelModelParams p = make_params();
    argus::TunnelModel model(p);
    const argus::CleanCloud empty = make_tunnel(false);
    for (int i = 0; i < 5; ++i) {
        model.integrate(empty, Eigen::Isometry3d::Identity());
    }

    const argus::CleanCloud blocked = make_tunnel(true);
    const argus::FreeSpaceCheck check = model.check(blocked, Eigen::Isometry3d::Identity(), 0.5f);
    EXPECT_GT(check.empty, 0u);
    EXPECT_GT(check.violation_rate(), 0.0f);

    float confidence = 0.0f;
    EXPECT_TRUE(model.violates_free_space(Eigen::Vector3f(0.0f, -14.0f, 0.6f), confidence));
}

TEST(TunnelModel, CleanPassKeepsViolationRateUnderOnePercent) {
    argus::TunnelModelParams p = make_params();
    argus::TunnelModel model(p);
    const argus::CleanCloud cloud = make_tunnel(false);
    float rate = 1.0f;
    for (int i = 0; i < 200; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
        if (i % 20 == 19) {
            const argus::FreeSpaceCheck check =
                model.check(cloud, Eigen::Isometry3d::Identity(), 0.7f);
            rate = check.violation_rate();
        }
    }
    EXPECT_LT(rate, 0.01f);
    const argus::TunnelModelStats s = model.stats();
    EXPECT_TRUE(s.ready);
    EXPECT_GT(s.n_free, 0u);
    EXPECT_GT(s.free_ratio, 0.0f);
}

TEST(TunnelModel, ObjectPresentFromFirstFrameIsNotCarvedAsFree) {
    argus::TunnelModel model(make_params());
    const argus::CleanCloud blocked = make_tunnel(true);
    for (int i = 0; i < 30; ++i) {
        model.integrate(blocked, Eigen::Isometry3d::Identity());
    }
    float confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(Eigen::Vector3f(0.0f, -14.0f, 0.6f), confidence));
}
TEST(TunnelModel, GroundMaskedPointsDoNotCarve) {
    argus::TunnelModel model(make_params());
    argus::CleanCloud cloud = make_tunnel(false);
    cloud.ground_mask.assign(cloud.size(), 1);
    for (int i = 0; i < 5; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    EXPECT_EQ(model.voxel_count(), 0u);
    EXPECT_FALSE(model.ready());
}

TEST(TunnelModel, PoseShiftsIntegrationIntoWorldFrame) {
    argus::TunnelModel model(make_params());
    const argus::CleanCloud cloud = make_tunnel(false);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.pretranslate(Eigen::Vector3d(0.0, -12.0, 0.0));
    for (int i = 0; i < 5; ++i) {
        model.integrate(cloud, pose);
    }

    float confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 1.1f), confidence));
    EXPECT_TRUE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 1.1f), confidence, pose));
}

TEST(TunnelModel, SaveAndLoadKeepsFreeSpace) {
    argus::TunnelModel model(make_params());
    const argus::CleanCloud cloud = make_tunnel(false);
    for (int i = 0; i < 5; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    const std::string path = "argus_tunnel_model_test.bin";
    ASSERT_TRUE(model.save(path));

    argus::TunnelModel restored(make_params());
    ASSERT_TRUE(restored.load(path));
    EXPECT_EQ(restored.voxel_count(), model.voxel_count());
    float confidence = 0.0f;
    EXPECT_TRUE(restored.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 0.6f), confidence));
    std::remove(path.c_str());
}

TEST(TunnelModel, ExportOccupancySeparatesFreeAndSolid) {
    argus::TunnelModel model(make_params());
    const argus::CleanCloud cloud = make_tunnel(false);
    for (int i = 0; i < 5; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    std::vector<Eigen::Vector3f> pts;
    std::vector<float> conf;
    model.export_occupancy(pts, conf, false);
    ASSERT_EQ(pts.size(), conf.size());
    EXPECT_GT(pts.size(), 0u);
    model.export_occupancy(pts, conf, true);
    EXPECT_GT(pts.size(), 0u);
}

TEST(TunnelModel, DisabledModelIntegratesNothing) {
    argus::TunnelModelParams p = make_params();
    p.enabled = false;
    argus::TunnelModel model(p);
    const argus::CleanCloud cloud = make_tunnel(false);
    for (int i = 0; i < 10; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    EXPECT_EQ(model.voxel_count(), 0u);
    EXPECT_FALSE(model.ready());
}

argus::CleanCloud make_indexed_tunnel(uint32_t w, uint32_t h) {
    argus::CleanCloud c;
    c.rings = h;
    c.n_raw = w * h;
    for (uint32_t az = 0; az < w; ++az) {
        for (uint32_t ring = 0; ring < h; ++ring) {
            const uint32_t raw = az * h + ring;
            c.x.push_back(0.0f);
            c.y.push_back(-kWallRange);
            c.z.push_back(0.6f + 0.12f * static_cast<float>(ring));
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
            c.raw_idx.push_back(raw);
        }
    }
    return c;
}

TEST(TunnelModel, CarveStrideZeroIsTreatedAsFullCoverage) {
    argus::TunnelModelParams p = make_params();
    p.carve_stride_az = 0;
    p.carve_stride_ring = 0;
    argus::TunnelModel model(p);
    const argus::CleanCloud cloud = make_indexed_tunnel(80, 8);
    for (int i = 0; i < 5; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    float confidence = 0.0f;
    EXPECT_TRUE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 0.75f), confidence));
    EXPECT_GT(confidence, 0.5f);
}

TEST(TunnelModel, CarveStrideKeepsCoverageWhenStrideExceedsAzimuthCount) {
    argus::TunnelModelParams p = make_params();
    p.carve_stride_az = 4;
    p.carve_stride_ring = 1;
    argus::TunnelModel model(p);
    const argus::CleanCloud cloud = make_indexed_tunnel(8, 8);
    for (int i = 0; i < 5; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    float confidence = 0.0f;
    EXPECT_TRUE(model.violates_free_space(Eigen::Vector3f(0.0f, -10.0f, 0.75f), confidence));
    EXPECT_GT(confidence, 0.5f);
}

TEST(TunnelModel, CarveStopsShortOfTheSurfaceCell) {
    argus::TunnelModelParams p = make_params();
    argus::TunnelModel model(p);
    argus::CleanCloud cloud;
    cloud.rings = 1;
    cloud.n_raw = 1;
    cloud.x = {20.0f};
    cloud.y = {-4.0f};
    cloud.z = {1.1f};
    cloud.intensity = {1.0f};
    cloud.ring = {0};
    cloud.azimuth_idx = {0};
    cloud.raw_idx = {0};
    for (int i = 0; i < 6; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    float confidence = 0.0f;
    EXPECT_TRUE(model.violates_free_space(Eigen::Vector3f(19.75f, -3.75f, 1.25f), confidence));
    EXPECT_GT(confidence, 0.5f);

    confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(Eigen::Vector3f(20.25f, -3.75f, 1.25f), confidence));
}

TEST(TunnelModel, CarveMaxRangeLeavesFarCellsUnobserved) {
    argus::TunnelModelParams p = make_params();
    p.carve_max_range = 8.0f;
    argus::TunnelModel model(p);
    argus::CleanCloud cloud;
    cloud.rings = 1;
    cloud.n_raw = 1;
    cloud.x = {20.0f};
    cloud.y = {-4.0f};
    cloud.z = {1.1f};
    cloud.intensity = {1.0f};
    cloud.ring = {0};
    cloud.azimuth_idx = {0};
    cloud.raw_idx = {0};
    for (int i = 0; i < 6; ++i) {
        model.integrate(cloud, Eigen::Isometry3d::Identity());
    }
    const Eigen::Vector3f dir = Eigen::Vector3f(20.0f, -4.0f, 1.1f).normalized();
    float confidence = 0.0f;
    EXPECT_TRUE(model.violates_free_space(dir * 3.0f, confidence));

    confidence = 0.0f;
    EXPECT_FALSE(model.violates_free_space(dir * 15.0f, confidence));
}
