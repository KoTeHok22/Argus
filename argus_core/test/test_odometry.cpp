#include <gtest/gtest.h>

#include "argus_core/odometry.hpp"

namespace {

argus::CleanCloud make_wall_cloud(float along) {
    argus::CleanCloud c;
    for (int i = 0; i < 40; ++i) {
        for (int k = 0; k < 20; ++k) {
            const float lat = -4.0f + 0.2f * static_cast<float>(i);
            const float z = -1.0f + 0.15f * static_cast<float>(k);
            c.x.push_back(lat);
            c.y.push_back(-8.0f + along);
            c.z.push_back(z);
            c.intensity.push_back(1.0f);
            c.x.push_back(lat);
            c.y.push_back(8.0f + along);
            c.z.push_back(z);
            c.intensity.push_back(1.0f);
        }
    }
    for (int i = 0; i < 30; ++i) {
        c.x.push_back(-1.2f + 0.08f * static_cast<float>(i % 5));
        c.y.push_back(-3.0f + along - 0.4f * static_cast<float>(i));
        c.z.push_back(-0.2f);
        c.intensity.push_back(4.0f);
    }
    return c;
}

} // namespace

TEST(Odometry, FirstFrameValidIdentity) {
    argus::OdometryParams p;
    p.forward_axis = argus::ForwardAxis::NegY;
    argus::TunnelOdometry odom(p);
    const auto r = odom.update(make_wall_cloud(0.0f), 0.0);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.frames, 1u);
    EXPECT_TRUE(r.pose.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

TEST(Odometry, ForwardShiftAccumulatesAlongNegY) {
    argus::OdometryParams p;
    p.forward_axis = argus::ForwardAxis::NegY;
    p.lock_lateral = true;
    p.lock_roll = true;
    p.voxel_size = 0.4f;
    p.max_correspondence_m = 3.0f;
    argus::TunnelOdometry odom(p);
    odom.update(make_wall_cloud(0.0f), 0.0);
    const auto r = odom.update(make_wall_cloud(1.5f), 0.1);
    EXPECT_TRUE(r.valid);
    EXPECT_GT(r.n_correspondences, 20u);
    const Eigen::Vector3d t = r.pose.translation();
    EXPECT_NEAR(t.x(), 0.0, 0.4);
    EXPECT_LT(t.y(), -0.5);
    EXPECT_GT(t.y(), -3.0);
    EXPECT_LT(r.speed_mps, p.max_speed_mps);
}

TEST(Odometry, ImplausibleSpeedRejected) {
    argus::OdometryParams p;
    p.forward_axis = argus::ForwardAxis::NegY;
    p.max_speed_mps = 2.0f;
    p.voxel_size = 0.4f;
    p.max_correspondence_m = 8.0f;
    argus::TunnelOdometry odom(p);
    odom.update(make_wall_cloud(0.0f), 0.0);
    const auto r = odom.update(make_wall_cloud(8.0f), 0.1);
    EXPECT_FALSE(r.valid);
    EXPECT_TRUE(r.pose.isApprox(Eigen::Isometry3d::Identity(), 1e-6));
}

TEST(Odometry, StaticCloudStaysAtRest) {
    argus::OdometryParams p;
    p.forward_axis = argus::ForwardAxis::NegY;
    p.voxel_size = 0.4f;
    p.max_correspondence_m = 2.0f;
    p.rest_translation_m = 0.20f;
    argus::TunnelOdometry odom(p);
    const auto cloud = make_wall_cloud(0.0f);
    odom.update(cloud, 0.0);
    const auto r = odom.update(cloud, 0.1);
    EXPECT_TRUE(r.valid);
    EXPECT_NEAR(r.speed_mps, 0.0f, 1e-4f);
    EXPECT_TRUE(r.pose.isApprox(Eigen::Isometry3d::Identity(), 1e-6));
}

TEST(Odometry, DisabledStaysIdentity) {
    argus::OdometryParams p;
    p.enabled = false;
    argus::TunnelOdometry odom(p);
    const auto r = odom.update(make_wall_cloud(2.0f), 0.1);
    EXPECT_TRUE(r.valid);
    EXPECT_TRUE(r.pose.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}
