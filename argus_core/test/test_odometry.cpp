#include <cmath>
#include <map>
#include <tuple>
#include <vector>

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

TEST(Odometry, ExpandingNearestMatchesFullRadiusCube) {
    std::vector<Eigen::Vector3f> pts;
    for (int i = -8; i <= 8; ++i) {
        for (int j = -8; j <= 8; ++j) {
            for (int k = -3; k <= 3; ++k) {
                pts.emplace_back(0.51f * static_cast<float>(i) + 0.07f,
                                 0.49f * static_cast<float>(j) - 0.11f,
                                 0.53f * static_cast<float>(k) + 0.03f);
            }
        }
    }
    const float vs = 0.5f;
    const float inv = 1.0f / vs;
    const float max_c = 2.0f;
    const float max_d2 = max_c * max_c;
    const int radius = 3;
    auto key_of = [inv](const Eigen::Vector3f& p) {
        return std::tuple<int, int, int>(static_cast<int>(std::floor(p.x() * inv)),
                                         static_cast<int>(std::floor(p.y() * inv)),
                                         static_cast<int>(std::floor(p.z() * inv)));
    };
    std::map<std::tuple<int, int, int>, Eigen::Vector3f> map;
    for (const auto& p : pts) {
        map[key_of(p)] = p;
    }
    auto brute = [&](const Eigen::Vector3f& q, Eigen::Vector3f& out) {
        const auto c = key_of(q);
        float best = max_d2;
        bool found = false;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    const auto k = std::make_tuple(std::get<0>(c) + dx, std::get<1>(c) + dy,
                                                   std::get<2>(c) + dz);
                    const auto it = map.find(k);
                    if (it == map.end()) {
                        continue;
                    }
                    const float d2 = (it->second - q).squaredNorm();
                    if (d2 < best) {
                        best = d2;
                        out = it->second;
                        found = true;
                    }
                }
            }
        }
        return found;
    };
    std::vector<Eigen::Vector3f> queries = {
        {0.1f, 0.2f, 0.0f},  {1.7f, -0.4f, 0.3f}, {3.1f, 2.8f, -0.6f},
        {-2.2f, 1.1f, 0.9f}, {0.0f, 0.0f, 0.0f},  {10.0f, 0.0f, 0.0f},
    };
    for (size_t n = 0; n < 80; ++n) {
        queries.push_back(pts[n * 17 % pts.size()] + Eigen::Vector3f(0.12f, -0.05f, 0.02f));
    }
    for (const auto& q : queries) {
        Eigen::Vector3f a;
        Eigen::Vector3f b;
        const bool ok_a = argus::icp_find_nearest(pts, q, vs, max_c, a);
        const bool ok_b = brute(q, b);
        EXPECT_EQ(ok_a, ok_b);
        if (ok_a && ok_b) {
            EXPECT_NEAR((a - q).squaredNorm(), (b - q).squaredNorm(), 1e-6f);
        }
    }
}
