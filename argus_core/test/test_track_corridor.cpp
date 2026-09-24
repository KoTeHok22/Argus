#include <limits>

#include <gtest/gtest.h>

#include "argus_core/track_corridor.hpp"

TEST(TrackCorridor, TurningWallInStraightLidarGaugeIsOutsideTrack) {
    argus::TrackCorridor corridor({1.5f, 5.0f, 1.0f});
    ASSERT_TRUE(corridor.set_centerline(
        {{0.0f, 0.0f}, {0.0f, -5.0f}, {-2.0f, -9.0f}, {-5.0f, -13.0f}, {-8.0f, -17.0f}}));
    float ahead = 0.0f;
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, -17.0f}, ahead));
    EXPECT_TRUE(corridor.contains({0.0f, 0.0f}, {-5.0f, -13.0f}, ahead));
    EXPECT_GT(ahead, 10.0f);
}

TEST(TrackCorridor, OnlyObservedCenterlineIsTrusted) {
    argus::TrackCorridor corridor({1.5f, 5.0f, 1.0f});
    float ahead = 0.0f;
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, -20.0f}, ahead));
    ASSERT_TRUE(corridor.set_centerline({{0.0f, 0.0f}, {0.0f, -5.0f}, {0.0f, -10.0f}}));
    EXPECT_TRUE(corridor.contains({0.0f, 0.0f}, {0.0f, -9.5f}, ahead));
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, -20.0f}, ahead));
    EXPECT_FALSE(corridor.contains({2.0f, 0.0f}, {0.0f, -9.5f}, ahead));
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, 1.0f}, ahead));
}

TEST(TrackCorridor, RejectsUnobservedGapsAndInvalidCoordinates) {
    argus::TrackCorridor corridor({1.5f, 5.0f, 1.0f});
    float ahead = 0.0f;
    EXPECT_FALSE(corridor.set_centerline({{0.0f, 0.0f}, {0.0f, -100.0f}}));
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, -20.0f}, ahead));
    EXPECT_FALSE(
        corridor.set_centerline({{0.0f, 0.0f}, {std::numeric_limits<float>::quiet_NaN(), -5.0f}}));
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, -4.0f}, ahead));
}

TEST(TrackCorridor, KeepsDistanceAlongCurvedPath) {
    argus::TrackCorridor corridor({1.5f, 5.0f, 1.0f});
    ASSERT_TRUE(corridor.set_centerline({{0.0f, 0.0f}, {0.0f, -5.0f}, {-3.0f, -9.0f}}));
    float ahead = 0.0f;
    EXPECT_TRUE(corridor.contains({0.0f, 0.0f}, {-3.0f, -9.0f}, ahead));
    EXPECT_NEAR(ahead, 10.0f, 1e-4f);
}

TEST(TrackCorridor, SupportsTwentySixtyAndHundredMetersOnlyWhenObserved) {
    argus::TrackCorridor corridor({1.5f, 5.0f, 1.0f});
    std::vector<Eigen::Vector2f> centerline;
    for (int i = 0; i <= 20; ++i) {
        centerline.emplace_back(0.0f, -5.0f * static_cast<float>(i));
    }
    ASSERT_TRUE(corridor.set_centerline(centerline));
    float ahead = 0.0f;
    for (float distance : {20.0f, 60.0f, 100.0f}) {
        EXPECT_TRUE(corridor.contains({0.0f, 0.0f}, {0.0f, -distance}, ahead));
        EXPECT_NEAR(ahead, distance, 1e-4f);
        EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {4.0f, -distance}, ahead));
    }
    EXPECT_FALSE(corridor.contains({0.0f, 0.0f}, {0.0f, -101.0f}, ahead));
}
