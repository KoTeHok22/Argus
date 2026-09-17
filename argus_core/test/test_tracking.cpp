
#include <gtest/gtest.h>

#include "argus_core/tracking.hpp"

namespace {

argus::Cluster make_cluster(float x, float y, float z) {
    argus::Cluster c;
    c.centroid = Eigen::Vector3f(x, y, z);
    c.nearest_range = std::sqrt(x * x + y * y + z * z);
    c.volume = 0.5f;
    c.point_count = 50;
    return c;
}

} // namespace

TEST(Tracking, ConfirmsAfterMinHits) {
    argus::TrackingParams p;
    p.min_hits_to_confirm = 3;
    argus::ObstacleTracker tr(p);

    const auto out1 = tr.update({make_cluster(10, 0, 0)}, 0.1f, 10.0f);
    const auto out2 = tr.update({make_cluster(11, 0, 0)}, 0.1f, 10.0f);
    const auto out3 = tr.update({make_cluster(12, 0, 0)}, 0.1f, 10.0f);
    EXPECT_TRUE(out1.empty());
    EXPECT_TRUE(out2.empty());
    ASSERT_EQ(out3.size(), 1u);
    EXPECT_EQ(out3[0].hits, 3u);
}

TEST(Tracking, KeepsIdAcrossFrames) {
    argus::TrackingParams p;
    p.min_hits_to_confirm = 1;
    argus::ObstacleTracker tr(p);

    const auto a = tr.update({make_cluster(10, 0, 0)}, 0.1f, 10.0f);
    ASSERT_EQ(a.size(), 1u);
    const uint32_t id = a[0].id;
    const auto b = tr.update({make_cluster(11, 0, 0)}, 0.1f, 10.0f);
    ASSERT_EQ(b.size(), 1u);
    EXPECT_EQ(b[0].id, id);
}

TEST(Tracking, SurvivesMissesThenResets) {
    argus::TrackingParams p;
    p.min_hits_to_confirm = 1;
    p.max_misses_to_keep = 4;
    argus::ObstacleTracker tr(p);

    tr.update({make_cluster(10, 0, 0)}, 0.1f, 10.0f);
    const auto after_misses = tr.update({}, 0.1f, 10.0f);
    ASSERT_EQ(tr.all_tracks().size(), 1u);
    const auto after_misses2 = tr.update({}, 0.1f, 10.0f);
    ASSERT_EQ(tr.all_tracks().size(), 1u);
    EXPECT_EQ(after_misses.size() + after_misses2.size(), 2u);
    for (int i = 0; i < 5; ++i) {
        tr.update({}, 0.1f, 10.0f);
    }
    EXPECT_TRUE(tr.all_tracks().empty());
}

TEST(Tracking, TtcClosingAndReceding) {
    argus::Track t;
    t.state(0) = 50.0f;
    t.state(3) = 0.0f;

    EXPECT_NEAR(argus::compute_ttc(t, 10.0f), 5.0f, 1e-4f);

    t.state(3) = 12.0f;
    EXPECT_FLOAT_EQ(argus::compute_ttc(t, 10.0f), -1.0f);
}
