#include <gtest/gtest.h>

#include "argus_core/verifier.hpp"

namespace argus {
namespace {

VerifierParams enabled_params() {
    VerifierParams p;
    p.enabled = true;
    return p;
}

Cluster make_cluster(float sx, float sy, float sz, uint32_t points, float range, float score) {
    Cluster c;
    c.min_corner = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    c.max_corner = Eigen::Vector3f(sx, sy, sz);
    c.point_count = points;
    c.nearest_range = range;
    c.score = score;
    return c;
}

TEST(Verifier, AcceptsStandaloneDenseObject) {
    ClusterVerifier v(enabled_params());
    const Cluster c = make_cluster(0.5f, 0.5f, 0.5f, 1200, 20.0f, 0.9f);
    const VerifierVerdict verdict = v.verify(c);
    EXPECT_TRUE(verdict.accepted);
    EXPECT_GT(verdict.score, 0.35f);
}

TEST(Verifier, RejectsThinWallSlab) {
    ClusterVerifier v(enabled_params());
    const Cluster c = make_cluster(4.0f, 0.15f, 3.0f, 400, 25.0f, 0.2f);
    const VerifierVerdict verdict = v.verify(c);
    EXPECT_FALSE(verdict.accepted);
    EXPECT_EQ(verdict.reason, "rejected_wall_like");
}

TEST(Verifier, RejectsSparseFarNoise) {
    ClusterVerifier v(enabled_params());
    const Cluster c = make_cluster(0.4f, 0.4f, 0.4f, 2, 90.0f, 0.1f);
    const VerifierVerdict verdict = v.verify(c);
    EXPECT_FALSE(verdict.accepted);
}

TEST(Verifier, PassesThroughWhenDisabled) {
    VerifierParams p;
    p.enabled = false;
    ClusterVerifier v(p);
    const Cluster c = make_cluster(4.0f, 0.15f, 3.0f, 2, 200.0f, 0.0f);
    const VerifierVerdict verdict = v.verify(c);
    EXPECT_TRUE(verdict.accepted);
}

TEST(Verifier, CloserThanMinRangeIsOutOfScope) {
    ClusterVerifier v(enabled_params());
    const Cluster c = make_cluster(0.5f, 0.5f, 0.5f, 500, 8.0f, 0.9f);
    const VerifierVerdict verdict = v.verify(c);
    EXPECT_TRUE(verdict.accepted);
    EXPECT_EQ(verdict.reason, "too_close_for_verifier");
}

TEST(Verifier, BeyondRangeIsOutOfScope) {
    ClusterVerifier v(enabled_params());
    const Cluster c = make_cluster(0.5f, 0.5f, 0.5f, 10, 200.0f, 0.9f);
    const VerifierVerdict verdict = v.verify(c);
    EXPECT_TRUE(verdict.accepted);
    EXPECT_EQ(verdict.reason, "beyond_verifier_range");
}

} // namespace
} // namespace argus
