
#include <gtest/gtest.h>

#include "argus_core/frame_sync.hpp"

namespace {

argus::FrameSyncParams params() {
    argus::FrameSyncParams p;
    p.tolerance_ns = 20000000;
    p.max_pending = 4;
    p.require_temporal = false;
    return p;
}

TEST(FrameSync, ExactStampsComplete) {
    argus::FrameSynchronizer sync(params());
    const uint64_t t = 1000000000;
    const uint64_t k1 = sync.add(argus::FrameChannel::cloud, t);
    const uint64_t k2 = sync.add(argus::FrameChannel::geometry, t);
    const uint64_t k3 = sync.add(argus::FrameChannel::no_return, t);
    EXPECT_EQ(k1, k2);
    EXPECT_EQ(k2, k3);
    EXPECT_TRUE(sync.complete(k1));
    EXPECT_EQ(sync.stats().snapped, 0u);
}

TEST(FrameSync, CloseStampsSnapToOneFrame) {
    argus::FrameSynchronizer sync(params());
    const uint64_t t = 1000000000;
    const uint64_t kc = sync.add(argus::FrameChannel::cloud, t);
    const uint64_t kg = sync.add(argus::FrameChannel::geometry, t + 3000000);
    const uint64_t kn = sync.add(argus::FrameChannel::no_return, t - 2000000);
    EXPECT_EQ(kc, kg);
    EXPECT_EQ(kc, kn);
    EXPECT_TRUE(sync.complete(kc));
    EXPECT_EQ(sync.stats().snapped, 2u);
}

TEST(FrameSync, DistantStampsStaySeparate) {
    argus::FrameSynchronizer sync(params());
    const uint64_t t = 1000000000;
    const uint64_t k1 = sync.add(argus::FrameChannel::cloud, t);
    const uint64_t k2 = sync.add(argus::FrameChannel::cloud, t + 30000000);
    EXPECT_NE(k1, k2);
    EXPECT_FALSE(sync.complete(k1));
    EXPECT_FALSE(sync.complete(k2));
    EXPECT_EQ(sync.pending(), 2u);
}

TEST(FrameSync, TemporalRequiredWhenEnabled) {
    argus::FrameSyncParams p = params();
    p.require_temporal = true;
    argus::FrameSynchronizer sync(p);
    const uint64_t t = 1000000000;
    const uint64_t k = sync.add(argus::FrameChannel::cloud, t);
    sync.add(argus::FrameChannel::geometry, t);
    sync.add(argus::FrameChannel::no_return, t);
    EXPECT_FALSE(sync.complete(k));
    sync.add(argus::FrameChannel::temporal, t + 1000000);
    EXPECT_TRUE(sync.complete(k));
}

TEST(FrameSync, ExpiryCountsMissingChannels) {
    argus::FrameSynchronizer sync(params());
    const uint64_t t = 1000000000;
    sync.add(argus::FrameChannel::cloud, t);
    for (int i = 1; i <= 4; ++i) {
        const uint64_t ti = t + static_cast<uint64_t>(i) * 100000000;
        sync.add(argus::FrameChannel::cloud, ti);
    }
    const auto expired = sync.take_expired();
    ASSERT_EQ(expired.size(), 1u);
    EXPECT_EQ(expired[0], t);
    EXPECT_EQ(sync.stats().expired, 1u);
    EXPECT_EQ(sync.stats().expired_missing_cloud, 0u);
    EXPECT_EQ(sync.stats().expired_missing_geometry, 1u);
    EXPECT_EQ(sync.stats().expired_missing_no_return, 1u);
    EXPECT_TRUE(sync.take_expired().empty());
}

TEST(FrameSync, CompleteFramesExpireNothing) {
    argus::FrameSynchronizer sync(params());
    const uint64_t t = 1000000000;
    for (int i = 0; i < 8; ++i) {
        const uint64_t ti = t + static_cast<uint64_t>(i) * 100000000;
        sync.add(argus::FrameChannel::cloud, ti);
        sync.add(argus::FrameChannel::geometry, ti);
        sync.add(argus::FrameChannel::no_return, ti);
        ASSERT_TRUE(sync.complete(ti));
        sync.release(ti);
    }
    EXPECT_EQ(sync.stats().expired, 0u);
    EXPECT_EQ(sync.pending(), 0u);
}

TEST(FrameSync, ReleaseUnknownKeyIsSafe) {
    argus::FrameSynchronizer sync(params());
    sync.release(42);
    EXPECT_EQ(sync.pending(), 0u);
}

} // namespace
