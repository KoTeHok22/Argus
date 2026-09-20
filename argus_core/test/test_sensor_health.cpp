
#include <gtest/gtest.h>

#include "argus_core/sensor_health.hpp"

namespace {

argus::SensorHealthParams params() {
    argus::SensorHealthParams p;
    p.max_no_return_ratio = 0.6f;
    p.min_valid_ratio = 0.10f;
    p.valid_drop_factor = 0.5f;
    p.warmup_frames = 10;
    p.bad_frames_to_raise = 5;
    p.good_frames_to_clear = 5;
    return p;
}

TEST(SensorHealth, CleanStreamStaysOk) {
    argus::SensorHealthMonitor mon(params());
    for (int i = 0; i < 60; ++i) {
        const auto st = mon.update(300000, 90000, 30000);
        EXPECT_TRUE(st.ok);
    }
}

TEST(SensorHealth, BlockedWindowRaises) {
    argus::SensorHealthMonitor mon(params());
    for (int i = 0; i < 20; ++i) {
        mon.update(300000, 90000, 30000);
    }
    argus::SensorHealthStatus st;
    for (int i = 0; i < 4; ++i) {
        st = mon.update(300000, 2000, 250000);
        EXPECT_TRUE(st.ok);
    }
    st = mon.update(300000, 2000, 250000);
    EXPECT_FALSE(st.ok);
    EXPECT_EQ(st.bad_streak, 5u);
}

TEST(SensorHealth, ValidDropBelowBaselineRaises) {
    argus::SensorHealthMonitor mon(params());
    for (int i = 0; i < 20; ++i) {
        mon.update(300000, 90000, 30000);
    }
    argus::SensorHealthStatus st;
    for (int i = 0; i < 5; ++i) {
        st = mon.update(300000, 30000, 30000);
    }
    EXPECT_FALSE(st.ok);
}

TEST(SensorHealth, RecoversAfterGoodFrames) {
    argus::SensorHealthMonitor mon(params());
    for (int i = 0; i < 20; ++i) {
        mon.update(300000, 90000, 30000);
    }
    for (int i = 0; i < 6; ++i) {
        mon.update(300000, 2000, 250000);
    }
    argus::SensorHealthStatus st;
    for (int i = 0; i < 4; ++i) {
        st = mon.update(300000, 90000, 30000);
        EXPECT_FALSE(st.ok);
    }
    st = mon.update(300000, 90000, 30000);
    EXPECT_TRUE(st.ok);
}

TEST(SensorHealth, NoFalseRaiseDuringWarmup) {
    argus::SensorHealthMonitor mon(params());
    argus::SensorHealthStatus st;
    for (int i = 0; i < 3; ++i) {
        st = mon.update(300000, 90000, 30000);
    }
    for (int i = 0; i < 4; ++i) {
        st = mon.update(300000, 20000, 30000);
    }
    EXPECT_TRUE(st.ok);
}

TEST(SensorHealth, ZeroRawDoesNotCrash) {
    argus::SensorHealthMonitor mon(params());
    const auto st = mon.update(0, 0, 0);
    EXPECT_FLOAT_EQ(st.no_return_ratio, 0.0f);
    EXPECT_FLOAT_EQ(st.valid_ratio, 0.0f);
}

} // namespace
