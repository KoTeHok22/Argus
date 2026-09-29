
#include <gtest/gtest.h>

#include "argus_core/health_monitor.hpp"

namespace {

argus::HealthMonitorParams params() {
    argus::HealthMonitorParams p;
    p.heartbeat_timeout_s = 2.0;
    p.critical_nodes = {"preprocess", "detector", "fusion"};
    return p;
}

TEST(HealthMonitor, AllAlive) {
    argus::HealthMonitor mon(params());
    mon.beat("preprocess", 10.0);
    mon.beat("detector", 10.1);
    mon.beat("fusion", 10.2);
    const argus::SystemHealth h = mon.evaluate(10.5);
    EXPECT_TRUE(h.all_critical_alive);
    EXPECT_TRUE(h.dead_critical.empty());
    EXPECT_EQ(h.nodes.size(), 3u);
    for (const auto& n : h.nodes) {
        EXPECT_TRUE(n.alive);
    }
}

TEST(HealthMonitor, DeadCriticalDetected) {
    argus::HealthMonitor mon(params());
    mon.beat("preprocess", 10.0);
    mon.beat("detector", 10.0);
    mon.beat("fusion", 10.0);
    const argus::SystemHealth h = mon.evaluate(13.0);
    EXPECT_FALSE(h.all_critical_alive);
    EXPECT_EQ(h.dead_critical, "detector,fusion,preprocess");
}

TEST(HealthMonitor, NonCriticalDeathDoesNotFailSystem) {
    argus::HealthMonitor mon(params());
    mon.beat("preprocess", 10.0);
    mon.beat("detector", 10.0);
    mon.beat("fusion", 10.0);
    mon.beat("webui", 1.0);
    const argus::SystemHealth h = mon.evaluate(10.5);
    EXPECT_TRUE(h.all_critical_alive);
    bool found_webui = false;
    for (const auto& n : h.nodes) {
        if (n.name == "webui") {
            found_webui = true;
            EXPECT_FALSE(n.alive);
            EXPECT_FALSE(n.critical);
        }
    }
    EXPECT_TRUE(found_webui);
}

TEST(HealthMonitor, NeverSeenCriticalIsDead) {
    argus::HealthMonitor mon(params());
    mon.beat("preprocess", 10.0);
    const argus::SystemHealth h = mon.evaluate(10.0);
    EXPECT_FALSE(h.all_critical_alive);
    EXPECT_EQ(h.dead_critical, "detector,fusion");
}

TEST(HealthMonitor, RecoveredNodeIsAliveAgain) {
    argus::HealthMonitor mon(params());
    mon.beat("preprocess", 10.0);
    mon.beat("detector", 10.0);
    mon.beat("fusion", 10.0);
    EXPECT_FALSE(mon.evaluate(20.0).all_critical_alive);
    mon.beat("preprocess", 20.5);
    mon.beat("detector", 20.5);
    mon.beat("fusion", 20.5);
    EXPECT_TRUE(mon.evaluate(21.0).all_critical_alive);
}

} // namespace
