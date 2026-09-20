
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace argus {

struct HealthMonitorParams {
    double heartbeat_timeout_s = 2.0;
    std::vector<std::string> critical_nodes;
};

struct NodeHealth {
    std::string name;
    double last_beat_s = -1.0;
    bool alive = false;
    bool critical = false;
};

struct SystemHealth {
    bool all_critical_alive = true;
    std::vector<NodeHealth> nodes;
    std::string dead_critical;
};

class HealthMonitor {
public:
    explicit HealthMonitor(const HealthMonitorParams& p);

    void beat(const std::string& node, double now_s);

    SystemHealth evaluate(double now_s) const;

private:
    HealthMonitorParams p_;
    std::map<std::string, double> last_beat_;
};

} // namespace argus
