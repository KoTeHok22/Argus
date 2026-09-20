
#include "argus_core/health_monitor.hpp"

#include <algorithm>

namespace argus {

HealthMonitor::HealthMonitor(const HealthMonitorParams& p) : p_(p) {}

void HealthMonitor::beat(const std::string& node, double now_s) {
    last_beat_[node] = now_s;
}

SystemHealth HealthMonitor::evaluate(double now_s) const {
    SystemHealth out;
    for (const auto& [name, last] : last_beat_) {
        NodeHealth h;
        h.name = name;
        h.last_beat_s = last;
        h.alive = (now_s - last) <= p_.heartbeat_timeout_s;
        h.critical = std::find(p_.critical_nodes.begin(), p_.critical_nodes.end(), name) !=
                     p_.critical_nodes.end();
        if (h.critical && !h.alive) {
            out.all_critical_alive = false;
            if (!out.dead_critical.empty()) {
                out.dead_critical += ",";
            }
            out.dead_critical += name;
        }
        out.nodes.push_back(std::move(h));
    }
    for (const std::string& critical : p_.critical_nodes) {
        if (last_beat_.find(critical) == last_beat_.end()) {
            NodeHealth h;
            h.name = critical;
            h.alive = false;
            h.critical = true;
            out.all_critical_alive = false;
            if (!out.dead_critical.empty()) {
                out.dead_critical += ",";
            }
            out.dead_critical += critical;
            out.nodes.push_back(std::move(h));
        }
    }
    return out;
}

} // namespace argus
