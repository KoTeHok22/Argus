
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "argus_core/health_monitor.hpp"

namespace argus {

class WatchdogNode : public rclcpp::Node {
public:
    WatchdogNode() : Node("argus_watchdog") {
        HealthMonitorParams p;
        p.heartbeat_timeout_s = declare_parameter<double>("watchdog.heartbeat_timeout_s", 2.0);
        p.critical_nodes = declare_parameter<std::vector<std::string>>(
            "watchdog.critical_nodes", {"argus_preprocess", "argus_detector", "argus_fusion"});
        if (p.heartbeat_timeout_s <= 0.0) {
            throw std::invalid_argument("watchdog.heartbeat_timeout_s must be > 0");
        }
        monitor_ = std::make_unique<HealthMonitor>(p);

        const double period_s = declare_parameter<double>("watchdog.eval_period_s", 0.5);
        if (period_s <= 0.0) {
            throw std::invalid_argument("watchdog.eval_period_s must be > 0");
        }

        auto qos = rclcpp::QoS(10);
        sub_ = create_subscription<std_msgs::msg::String>(
            "/argus/heartbeat", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
                monitor_->beat(msg->data, now_seconds());
            });
        pub_ = create_publisher<std_msgs::msg::String>("/argus/system_health", qos);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::duration<double>(period_s)),
                                   [this] { on_timer(); });

        RCLCPP_INFO(get_logger(), "argus_watchdog: timeout %.1f s, критических узлов %zu",
                    p.heartbeat_timeout_s, p.critical_nodes.size());
    }

private:
    double now_seconds() const { return static_cast<double>(now().seconds()); }

    void on_timer() {
        const SystemHealth health = monitor_->evaluate(now_seconds());
        std_msgs::msg::String msg;
        msg.data = health.all_critical_alive ? "OK" : "DEAD:" + health.dead_critical;
        pub_->publish(msg);
        if (!health.all_critical_alive) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "критические узлы молчат: %s",
                                 health.dead_critical.c_str());
        }
    }

    std::unique_ptr<HealthMonitor> monitor_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::WatchdogNode>());
    rclcpp::shutdown();
    return 0;
}
