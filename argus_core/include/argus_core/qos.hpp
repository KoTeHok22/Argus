#pragma once

#include <rclcpp/qos.hpp>

namespace argus {

inline rclcpp::QoS sensor_qos(std::size_t depth = 5) {
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    qos.best_effort();
    qos.durability_volatile();
    return qos;
}

inline rclcpp::QoS control_qos(std::size_t depth = 10) {
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    qos.reliable();
    return qos;
}

} // namespace argus
