// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Нода одометрии (Фаза 2, PLAN.md 7.1.1): KISS-ICP + ограничение 4DoF.
// Выход: /argus/pose. Реализация - Ф2.3.1.

#include <rclcpp/rclcpp.hpp>

namespace argus {

class OdometryNode : public rclcpp::Node {
 public:
  OdometryNode() : Node("argus_odometry") {
    RCLCPP_INFO(get_logger(),
                "argus_odometry: каркас. Реализация - Ф2.3.1 (KISS-ICP, 4DoF)");
  }
};

}  // namespace argus

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<argus::OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
