// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Нода детекторов (Фаза 3, PLAN.md 8): no_return, geometry, free_space.
// Выходы: /argus/anom_nr, /argus/anom_g, /argus/anom_fs.
// Реализация - Ф3.5.1-Ф3.5.3.

#include <rclcpp/rclcpp.hpp>

namespace argus {

class DetectorNode : public rclcpp::Node {
 public:
  DetectorNode() : Node("argus_detector") {
    RCLCPP_INFO(get_logger(),
                "argus_detector: каркас. Реализация - Ф3.5.x (три детектора "
                "аномалий)");
  }
};

}  // namespace argus

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<argus::DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
