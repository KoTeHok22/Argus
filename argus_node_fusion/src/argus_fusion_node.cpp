// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Нода слияния (Фаза 4, PLAN.md 9): голосование детекторов, габарит,
// кластеризация, трекинг, тревоги. Выходы: /argus/obstacles, /argus/gauge,
// /argus/markers, /argus/explain. Реализация - Ф4.x (LifecycleNode).

#include <rclcpp/rclcpp.hpp>

namespace argus {

class FusionNode : public rclcpp::Node {
 public:
  FusionNode() : Node("argus_fusion") {
    RCLCPP_INFO(get_logger(),
                "argus_fusion: каркас. Реализация - Ф4.x (голосование, "
                "габарит, кластеризация, трекинг). Перевод на LifecycleNode "
                "- Ф5.6.6.");
  }
};

}  // namespace argus

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<argus::FusionNode>());
  rclcpp::shutdown();
  return 0;
}
