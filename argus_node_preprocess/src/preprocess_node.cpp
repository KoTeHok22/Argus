// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Нода предобработки (Фаза 1, PLAN.md §6).
// Вход:  sensor_msgs/PointCloud2 (топик из параметра cloud.input_topic)
// Выход: /argus/clean — PointCloud2 после sanity-фильтра (каркас),
//        /argus/diagnostics — счётчики отбраковки.
//
// Фаза 1 (Ф1.2.x) реализует: динамический parse_layout, filter_and_index,
// build_range_image, диагностику. Здесь — рабочий каркас подписки.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "argus_core/cloud_filter.hpp"

namespace argus {

class PreprocessNode : public rclcpp::Node {
 public:
  PreprocessNode() : Node("argus_preprocess") {
    input_topic_ = declare_parameter<std::string>("cloud.input_topic",
                                                  "/lidar_points");
    params_.min_range = static_cast<float>(
        declare_parameter<double>("cloud.min_range", 0.5));
    params_.max_range = static_cast<float>(
        declare_parameter<double>("cloud.max_range", 400.0));
    params_.max_abs_coord = static_cast<float>(
        declare_parameter<double>("cloud.max_abs_coord", 1.0e4));
    params_.drop_zero_xyz =
        declare_parameter<bool>("cloud.drop_zero_xyz", true);
    params_.drop_nonfinite =
        declare_parameter<bool>("cloud.drop_nonfinite", true);

    auto qos = rclcpp::QoS(10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          on_cloud(std::move(msg));
        });

    RCLCPP_INFO(get_logger(), "argus_preprocess подписан на '%s'",
                input_topic_.c_str());
  }

 private:
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    const auto layout = parse_layout(*msg);
    if (!layout.has_value()) {
      // Один раз залогировать, дальше — только диагностика (не спамить).
      if (!layout_error_logged_) {
        RCLCPP_ERROR(get_logger(), "раскладка отвергнута: %s",
                     layout->error.c_str());
        layout_error_logged_ = true;
      }
      return;
    }

    FilterStats stats;
    const CleanCloud cloud = filter_and_index(*msg, *layout, params_, &stats);

    RCLCPP_DEBUG(get_logger(),
                 "raw=%u valid=%u zero=%u nonfinite=%u range=%u",
                 stats.n_raw, stats.n_valid, stats.reject_zero,
                 stats.reject_nonfinite, stats.reject_out_of_range);

    // Ф1.2.2: build_range_image(cloud, ...);
    // Ф1.2.4: публикация /argus/diagnostics.
  }

  std::string input_topic_;
  FilterParams params_;
  bool layout_error_logged_ = false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace argus

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<argus::PreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
