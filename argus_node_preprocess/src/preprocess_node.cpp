// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Нода предобработки (Фаза 1, PLAN.md §6).
// Вход:  sensor_msgs/PointCloud2 (топик из параметра cloud.input_topic)
// Выход: /argus/clean — PointCloud2 после sanity-фильтра,
//        /argus/diagnostics — счётчики отбраковки и структура развёртки.

#include <chrono>
#include <cstring>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <argus_msgs/msg/diagnostics.hpp>

#include "argus_core/cloud_filter.hpp"
#include "argus_core/range_image.hpp"

namespace argus {

using sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Diagnostics = argus_msgs::msg::Diagnostics;

class PreprocessNode : public rclcpp::Node {
public:
    PreprocessNode() : Node("argus_preprocess") {
        input_topic_ = declare_parameter<std::string>("cloud.input_topic", "/lidar_points");
        params_.min_range = static_cast<float>(declare_parameter<double>("cloud.min_range", 0.5));
        params_.max_range = static_cast<float>(declare_parameter<double>("cloud.max_range", 400.0));
        params_.max_abs_coord =
            static_cast<float>(declare_parameter<double>("cloud.max_abs_coord", 1.0e4));
        params_.drop_zero_xyz = declare_parameter<bool>("cloud.drop_zero_xyz", true);
        params_.drop_nonfinite = declare_parameter<bool>("cloud.drop_nonfinite", true);
        ri_params_.rings_fallback =
            static_cast<uint32_t>(declare_parameter<int>("cloud.rings", 128));

        auto qos = rclcpp::QoS(10);
        sub_ = create_subscription<PointCloud2>(
            input_topic_, qos, [this](PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
        pub_clean_ = create_publisher<PointCloud2>("/argus/clean", qos);
        pub_diag_ = create_publisher<Diagnostics>("/argus/diagnostics", qos);

        RCLCPP_INFO(get_logger(), "argus_preprocess подписан на '%s'", input_topic_.c_str());
    }

private:
    static float ms_since(std::chrono::steady_clock::time_point t0) {
        const auto dt = std::chrono::steady_clock::now() - t0;
        return static_cast<float>(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt).count());
    }

    static PointCloud2 make_clean_msg(const CleanCloud& cloud, const rclcpp::Time& stamp) {
        PointCloud2 out;
        out.header.stamp = stamp;
        out.header.frame_id = cloud.frame_id;
        out.height = 1;
        out.width = static_cast<uint32_t>(cloud.size());
        out.point_step = 16; // x@0 y@4 z@8 intensity@12, все float32
        out.row_step = out.width * out.point_step;
        out.is_dense = true;

        auto add = [&out](const std::string& name, uint32_t offset) {
            PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = PointField::FLOAT32;
            f.count = 1;
            out.fields.push_back(f);
        };
        add("x", 0);
        add("y", 4);
        add("z", 8);
        add("intensity", 12);

        out.data.resize(out.data.size() + static_cast<size_t>(out.width) * out.point_step);
        for (size_t i = 0; i < cloud.size(); ++i) {
            uint8_t* p = out.data.data() + i * out.point_step;
            std::memcpy(p + 0, &cloud.x[i], 4);
            std::memcpy(p + 4, &cloud.y[i], 4);
            std::memcpy(p + 8, &cloud.z[i], 4);
            std::memcpy(p + 12, &cloud.intensity[i], 4);
        }
        return out;
    }

    void on_cloud(PointCloud2::ConstSharedPtr msg) {
        const auto t_start = std::chrono::steady_clock::now();
        const rclcpp::Time stamp = msg->header.stamp;

        const auto layout = parse_layout(*msg);
        if (!layout.has_value()) {
            // Один раз залогировать, дальше — только диагностика (не спамить).
            if (!layout_error_logged_) {
                RCLCPP_ERROR(get_logger(), "раскладка отвергнута: %s", layout->error.c_str());
                layout_error_logged_ = true;
            }
            return;
        }

        FilterStats stats;
        const auto t_filter0 = std::chrono::steady_clock::now();
        const CleanCloud cloud = filter_and_index(*msg, *layout, params_, &stats);
        const float t_filter_ms = ms_since(t_filter0);

        const auto t_ri0 = std::chrono::steady_clock::now();
        const RangeImage ri = build_range_image(cloud, ri_params_);
        const float t_range_image_ms = ms_since(t_ri0);

        pub_clean_->publish(make_clean_msg(cloud, stamp));

        Diagnostics d;
        d.header.stamp = stamp;
        d.header.frame_id = cloud.frame_id;
        d.n_points_raw = stats.n_raw;
        d.n_points_valid = stats.n_valid;
        d.n_points_rejected = stats.n_rejected;
        d.reject_zero = stats.reject_zero;
        d.reject_nonfinite = stats.reject_nonfinite;
        d.reject_out_of_range = stats.reject_out_of_range;
        d.rings_detected = ri.height;
        d.az_steps_detected = ri.width;
        d.echo_multiplier = cloud.echo_multiplier;
        d.point_step = layout->point_step;
        d.layout_ok = true;
        d.t_filter_ms = t_filter_ms;
        d.t_range_image_ms = t_range_image_ms;
        d.fps = (t_total_ms_prev_ > 0.0f) ? 1000.0f / t_total_ms_prev_ : 0.0f;
        d.frames_processed = ++frames_processed_;
        pub_diag_->publish(d);

        t_total_ms_prev_ = ms_since(t_start);
        RCLCPP_DEBUG(get_logger(),
                     "raw=%u valid=%u zero=%u nonfinite=%u range=%u | ri %ux%u | "
                     "filter %.2f ms, ri %.2f ms",
                     stats.n_raw, stats.n_valid, stats.reject_zero, stats.reject_nonfinite,
                     stats.reject_out_of_range, ri.width, ri.height, t_filter_ms, t_range_image_ms);
    }

    std::string input_topic_;
    FilterParams params_;
    RangeImageParams ri_params_;
    bool layout_error_logged_ = false;
    uint64_t frames_processed_ = 0;
    float t_total_ms_prev_ = 0.0f;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_clean_;
    rclcpp::Publisher<Diagnostics>::SharedPtr pub_diag_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::PreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
