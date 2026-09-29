
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include <argus_msgs/msg/clean_cloud.hpp>
#include <argus_msgs/msg/diagnostics.hpp>

#include "argus_core/cloud_filter.hpp"
#include "argus_core/ground.hpp"
#include "argus_core/range_image.hpp"
#include "argus_core/sensor_health.hpp"

namespace argus {

using sensor_msgs::msg::PointCloud2;
using CleanCloudMsg = argus_msgs::msg::CleanCloud;
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

        const int rings_param = declare_parameter<int>("range_image.rings", 0);
        if (rings_param > 0) {
            ri_params_.rings_fallback = static_cast<uint32_t>(rings_param);
        }
        ri_params_.prefer_ring_field =
            declare_parameter<bool>("range_image.prefer_ring_field", true);

        GroundParams gp;
        gp.enabled = declare_parameter<bool>("ground_segmentation.enabled", true);
        const double gs_h = declare_parameter<double>("ground_segmentation.sensor_height", -1.20);
        gp.sensor_height = static_cast<float>(std::fabs(gs_h));
        gp.rail_zone_m =
            static_cast<float>(declare_parameter<double>("ground_segmentation.rail_zone_m", 2.0));
        gp.rail_max_height = static_cast<float>(
            declare_parameter<double>("ground_segmentation.rail_max_height", 0.55));
        gp.max_ground_z_rel = static_cast<float>(
            declare_parameter<double>("ground_segmentation.max_ground_z_rel", 0.60));
        gp.min_range =
            static_cast<float>(declare_parameter<double>("ground_segmentation.min_range", 1.0));
        gp.max_range =
            static_cast<float>(declare_parameter<double>("ground_segmentation.max_range", 80.0));
        gp.enable_RNR = declare_parameter<bool>("ground_segmentation.enable_RNR", true);
        gp.enable_TGR = declare_parameter<bool>("ground_segmentation.enable_TGR", true);
        gp.num_zones = declare_parameter<int>("ground_segmentation.num_zones", 4);
        gp.num_sectors = declare_parameter<int>("ground_segmentation.num_sectors", 0);
        const std::string method_name =
            declare_parameter<std::string>("ground_segmentation.method", "z_threshold");
        if (method_name == "z" || method_name == "z_threshold") {
            gp.method = GroundMethod::ZThreshold;
        }
        const std::string axis_name =
            declare_parameter<std::string>("ground_segmentation.forward_axis", "-y");
        if (!parse_forward_axis(axis_name, gp.forward_axis)) {
            gp.forward_axis = ForwardAxis::NegY;
        }
        ground_ = std::make_unique<GroundSegmenter>(gp);

        SensorHealthParams shp;
        shp.max_no_return_ratio =
            static_cast<float>(declare_parameter<double>("sensor_health.max_no_return_ratio", 0.6));
        shp.min_valid_ratio =
            static_cast<float>(declare_parameter<double>("sensor_health.min_valid_ratio", 0.10));
        shp.valid_drop_factor =
            static_cast<float>(declare_parameter<double>("sensor_health.valid_drop_factor", 0.5));
        shp.warmup_frames =
            static_cast<uint32_t>(declare_parameter<int>("sensor_health.warmup_frames", 30));
        shp.bad_frames_to_raise =
            static_cast<uint32_t>(declare_parameter<int>("sensor_health.bad_frames_to_raise", 20));
        shp.good_frames_to_clear =
            static_cast<uint32_t>(declare_parameter<int>("sensor_health.good_frames_to_clear", 20));
        sensor_health_ = std::make_unique<SensorHealthMonitor>(shp);

        auto qos = rclcpp::QoS(10);
        sub_ = create_subscription<PointCloud2>(
            input_topic_, qos, [this](PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
        pub_clean_ = create_publisher<CleanCloudMsg>("/argus/clean", qos);
        pub_cloud_ = create_publisher<PointCloud2>("/argus/cloud", qos);
        pub_diag_ = create_publisher<Diagnostics>("/argus/diagnostics", qos);
        pub_heartbeat_ = create_publisher<std_msgs::msg::String>("/argus/heartbeat", qos);
        heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            std_msgs::msg::String beat;
            beat.data = get_name();
            pub_heartbeat_->publish(beat);
        });

        RCLCPP_INFO(get_logger(), "argus_preprocess подписан на '%s'", input_topic_.c_str());
    }

private:
    static float ms_since(std::chrono::steady_clock::time_point t0) {
        const auto dt = std::chrono::steady_clock::now() - t0;
        return static_cast<float>(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt).count());
    }

    static CleanCloudMsg make_clean_msg(const CleanCloud& cloud, const RangeImage& ri,
                                        const rclcpp::Time& stamp) {
        CleanCloudMsg out;
        out.header.stamp = stamp;
        out.header.frame_id = cloud.frame_id;
        out.n_raw = cloud.n_raw;
        out.rings = ri.height;
        out.az_steps = ri.width;
        out.echo_multiplier = cloud.echo_multiplier;
        out.x = cloud.x;
        out.y = cloud.y;
        out.z = cloud.z;
        out.intensity = cloud.intensity;
        out.raw_idx = cloud.raw_idx;
        out.no_return_raw = cloud.no_return_raw;
        out.ground_mask = cloud.ground_mask;
        return out;
    }

    static PointCloud2 make_cloud_pc2(const CleanCloud& cloud, const rclcpp::Time& stamp) {
        PointCloud2 out;
        out.header.stamp = stamp;
        out.header.frame_id = cloud.frame_id.empty() ? "lidar_livox" : cloud.frame_id;
        const size_t n = cloud.x.size();
        sensor_msgs::PointCloud2Modifier modifier(out);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(n);
        sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");
        sensor_msgs::PointCloud2Iterator<float> it_i(out, "intensity");
        for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z, ++it_i) {
            *it_x = cloud.x[i];
            *it_y = cloud.y[i];
            *it_z = cloud.z[i];
            *it_i = i < cloud.intensity.size() ? cloud.intensity[i] : 0.0f;
        }
        out.height = 1;
        out.width = static_cast<uint32_t>(n);
        out.is_dense = false;
        return out;
    }

    void on_cloud(PointCloud2::ConstSharedPtr msg) {
        const auto t_start = std::chrono::steady_clock::now();
        const rclcpp::Time stamp = msg->header.stamp;

        const auto layout = parse_layout(*msg);
        if (!layout.has_value()) {
            if (!layout_error_logged_) {
                RCLCPP_ERROR(get_logger(), "раскладка отвергнута: %s", layout->error.c_str());
                layout_error_logged_ = true;
            }
            return;
        }

        FilterStats stats;
        const auto t_filter0 = std::chrono::steady_clock::now();
        CleanCloud cloud = filter_and_index(*msg, *layout, params_, &stats);
        const float t_filter_ms = ms_since(t_filter0);

        GroundStats gst;
        const auto t_g0 = std::chrono::steady_clock::now();
        ground_->apply(cloud, &gst);
        const float t_ground_ms = ms_since(t_g0);

        const auto t_ri0 = std::chrono::steady_clock::now();
        const RangeImage ri = build_range_image(cloud, ri_params_);
        const float t_range_image_ms = ms_since(t_ri0);

        pub_clean_->publish(make_clean_msg(cloud, ri, stamp));
        pub_cloud_->publish(make_cloud_pc2(cloud, stamp));

        const SensorHealthStatus sh = sensor_health_->update(
            stats.n_raw, stats.n_valid, static_cast<uint64_t>(cloud.no_return_raw.size()));
        if (!sh.ok && !sensor_alarm_logged_) {
            RCLCPP_ERROR(get_logger(), "СЕНСОР НЕИСПРАВЕН: no_return=%.2f valid=%.2f baseline=%.0f",
                         sh.no_return_ratio, sh.valid_ratio, sh.baseline_valid);
            sensor_alarm_logged_ = true;
        } else if (sh.ok) {
            sensor_alarm_logged_ = false;
        }

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
        d.n_ground_points = gst.n_ground;
        d.t_filter_ms = t_filter_ms;
        d.t_range_image_ms = t_range_image_ms;
        d.t_ground_ms = t_ground_ms;
        d.fps = (t_total_ms_prev_ > 0.0f) ? 1000.0f / t_total_ms_prev_ : 0.0f;
        d.frames_processed = ++frames_processed_;
        d.sensor_ok = sh.ok;
        d.no_return_ratio = sh.no_return_ratio;
        d.valid_ratio = sh.valid_ratio;
        pub_diag_->publish(d);

        t_total_ms_prev_ = ms_since(t_start);
        RCLCPP_DEBUG(get_logger(),
                     "raw=%u valid=%u zero=%u nonfinite=%u range=%u | ri %ux%u | "
                     "filter %.2f ms, ground %.2f ms (%u pts), ri %.2f ms",
                     stats.n_raw, stats.n_valid, stats.reject_zero, stats.reject_nonfinite,
                     stats.reject_out_of_range, ri.width, ri.height, t_filter_ms, t_ground_ms,
                     gst.n_ground, t_range_image_ms);
    }

    std::string input_topic_;
    FilterParams params_;
    RangeImageParams ri_params_;
    std::unique_ptr<GroundSegmenter> ground_;
    std::unique_ptr<SensorHealthMonitor> sensor_health_;
    bool sensor_alarm_logged_ = false;
    bool layout_error_logged_ = false;
    uint64_t frames_processed_ = 0;
    float t_total_ms_prev_ = 0.0f;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<CleanCloudMsg>::SharedPtr pub_clean_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<Diagnostics>::SharedPtr pub_diag_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::PreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
