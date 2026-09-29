
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <argus_msgs/msg/anomaly_set.hpp>
#include <argus_msgs/msg/clean_cloud.hpp>
#include <argus_msgs/msg/diagnostics.hpp>

#include "argus_core/anomaly.hpp"
#include "argus_core/range_image.hpp"
#include "argus_core/types.hpp"

namespace argus {

using CleanCloudMsg = argus_msgs::msg::CleanCloud;
using AnomalySetMsg = argus_msgs::msg::AnomalySet;
using Diagnostics = argus_msgs::msg::Diagnostics;

CleanCloud msg_to_cloud(const CleanCloudMsg& msg) {
    CleanCloud cloud;
    cloud.n_raw = msg.n_raw;
    cloud.rings = msg.rings;
    cloud.az_steps = msg.az_steps;
    cloud.echo_multiplier = msg.echo_multiplier;
    cloud.frame_id = msg.header.frame_id;
    cloud.stamp_s = static_cast<double>(msg.header.stamp.sec) +
                    static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    cloud.x = msg.x;
    cloud.y = msg.y;
    cloud.z = msg.z;
    cloud.intensity = msg.intensity;
    cloud.raw_idx = msg.raw_idx;
    cloud.no_return_raw = msg.no_return_raw;
    cloud.ground_mask = msg.ground_mask;
    return cloud;
}

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode() : Node("argus_detector") {
        NoReturnParams nr;
        nr.min_missing_run =
            static_cast<uint32_t>(declare_parameter<int>("detector_no_return.min_missing_run", 8));
        nr.max_missing_run_deg = static_cast<float>(
            declare_parameter<double>("detector_no_return.max_missing_run_deg", 15.0));
        nr.azimuth_window =
            static_cast<uint32_t>(declare_parameter<int>("detector_no_return.azimuth_window", 5));
        nr.min_baseline_hits = static_cast<float>(
            declare_parameter<double>("detector_no_return.min_baseline_hits", 0.70));
        nr.min_range =
            static_cast<float>(declare_parameter<double>("detector_no_return.min_range", 3.0));

        GeometryResidualParams geom;
        geom.median_half_window = static_cast<uint32_t>(
            declare_parameter<int>("detector_geometry.median_half_window", 100));
        geom.median_window_override_min_range_m = static_cast<float>(declare_parameter<double>(
            "detector_geometry.median_window_override_min_range_m", 18.5));
        geom.median_window_override_max_range_m = static_cast<float>(declare_parameter<double>(
            "detector_geometry.median_window_override_max_range_m", 40.0));
        geom.median_half_window_override = static_cast<uint32_t>(
            declare_parameter<int>("detector_geometry.median_half_window_override", 200));
        geom.residual_threshold_m = static_cast<float>(
            declare_parameter<double>("detector_geometry.residual_threshold_m", 1.0));
        geom.residual_threshold_far_m = static_cast<float>(
            declare_parameter<double>("detector_geometry.residual_threshold_far_m", 1.0));
        geom.min_cells =
            static_cast<uint32_t>(declare_parameter<int>("detector_geometry.min_cells", 50));
        geom.min_fill =
            static_cast<float>(declare_parameter<double>("detector_geometry.min_fill", 0.3));
        geom.min_range =
            static_cast<float>(declare_parameter<double>("detector_geometry.min_range", 4.0));
        geom.max_target_range_m = static_cast<float>(
            declare_parameter<double>("detector_geometry.max_target_range_m", 45.0));
        geom.far_range_m =
            static_cast<float>(declare_parameter<double>("detector_geometry.far_range_m", 60.0));
        geom.min_cells_far =
            static_cast<uint32_t>(declare_parameter<int>("detector_geometry.min_cells_far", 8));
        geom.min_fill_far =
            static_cast<float>(declare_parameter<double>("detector_geometry.min_fill_far", 0.12));
        geom.min_stable_frames =
            static_cast<uint32_t>(declare_parameter<int>("detector_geometry.min_stable_frames", 2));
        geom.az_tolerance =
            static_cast<uint32_t>(declare_parameter<int>("detector_geometry.az_tolerance", 20));
        geom.range_tolerance_m = static_cast<float>(
            declare_parameter<double>("detector_geometry.range_tolerance_m", 3.0));

        TemporalResidualParams temporal_params;
        temporal_params.window_frames = static_cast<uint32_t>(declare_parameter<int>(
            "detector_temporal.window_frames", static_cast<int>(temporal_params.window_frames)));
        temporal_params.residual_threshold_m = static_cast<float>(declare_parameter<double>(
            "detector_temporal.residual_threshold_m", temporal_params.residual_threshold_m));
        temporal_params.min_range_m = static_cast<float>(declare_parameter<double>(
            "detector_temporal.min_range_m", temporal_params.min_range_m));
        temporal_params.max_range_m = static_cast<float>(declare_parameter<double>(
            "detector_temporal.max_range_m", temporal_params.max_range_m));
        temporal_params.max_frame_gap_s = declare_parameter<double>(
            "detector_temporal.max_frame_gap_s", temporal_params.max_frame_gap_s);

        free_space_enabled_ = declare_parameter<bool>("detector_free_space.enabled", false);
        geom_enabled_ = declare_parameter<bool>("detector_geometry.enabled", true);
        nr_enabled_ = declare_parameter<bool>("detector_no_return.enabled", true);
        auto qos = rclcpp::QoS(10);
        sub_ = create_subscription<CleanCloudMsg>(
            "/argus/clean", qos, [this](CleanCloudMsg::ConstSharedPtr msg) { on_clean(msg); });
        pub_geometry_ = create_publisher<AnomalySetMsg>("/argus/anom_g", qos);
        pub_temporal_ = create_publisher<AnomalySetMsg>("/argus/anom_temporal", qos);
        pub_no_return_ = create_publisher<AnomalySetMsg>("/argus/anom_nr", qos);
        pub_diag_ = create_publisher<Diagnostics>("/argus/diagnostics_detector", qos);
        pub_heartbeat_ = create_publisher<std_msgs::msg::String>("/argus/heartbeat", qos);
        heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            std_msgs::msg::String beat;
            beat.data = get_name();
            pub_heartbeat_->publish(beat);
        });

        no_return_ = std::make_unique<NoReturnDetector>(nr);
        geometry_ = std::make_unique<GeometryResidualDetector>(geom);
        temporal_ = std::make_unique<TemporalResidualDetector>(temporal_params);

        RCLCPP_INFO(get_logger(), "argus_detector: geometry=%s no_return=%s temporal=on",
                    geom_enabled_ ? "on" : "off", nr_enabled_ ? "on" : "off");
        if (free_space_enabled_) {
            RCLCPP_WARN(get_logger(),
                        "detector_free_space.enabled=true, детектор публикует /argus/anom_fs "
                        "из argus_tunnel_model (нужна одометрия и прогретая карта)");
        }
    }

private:
    static float ms_since(std::chrono::steady_clock::time_point t0) {
        const auto dt = std::chrono::steady_clock::now() - t0;
        return static_cast<float>(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt).count());
    }

    static AnomalySetMsg to_msg(const AnomalySet& set, const rclcpp::Time& stamp,
                                const std::string& frame_id) {
        AnomalySetMsg out;
        out.header.stamp = stamp;
        out.header.frame_id = frame_id;
        out.source = set.source;
        out.indices = set.indices;
        out.scores = set.score;
        out.cells = set.cells;
        out.cells_scores = set.cells_score;
        return out;
    }

    void on_clean(CleanCloudMsg::ConstSharedPtr msg) {
        const auto t_start = std::chrono::steady_clock::now();
        const rclcpp::Time stamp = msg->header.stamp;

        const CleanCloud cloud = msg_to_cloud(*msg);
        RangeImageParams ri_params;
        ri_params.rings_fallback = msg->rings > 1 ? msg->rings : ri_params.rings_fallback;
        const RangeImage ri = build_range_image(cloud, ri_params);
        if (!ri.valid()) {
            RCLCPP_WARN(get_logger(), "кадр с пустым range image, пропущен");
            return;
        }

        float t_detect_ms = 0.0f;
        auto t0 = std::chrono::steady_clock::now();
        const AnomalySet geometry = geom_enabled_ ? geometry_->detect(cloud, ri) : AnomalySet{};
        t_detect_ms += ms_since(t0);
        t0 = std::chrono::steady_clock::now();
        const AnomalySet temporal = temporal_->detect(cloud, ri);
        t_detect_ms += ms_since(t0);
        t0 = std::chrono::steady_clock::now();
        const AnomalySet no_return = nr_enabled_ ? no_return_->detect(cloud, ri) : AnomalySet{};
        t_detect_ms += ms_since(t0);

        pub_geometry_->publish(to_msg(geometry, stamp, cloud.frame_id));
        pub_temporal_->publish(to_msg(temporal, stamp, cloud.frame_id));
        pub_no_return_->publish(to_msg(no_return, stamp, cloud.frame_id));

        Diagnostics d;
        d.header.stamp = stamp;
        d.header.frame_id = cloud.frame_id;
        d.n_points_valid = static_cast<uint32_t>(cloud.size());
        d.n_points_raw = cloud.n_raw;
        d.rings_detected = ri.height;
        d.az_steps_detected = ri.width;
        d.t_detect_ms = t_detect_ms;
        d.fps = (t_frame_ms_prev_ > 0.0f) ? 1000.0f / t_frame_ms_prev_ : 0.0f;
        d.frames_processed = ++frames_processed_;
        pub_diag_->publish(d);

        t_frame_ms_prev_ = ms_since(t_start);
        RCLCPP_DEBUG(get_logger(), "ri %ux%u | detect %.2f ms | geom=%u nr_cells=%u", ri.width,
                     ri.height, t_detect_ms, static_cast<uint32_t>(geometry.indices.size()),
                     static_cast<uint32_t>(no_return.cells.size()));
    }

    bool free_space_enabled_ = false;
    bool geom_enabled_ = true;
    bool nr_enabled_ = true;
    uint64_t frames_processed_ = 0;
    float t_frame_ms_prev_ = 0.0f;
    std::unique_ptr<GeometryResidualDetector> geometry_;
    std::unique_ptr<TemporalResidualDetector> temporal_;
    std::unique_ptr<NoReturnDetector> no_return_;
    rclcpp::Subscription<CleanCloudMsg>::SharedPtr sub_;
    rclcpp::Publisher<AnomalySetMsg>::SharedPtr pub_geometry_;
    rclcpp::Publisher<AnomalySetMsg>::SharedPtr pub_temporal_;
    rclcpp::Publisher<AnomalySetMsg>::SharedPtr pub_no_return_;
    rclcpp::Publisher<Diagnostics>::SharedPtr pub_diag_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
