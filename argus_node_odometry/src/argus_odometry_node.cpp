#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <argus_msgs/msg/clean_cloud.hpp>
#include <argus_msgs/msg/diagnostics.hpp>

#include "argus_core/odometry.hpp"
#include "argus_core/types.hpp"

namespace argus {

using CleanCloudMsg = argus_msgs::msg::CleanCloud;
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

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("argus_odometry") {
        OdometryParams p;
        p.enabled = declare_parameter<bool>("odometry.enabled", true);
        p.lock_lateral = declare_parameter<bool>("odometry.lock_lateral", true);
        p.lock_roll = declare_parameter<bool>("odometry.lock_roll", true);
        p.lock_vertical = declare_parameter<bool>("odometry.lock_vertical", false);
        p.max_range = static_cast<float>(declare_parameter<double>("odometry.max_range", 120.0));
        p.min_range = static_cast<float>(declare_parameter<double>("odometry.min_range", 1.0));
        p.voxel_size = static_cast<float>(declare_parameter<double>("odometry.voxel_size", 0.5));
        p.max_speed_mps =
            static_cast<float>(declare_parameter<double>("odometry.max_speed_mps", 30.0));
        p.max_accel_mps2 =
            static_cast<float>(declare_parameter<double>("odometry.max_accel_mps2", 2.0));
        p.max_iterations =
            static_cast<uint32_t>(declare_parameter<int>("odometry.max_iterations", 4));
        p.max_correspondence_m =
            static_cast<float>(declare_parameter<double>("odometry.max_correspondence_m", 2.0));
        p.rest_translation_m =
            static_cast<float>(declare_parameter<double>("odometry.rest_translation_m", 0.20));
        p.keep_ground = declare_parameter<bool>("odometry.keep_ground", true);
        p.max_fitness = static_cast<float>(declare_parameter<double>("odometry.max_fitness", 0.05));
        p.min_correspondences_quality = static_cast<uint32_t>(
            declare_parameter<int>("odometry.min_correspondences_quality", 100));
        const std::string axis_name = declare_parameter<std::string>("odometry.forward_axis", "-y");
        if (!parse_forward_axis(axis_name, p.forward_axis)) {
            p.forward_axis = ForwardAxis::NegY;
        }
        odom_ = std::make_unique<TunnelOdometry>(p);

        auto qos = rclcpp::QoS(10);
        sub_ = create_subscription<CleanCloudMsg>(
            "/argus/clean", qos, [this](CleanCloudMsg::ConstSharedPtr msg) { on_clean(msg); });
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/argus/pose", qos);
        pub_diag_ = create_publisher<Diagnostics>("/argus/diagnostics_odometry", qos);
        pub_heartbeat_ = create_publisher<std_msgs::msg::String>("/argus/heartbeat", qos);
        heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            std_msgs::msg::String beat;
            beat.data = get_name();
            pub_heartbeat_->publish(beat);
        });

        RCLCPP_INFO(get_logger(), "argus_odometry: 4DoF ICP, ось '%s'", axis_name.c_str());
    }

private:
    static float ms_since(std::chrono::steady_clock::time_point t0) {
        const auto dt = std::chrono::steady_clock::now() - t0;
        return static_cast<float>(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt).count());
    }

    void on_clean(CleanCloudMsg::ConstSharedPtr msg) {
        const auto t0 = std::chrono::steady_clock::now();
        const CleanCloud cloud = msg_to_cloud(*msg);
        const OdometryResult r = odom_->update(cloud, cloud.stamp_s);
        const float t_ms = ms_since(t0);

        if (r.valid && r.quality_ok) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = msg->header.stamp;
            pose.header.frame_id = "odom";
            const Eigen::Vector3d t = r.pose.translation();
            pose.pose.position.x = t.x();
            pose.pose.position.y = t.y();
            pose.pose.position.z = t.z();
            const Eigen::Quaterniond q(r.pose.rotation());
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            pub_pose_->publish(pose);
        }

        Diagnostics d;
        d.header = msg->header;
        d.t_odometry_ms = t_ms;
        d.model_ready = r.valid && r.quality_ok;
        d.frames_processed = r.frames;
        d.n_points_valid = r.n_downsampled;
        d.n_points_rejected = r.valid ? 0u : 1u;
        d.odom_fitness = r.fitness;
        d.odom_correspondences = r.n_correspondences;
        d.odom_quality_ok = r.quality_ok;
        d.fps = (t_prev_ms_ > 0.0f) ? 1000.0f / t_prev_ms_ : 0.0f;
        pub_diag_->publish(d);
        t_prev_ms_ = t_ms;
    }

    std::unique_ptr<TunnelOdometry> odom_;
    float t_prev_ms_ = 0.0f;
    rclcpp::Subscription<CleanCloudMsg>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<Diagnostics>::SharedPtr pub_diag_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
