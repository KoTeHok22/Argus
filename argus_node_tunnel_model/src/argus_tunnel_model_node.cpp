
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include <argus_msgs/msg/anomaly_set.hpp>
#include <argus_msgs/msg/clean_cloud.hpp>
#include <argus_msgs/msg/diagnostics.hpp>

#include "argus_core/anomaly.hpp"
#include "argus_core/gauge.hpp"
#include "argus_core/qos.hpp"
#include "argus_core/range_image.hpp"
#include "argus_core/tunnel_model.hpp"
#include "argus_core/tunnel_profile.hpp"
#include "argus_core/types.hpp"

namespace argus {

using CleanCloudMsg = argus_msgs::msg::CleanCloud;
using AnomalySetMsg = argus_msgs::msg::AnomalySet;
using Diagnostics = argus_msgs::msg::Diagnostics;
using PoseStamped = geometry_msgs::msg::PoseStamped;

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

class TunnelModelNode : public rclcpp::Node {
public:
    struct FrameSlot {
        CleanCloudMsg::ConstSharedPtr cloud;
        AnomalySetMsg::ConstSharedPtr geometry;
        AnomalySetMsg::ConstSharedPtr no_return;
    };

    struct PoseSlot {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        bool valid = false;
    };

    TunnelModelNode() : Node("argus_tunnel_model") {
        read_params();
        model_ = std::make_unique<TunnelModel>(model_params_);
        profile_ = std::make_unique<TunnelProfile>(profile_params_);
        free_space_ = std::make_unique<FreeSpaceDetector>(model_.get(), free_space_params_);

        if (!prior_map_path_.empty()) {
            if (model_->load(prior_map_path_)) {
                RCLCPP_INFO(get_logger(), "prior map загружена: %s (вокселей %zu)",
                            prior_map_path_.c_str(), model_->voxel_count());
            } else {
                RCLCPP_ERROR(get_logger(), "prior map не загружена: %s — модель стартует с нуля",
                             prior_map_path_.c_str());
            }
        }

        auto qos = rclcpp::QoS(10);
        auto stream_qos = sensor_qos();
        auto control_qos_ = control_qos();
        sub_clean_ = create_subscription<CleanCloudMsg>(
            "/argus/clean", stream_qos, [this](CleanCloudMsg::ConstSharedPtr msg) {
                slots_[stamp_ns(msg->header.stamp)].cloud = msg;
            });
        sub_geometry_ = create_subscription<AnomalySetMsg>(
            "/argus/anom_g", stream_qos, [this](AnomalySetMsg::ConstSharedPtr msg) {
                slots_[stamp_ns(msg->header.stamp)].geometry = msg;
            });
        sub_no_return_ = create_subscription<AnomalySetMsg>(
            "/argus/anom_nr", stream_qos, [this](AnomalySetMsg::ConstSharedPtr msg) {
                slots_[stamp_ns(msg->header.stamp)].no_return = msg;
            });
        sub_pose_ = create_subscription<PoseStamped>(
            "/argus/pose", stream_qos, [this](PoseStamped::ConstSharedPtr msg) {
                PoseSlot& slot = poses_[stamp_ns(msg->header.stamp)];
                slot.pose = to_pose(*msg);
                slot.valid = true;
            });

        pub_free_space_ = create_publisher<AnomalySetMsg>("/argus/anom_fs", stream_qos);
        pub_diag_ = create_publisher<Diagnostics>("/argus/diagnostics_model", control_qos_);
        pub_heartbeat_ = create_publisher<std_msgs::msg::String>("/argus/heartbeat", control_qos_);
        heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            std_msgs::msg::String beat;
            beat.data = get_name();
            pub_heartbeat_->publish(beat);
        });
        if (publish_model_points_) {
            pub_model_ =
                create_publisher<sensor_msgs::msg::PointCloud2>("/argus/model", stream_qos);
        }

        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(process_period_s_));
        timer_ = create_wall_timer(period, [this] { process_ready_frames(); });

        RCLCPP_INFO(get_logger(),
                    "argus_tunnel_model: воксель %.2f м, min_observations %u, прогрев %u кадров, "
                    "устаревание %.1f с",
                    model_params_.voxel_size, model_params_.min_observations,
                    free_space_warmup_frames_, pose_timeout_s_);
    }

private:
    static uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<uint64_t>(stamp.sec) * 1000000000ULL +
               static_cast<uint64_t>(stamp.nanosec);
    }

    static Eigen::Isometry3d to_pose(const PoseStamped& msg) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        const Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x,
                                   msg.pose.orientation.y, msg.pose.orientation.z);
        pose.rotate(q.normalized());
        pose.pretranslate(
            Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
        return pose;
    }

    static float ms_since(std::chrono::steady_clock::time_point t0) {
        const auto dt = std::chrono::steady_clock::now() - t0;
        return static_cast<float>(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt).count());
    }

    void read_params() {
        model_params_.enabled = declare_parameter<bool>("tunnel_model.enabled", true);
        model_params_.voxel_size =
            static_cast<float>(declare_parameter<double>("tunnel_model.voxel_size", 0.20));
        model_params_.free_space_margin =
            static_cast<float>(declare_parameter<double>("tunnel_model.free_space_margin", 0.15));
        model_params_.min_observations =
            static_cast<uint32_t>(declare_parameter<int>("tunnel_model.min_observations", 3));
        model_params_.free_hits_to_clear =
            static_cast<uint32_t>(declare_parameter<int>("tunnel_model.free_hits_to_clear", 2));
        model_params_.max_range =
            static_cast<float>(declare_parameter<double>("tunnel_model.max_range", 150.0));
        model_params_.carve_max_range =
            static_cast<float>(declare_parameter<double>("tunnel_model.carve_max_range", 0.0));
        model_params_.carve_half_width_m =
            static_cast<float>(declare_parameter<double>("tunnel_model.carve_half_width_m", 0.0));
        model_params_.carve_stride_az = static_cast<uint32_t>(
            std::max<int>(1, declare_parameter<int>("tunnel_model.carve_stride_az", 1)));
        model_params_.carve_stride_ring = static_cast<uint32_t>(
            std::max<int>(1, declare_parameter<int>("tunnel_model.carve_stride_ring", 1)));
        model_params_.enable_decay = declare_parameter<bool>("tunnel_model.enable_decay", false);
        model_params_.decay_after_frames =
            static_cast<uint32_t>(declare_parameter<int>("tunnel_model.decay_after_frames", 3000));
        model_params_.forget_ray_fraction =
            static_cast<float>(declare_parameter<double>("tunnel_model.forget_ray_fraction", 0.02));

        profile_params_.enabled = declare_parameter<bool>("tunnel_profile.enabled", true);
        profile_params_.alpha_initial =
            static_cast<float>(declare_parameter<double>("tunnel_profile.alpha_initial", 0.30));
        profile_params_.alpha_steady =
            static_cast<float>(declare_parameter<double>("tunnel_profile.alpha_steady", 0.02));
        profile_params_.warmup_frames =
            static_cast<uint32_t>(declare_parameter<int>("tunnel_profile.warmup_frames", 200));

        free_space_params_.enabled = declare_parameter<bool>("detector_free_space.enabled", true);
        free_space_params_.min_confidence = static_cast<float>(
            declare_parameter<double>("detector_free_space.min_confidence", 0.85));
        free_space_params_.min_range =
            static_cast<float>(declare_parameter<double>("detector_free_space.min_range", 3.0));
        free_space_params_.max_range =
            static_cast<float>(declare_parameter<double>("detector_free_space.max_range", 90.0));
        free_space_params_.min_free_observations = static_cast<uint32_t>(
            declare_parameter<int>("detector_free_space.min_free_observations", 10));

        free_space_warmup_frames_ =
            static_cast<uint32_t>(declare_parameter<int>("detector_free_space.warmup_frames", 60));
        prior_map_path_ = declare_parameter<std::string>("tunnel_model.prior_map_path", "");
        process_period_s_ = declare_parameter<double>("tunnel_model.process_period_s", 0.05);
        pose_timeout_s_ = declare_parameter<double>("tunnel_model.pose_timeout_s", 1.0);
        publish_model_points_ = declare_parameter<bool>("tunnel_model.publish_points", true);
        model_publish_divisor_ = static_cast<uint32_t>(
            std::max<int>(1, declare_parameter<int>("tunnel_model.publish_divisor", 20)));
        model_point_max_range_ =
            static_cast<float>(declare_parameter<double>("tunnel_model.publish_max_range_m", 60.0));
        gauge_params_.forward_axis = ForwardAxis::NegY;
        gauge_params_.half_width = static_cast<float>(
            declare_parameter<double>("gauge.half_width", gauge_params_.half_width));
        gauge_params_.height =
            static_cast<float>(declare_parameter<double>("gauge.height", gauge_params_.height));
        gauge_params_.base_offset = static_cast<float>(
            declare_parameter<double>("gauge.base_offset", gauge_params_.base_offset));
        gauge_params_.chamfer =
            static_cast<float>(declare_parameter<double>("gauge.chamfer", gauge_params_.chamfer));
        gauge_params_.nose_offset = static_cast<float>(
            declare_parameter<double>("gauge.nose_offset", gauge_params_.nose_offset));
        gauge_params_.sensor_height = static_cast<float>(
            declare_parameter<double>("gauge.sensor_height", gauge_params_.sensor_height));
        std::string axis_name = declare_parameter<std::string>("gauge.forward_axis", "-y");
        ForwardAxis axis;
        if (!parse_forward_axis(axis_name, axis)) {
            throw std::invalid_argument("gauge.forward_axis: неизвестная ось '" + axis_name + "'");
        }
        gauge_params_.forward_axis = axis;
        if (process_period_s_ <= 0.0) {
            throw std::invalid_argument("tunnel_model.process_period_s must be > 0");
        }
        if (pose_timeout_s_ <= 0.0) {
            throw std::invalid_argument("tunnel_model.pose_timeout_s must be > 0");
        }
    }

    void process_ready_frames() {
        while (!slots_.empty()) {
            const uint64_t key = slots_.begin()->first;
            const auto pose_it = poses_.find(key);
            if (pose_it == poses_.end()) {
                if (static_cast<double>(now().nanoseconds()) * 1e-9 -
                        static_cast<double>(key) * 1e-9 >
                    pose_timeout_s_) {
                    slots_.erase(slots_.begin());
                }
                break;
            }

            FrameSlot slot = slots_.begin()->second;
            if (!slot.cloud) {
                slots_.erase(slots_.begin());
                continue;
            }
            process(slot, pose_it->second.pose);
            slots_.erase(slots_.begin());
            poses_.erase(poses_.begin(), poses_.upper_bound(key));
        }
        if (slots_.size() > max_pending_) {
            slots_.erase(
                slots_.begin(),
                std::next(slots_.begin(), static_cast<long>(slots_.size() - max_pending_)));
        }
    }

    void process(FrameSlot& slot, const Eigen::Isometry3d& pose) {
        const auto t0 = std::chrono::steady_clock::now();
        const CleanCloud cloud = msg_to_cloud(*slot.cloud);
        RangeImageParams ri_params;
        ri_params.rings_fallback =
            slot.cloud->rings > 1 ? slot.cloud->rings : ri_params.rings_fallback;
        const RangeImage ri = build_range_image(cloud, ri_params);
        if (!ri.valid()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "кадр с пустым range image, модель не обновлена");
            return;
        }

        model_->integrate(cloud, pose, slot.geometry == nullptr || slot.geometry->indices.empty());
        profile_->update(ri, slot.geometry == nullptr || slot.geometry->indices.empty());

        const AnomalySet free_space = free_space_->detect(cloud, ri, pose);
        AnomalySetMsg fs_msg;
        fs_msg.header = slot.cloud->header;
        fs_msg.source = free_space.source;
        fs_msg.indices = free_space.indices;
        fs_msg.scores = free_space.score;
        pub_free_space_->publish(fs_msg);

        const TunnelModelStats model_stats = model_->stats();
        const FreeSpaceCheck check = model_->check(cloud, pose, free_space_params_.min_confidence);
        const float processing_ms = ms_since(t0);

        Diagnostics d;
        d.header = slot.cloud->header;
        d.model_voxel_count = model_stats.n_voxels;
        d.model_ready = model_stats.ready;
        d.profile_median_deviation_m = profile_->median_deviation(ri);
        d.t_model_ms = processing_ms;
        d.n_points_valid = static_cast<uint32_t>(free_space.indices.size());
        d.n_points_rejected = check.observed;
        d.frames_processed = ++frames_processed_;
        d.fps = (t_prev_ms_ > 0.0f) ? 1000.0f / t_prev_ms_ : 0.0f;
        pub_diag_->publish(d);

        if (pub_model_ && frames_processed_ % model_publish_divisor_ == 0) {
            publish_model(slot.cloud->header, pose);
        }

        t_prev_ms_ = processing_ms;
        RCLCPP_DEBUG(get_logger(),
                     "вокселей %u, пусто %.1f%%, проверено %u, пустых %u, free_space точек %u, "
                     "%.2f мс",
                     model_stats.n_voxels, static_cast<double>(model_stats.free_ratio) * 100.0,
                     check.observed, check.empty, static_cast<uint32_t>(free_space.indices.size()),
                     static_cast<double>(processing_ms));
    }

    void publish_model(const std_msgs::msg::Header& header, const Eigen::Isometry3d& pose) {
        std::vector<Eigen::Vector3f> points;
        std::vector<float> confidence;
        model_->export_occupancy(points, confidence, true);

        std::vector<Eigen::Vector3f> gauge_vertices;
        std::vector<uint32_t> gauge_indices;
        ClearanceGauge(gauge_params_)
            .to_mesh(model_point_max_range_, gauge_vertices, gauge_indices);

        const Eigen::Isometry3d inv = pose.inverse();
        const size_t total = points.size() + gauge_indices.size();
        if (total == 0) {
            return;
        }

        sensor_msgs::msg::PointCloud2 msg;
        msg.header = header;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(total);
        msg.is_bigendian = false;
        msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(total);

        sensor_msgs::PointCloud2Iterator<float> out_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> out_i(msg, "intensity");

        for (const Eigen::Vector3f& p : points) {
            const Eigen::Vector3f world = (inv * p.cast<double>()).cast<float>();
            *out_x = world.x();
            *out_y = world.y();
            *out_z = world.z();
            *out_i = 0.35f;
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_i;
        }
        for (uint32_t index : gauge_indices) {
            const Eigen::Vector3f world =
                (inv * gauge_vertices[index].cast<double>()).cast<float>();
            *out_x = world.x();
            *out_y = world.y();
            *out_z = world.z();
            *out_i = 1.0f;
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_i;
        }
        pub_model_->publish(msg);
    }

    TunnelModelParams model_params_;
    TunnelProfileParams profile_params_;
    FreeSpaceParams free_space_params_;
    uint32_t free_space_warmup_frames_ = 60;
    std::string prior_map_path_;
    double process_period_s_ = 0.05;
    double pose_timeout_s_ = 1.0;
    bool publish_model_points_ = true;
    uint32_t model_publish_divisor_ = 20;
    float model_point_max_range_ = 60.0f;
    ClearanceGaugeParams gauge_params_;
    size_t max_pending_ = 16;
    uint64_t frames_processed_ = 0;
    float t_prev_ms_ = 0.0f;

    std::unique_ptr<TunnelModel> model_;
    std::unique_ptr<TunnelProfile> profile_;
    std::unique_ptr<FreeSpaceDetector> free_space_;
    std::map<uint64_t, FrameSlot> slots_;
    std::map<uint64_t, PoseSlot> poses_;

    rclcpp::Subscription<CleanCloudMsg>::SharedPtr sub_clean_;
    rclcpp::Subscription<AnomalySetMsg>::SharedPtr sub_geometry_;
    rclcpp::Subscription<AnomalySetMsg>::SharedPtr sub_no_return_;
    rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<AnomalySetMsg>::SharedPtr pub_free_space_;
    rclcpp::Publisher<Diagnostics>::SharedPtr pub_diag_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_model_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::TunnelModelNode>());
    rclcpp::shutdown();
    return 0;
}
