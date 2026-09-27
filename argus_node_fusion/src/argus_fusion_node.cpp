
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <argus_msgs/msg/anomaly_set.hpp>
#include <argus_msgs/msg/candidate.hpp>
#include <argus_msgs/msg/clean_cloud.hpp>
#include <argus_msgs/msg/obstacle.hpp>
#include <argus_msgs/msg/obstacle_array.hpp>

#include "argus_core/explain.hpp"
#include "argus_core/fusion.hpp"
#include "argus_core/gauge.hpp"
#include "argus_core/range_image.hpp"

namespace argus {

using CleanCloudMsg = argus_msgs::msg::CleanCloud;
using AnomalySetMsg = argus_msgs::msg::AnomalySet;
using Obstacle = argus_msgs::msg::Obstacle;
using ObstacleArray = argus_msgs::msg::ObstacleArray;
using Candidate = argus_msgs::msg::Candidate;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

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

AnomalySet msg_to_set(const AnomalySetMsg& msg) {
    AnomalySet set;
    set.source = msg.source;
    set.indices = msg.indices;
    set.score = msg.scores;
    set.cells = msg.cells;
    set.cells_score = msg.cells_scores;
    return set;
}

uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("argus_fusion") {
        gauge_params_ = read_gauge_params();
        params_ = read_fusion_params();

        default_dt_ = declare_parameter<double>("fusion.default_dt_s", 0.1);
        stale_timeout_ = declare_parameter<double>("fusion.stale_timeout_s", 1.0);
        max_pending_ = static_cast<size_t>(
            declare_parameter<int>("fusion.max_pending_frames", static_cast<int>(max_pending_)));
        if (default_dt_ <= 0.0) {
            throw std::invalid_argument("fusion.default_dt_s must be > 0");
        }
        if (stale_timeout_ <= 0.0) {
            throw std::invalid_argument("fusion.stale_timeout_s must be > 0");
        }
        if (max_pending_ == 0) {
            throw std::invalid_argument("fusion.max_pending_frames must be > 0");
        }
        train_speed_ = declare_parameter<double>("vehicle.default_speed_mps", 0.0);
        publish_markers_ = declare_parameter<bool>("fusion.publish_markers", true);
        marker_max_range_ =
            static_cast<float>(declare_parameter<double>("fusion.marker_max_range_m", 60.0));
        marker_max_obstacles_ = static_cast<size_t>(
            std::max<int>(0, declare_parameter<int>("fusion.marker_max_obstacles", 16)));
        if (marker_max_range_ <= 0.0f) {
            throw std::invalid_argument("fusion.marker_max_range_m must be > 0");
        }

        pipeline_ = std::make_unique<FusionPipeline>(ClearanceGauge(gauge_params_), params_);

        auto qos = rclcpp::QoS(10);
        sub_clean_ = create_subscription<CleanCloudMsg>(
            "/argus/clean", qos, [this](CleanCloudMsg::ConstSharedPtr msg) {
                const uint64_t key = stamp_ns(msg->header.stamp);
                slots_[key].cloud = msg;
                try_process(key);
            });
        sub_geometry_ = create_subscription<AnomalySetMsg>(
            "/argus/anom_g", qos, [this](AnomalySetMsg::ConstSharedPtr msg) {
                const uint64_t key = stamp_ns(msg->header.stamp);
                slots_[key].geometry = msg;
                try_process(key);
            });
        sub_no_return_ = create_subscription<AnomalySetMsg>(
            "/argus/anom_nr", qos, [this](AnomalySetMsg::ConstSharedPtr msg) {
                const uint64_t key = stamp_ns(msg->header.stamp);
                slots_[key].no_return = msg;
                try_process(key);
            });
        sub_free_space_ = create_subscription<AnomalySetMsg>(
            "/argus/anom_fs", qos, [this](AnomalySetMsg::ConstSharedPtr msg) {
                const uint64_t key = stamp_ns(msg->header.stamp);
                slots_[key].free_space = msg;
            });
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/argus/pose", qos, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                poses_[stamp_ns(msg->header.stamp)] = to_pose(*msg);
            });
        pub_obstacles_ = create_publisher<ObstacleArray>("/argus/obstacles", qos);
        pub_explain_ = create_publisher<std_msgs::msg::String>("/argus/explain", qos);
        if (publish_markers_) {
            pub_markers_ = create_publisher<MarkerArray>("/argus/markers", qos);
        }

        const auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(stale_timeout_));
        stale_timer_ = create_wall_timer(timer_period, [this] { check_stale(); });

        RCLCPP_INFO(get_logger(),
                    "argus_fusion: ось габарита '%s', sensor_height %.2f м, "
                    "min_votes=%u, подтверждение с %u кадров",
                    forward_axis_name_.c_str(), gauge_params_.sensor_height,
                    params_.min_votes_for_alert, params_.tracking.min_hits_to_confirm);
    }

private:
    static float ms_since(std::chrono::steady_clock::time_point t0) {
        const auto dt = std::chrono::steady_clock::now() - t0;
        return static_cast<float>(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(dt).count());
    }

    ClearanceGaugeParams read_gauge_params() {
        ClearanceGaugeParams p;
        forward_axis_name_ = declare_parameter<std::string>("gauge.forward_axis", "x");
        ForwardAxis axis;
        if (!parse_forward_axis(forward_axis_name_, axis)) {
            throw std::invalid_argument("gauge.forward_axis: неизвестная ось '" +
                                        forward_axis_name_ + "'");
        }
        p.forward_axis = axis;
        p.half_width =
            static_cast<float>(declare_parameter<double>("gauge.half_width", p.half_width));
        p.height = static_cast<float>(declare_parameter<double>("gauge.height", p.height));
        p.base_offset =
            static_cast<float>(declare_parameter<double>("gauge.base_offset", p.base_offset));
        p.chamfer = static_cast<float>(declare_parameter<double>("gauge.chamfer", p.chamfer));
        p.nose_offset =
            static_cast<float>(declare_parameter<double>("gauge.nose_offset", p.nose_offset));
        p.sensor_height =
            static_cast<float>(declare_parameter<double>("gauge.sensor_height", p.sensor_height));
        p.max_range = static_cast<float>(declare_parameter<double>("gauge.max_range", p.max_range));
        p.safety_margin =
            static_cast<float>(declare_parameter<double>("gauge.safety_margin", p.safety_margin));
        return p;
    }

    FusionParams read_fusion_params() {
        FusionParams p;
        p.require_confirmed_track =
            declare_parameter<bool>("fusion.require_confirmed_track", p.require_confirmed_track);
        p.critical_range_m = static_cast<float>(
            declare_parameter<double>("fusion.critical_range_m", p.critical_range_m));
        p.warning_range_m = static_cast<float>(
            declare_parameter<double>("fusion.warning_range_m", p.warning_range_m));
        p.critical_ttc_s = static_cast<float>(
            declare_parameter<double>("fusion.critical_ttc_s", p.critical_ttc_s));
        p.min_votes_for_alert = static_cast<uint32_t>(declare_parameter<int>(
            "fusion.min_votes_for_alert", static_cast<int>(p.min_votes_for_alert)));
        p.max_confirmed_track_misses = static_cast<uint32_t>(declare_parameter<int>(
            "fusion.max_confirmed_track_misses", static_cast<int>(p.max_confirmed_track_misses)));
        p.strict_track_freshness_range_m = static_cast<float>(declare_parameter<double>(
            "fusion.strict_track_freshness_range_m", p.strict_track_freshness_range_m));
        p.ground_filter = declare_parameter<bool>("fusion.ground_filter", p.ground_filter);
        p.ground_clearance_m = static_cast<float>(
            declare_parameter<double>("fusion.ground_clearance_m", p.ground_clearance_m));
        p.rail_zone_m =
            static_cast<float>(declare_parameter<double>("fusion.rail_zone_m", p.rail_zone_m));

        ClusteringParams& c = p.clustering;
        c.neighbor_distance = static_cast<float>(
            declare_parameter<double>("clustering.neighbor_distance", c.neighbor_distance));
        c.adaptive_scaling =
            declare_parameter<bool>("clustering.adaptive_scaling", c.adaptive_scaling);
        c.reference_range = static_cast<float>(
            declare_parameter<double>("clustering.reference_range", c.reference_range));
        c.min_cluster_size_near = static_cast<uint32_t>(declare_parameter<int>(
            "clustering.min_cluster_size_near", static_cast<int>(c.min_cluster_size_near)));
        c.min_cluster_size_mid = static_cast<uint32_t>(declare_parameter<int>(
            "clustering.min_cluster_size_mid", static_cast<int>(c.min_cluster_size_mid)));
        c.min_cluster_size_far = static_cast<uint32_t>(declare_parameter<int>(
            "clustering.min_cluster_size_far", static_cast<int>(c.min_cluster_size_far)));
        c.size_near_range_m = static_cast<float>(
            declare_parameter<double>("clustering.size_near_range_m", c.size_near_range_m));
        c.size_mid_range_m = static_cast<float>(
            declare_parameter<double>("clustering.size_mid_range_m", c.size_mid_range_m));
        c.size_far_range_m = static_cast<float>(
            declare_parameter<double>("clustering.size_far_range_m", c.size_far_range_m));
        c.min_cluster_size_long = static_cast<uint32_t>(declare_parameter<int>(
            "clustering.min_cluster_size_long", static_cast<int>(c.min_cluster_size_long)));
        c.size_long_range_m = static_cast<float>(
            declare_parameter<double>("clustering.size_long_range_m", c.size_long_range_m));
        c.max_cluster_size = static_cast<uint32_t>(declare_parameter<int>(
            "clustering.max_cluster_size", static_cast<int>(c.max_cluster_size)));
        c.min_extent_m = static_cast<float>(
            declare_parameter<double>("clustering.min_extent_m", c.min_extent_m));
        c.max_extent_m = static_cast<float>(
            declare_parameter<double>("clustering.max_extent_m", c.max_extent_m));

        TrackingParams& t = p.tracking;
        t.max_association_distance = static_cast<float>(declare_parameter<double>(
            "tracking.max_association_distance", t.max_association_distance));
        t.min_hits_to_confirm = static_cast<uint32_t>(declare_parameter<int>(
            "tracking.min_hits_to_confirm", static_cast<int>(t.min_hits_to_confirm)));
        t.min_hits_to_confirm_far = static_cast<uint32_t>(declare_parameter<int>(
            "tracking.min_hits_to_confirm_far", static_cast<int>(t.min_hits_to_confirm_far)));
        t.confirm_near_range_m = static_cast<float>(
            declare_parameter<double>("tracking.confirm_near_range_m", t.confirm_near_range_m));
        t.confirm_far_range_m = static_cast<float>(
            declare_parameter<double>("tracking.confirm_far_range_m", t.confirm_far_range_m));
        t.max_misses_to_keep = static_cast<uint32_t>(declare_parameter<int>(
            "tracking.max_misses_to_keep", static_cast<int>(t.max_misses_to_keep)));
        t.process_noise = static_cast<float>(
            declare_parameter<double>("tracking.process_noise", t.process_noise));
        t.measurement_noise = static_cast<float>(
            declare_parameter<double>("tracking.measurement_noise", t.measurement_noise));
        return p;
    }

    void try_process(uint64_t key) {
        const auto it = slots_.find(key);
        if (it == slots_.end()) {
            return;
        }
        const FrameSlot& slot = it->second;
        if (!slot.cloud || !slot.geometry || !slot.no_return) {
            return;
        }

        const CleanCloud cloud = msg_to_cloud(*slot.cloud);
        RangeImageParams ri_params;
        ri_params.rings_fallback =
            slot.cloud->rings > 1 ? slot.cloud->rings : ri_params.rings_fallback;
        const RangeImage ri = build_range_image(cloud, ri_params);
        if (!ri.valid()) {
            RCLCPP_WARN(get_logger(), "кадр %lu: пустой range image, пропущен",
                        static_cast<unsigned long>(key));
            slots_.erase(slots_.begin(), slots_.upper_bound(key));
            return;
        }

        float dt = static_cast<float>(default_dt_);
        if (last_stamp_ns_ > 0 && key > last_stamp_ns_) {
            const double d = static_cast<double>(key - last_stamp_ns_) * 1e-9;
            if (d < 1.0) {
                dt = static_cast<float>(d);
                fps_ = (fps_ > 0.0f) ? 0.9f * fps_ + 0.1f / dt : 1.0f / dt;
            }
        }
        last_stamp_ns_ = key;

        const auto t0 = std::chrono::steady_clock::now();
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        const auto pose_it = poses_.find(key);
        const bool pose_valid = pose_it != poses_.end();
        if (pose_valid) {
            pose = pose_it->second;
        }
        AnomalySet free_space;
        if (pose_valid && slot.free_space) {
            free_space = msg_to_set(*slot.free_space);
        }
        const FusionResult result = pipeline_->update(
            cloud, ri, msg_to_set(*slot.geometry), msg_to_set(*slot.no_return), free_space, dt,
            static_cast<float>(train_speed_), pose, true, pose_valid);
        const float processing_ms = ms_since(t0);
        ++frames_processed_;

        publish_result(result, *slot.cloud, processing_ms);

        slots_.erase(slots_.begin(), slots_.upper_bound(key));
        poses_.erase(poses_.begin(), poses_.upper_bound(key));
        while (slots_.size() > max_pending_) {
            slots_.erase(slots_.begin());
        }
        last_output_ = std::chrono::steady_clock::now();
    }

    void publish_result(const FusionResult& result, const CleanCloudMsg& clean,
                        float processing_ms) {
        ObstacleArray out;
        out.header.stamp = clean.header.stamp;
        out.header.frame_id = clean.header.frame_id;

        uint8_t status = ObstacleArray::STATUS_CLEAR;
        for (const Track& t : result.tracks) {
            const Obstacle o = to_obstacle(t);
            if (o.severity == Obstacle::SEVERITY_CRITICAL) {
                status = ObstacleArray::STATUS_BLOCKED;
            } else if (o.severity == Obstacle::SEVERITY_WARNING &&
                       status != ObstacleArray::STATUS_BLOCKED) {
                status = ObstacleArray::STATUS_WARNING;
            }
            out.obstacles.push_back(o);
        }
        for (size_t i = 0; i < result.candidates.size(); ++i) {
            const Cluster& c = result.candidates[i];
            Candidate candidate;
            candidate.header = out.header;
            candidate.candidate_id = static_cast<uint32_t>(i);
            candidate.position.x = c.centroid.x();
            candidate.position.y = c.centroid.y();
            candidate.position.z = c.centroid.z();
            candidate.extent.x = (c.max_corner - c.min_corner).x();
            candidate.extent.y = (c.max_corner - c.min_corner).y();
            candidate.extent.z = (c.max_corner - c.min_corner).z();
            candidate.range_m = c.nearest_range;
            candidate.confidence = c.score;
            candidate.votes_free_space = c.votes_free_space;
            candidate.votes_no_return = c.votes_no_return;
            candidate.votes_geometry = c.votes_geometry;
            candidate.point_count = c.point_count;
            candidate.reason = reason_of(c);
            out.candidates.push_back(candidate);
        }
        if (!result.pose_used && status == ObstacleArray::STATUS_CLEAR) {
            status = ObstacleArray::STATUS_DEGRADED;
        }
        out.status = status;
        out.nearest_range_m = result.alert ? result.nearest_range_m : -1.0f;
        out.obstacles_detected = static_cast<uint32_t>(out.obstacles.size());
        out.processing_ms = processing_ms;
        out.fps = fps_;
        out.model_ready = false;
        out.explain = explain_text(result, out, clean.header.frame_id);
        pub_obstacles_->publish(out);

        publish_explain(out);
        if (pub_markers_) {
            publish_markers(out, result);
        }

        if (result.alert) {
            RCLCPP_INFO(get_logger(),
                        "ТРЕВОГА: объектов %u, впереди %.1f м, дистанция %.1f м, TTC %.1f с",
                        out.obstacles_detected, result.nearest_forward_m, result.nearest_range_m,
                        result.ttc_s);
        }
    }

    std::string explain_text(const FusionResult& result, const ObstacleArray& out,
                             const std::string& frame_id) const {
        std::string text;
        text += "frame " + frame_id + "\n";
        if (!result.tracks.empty()) {
            const Track& t = result.tracks.front();
            AlertReason reason;
            reason.votes_free_space = t.votes_free_space;
            reason.votes_no_return = t.votes_no_return;
            reason.votes_geometry = t.votes_geometry;
            reason.total_votes =
                static_cast<uint8_t>(t.votes_free_space + t.votes_no_return + t.votes_geometry);
            text += format_alert(t, reason);
        } else if (out.status == ObstacleArray::STATUS_DEGRADED) {
            text += "DEGRADED: нет валидной позы одометрии, свободное пространство не проверено\n";
        } else {
            text += format_clear(frames_processed_, fps_);
            text += "\n";
        }
        char tail[160];
        std::snprintf(
            tail, sizeof(tail), "status=%s obstacles=%u nearest=%.1f m processing=%.1f ms\n",
            status_name(out.status).c_str(), out.obstacles_detected,
            static_cast<double>(out.nearest_range_m), static_cast<double>(out.processing_ms));
        text += tail;
        return text;
    }

    static std::string status_name(uint8_t status) {
        switch (status) {
            case ObstacleArray::STATUS_BLOCKED:
                return "BLOCKED";
            case ObstacleArray::STATUS_WARNING:
                return "WARNING";
            case ObstacleArray::STATUS_DEGRADED:
                return "DEGRADED";
            default:
                return "CLEAR";
        }
    }

    void publish_explain(const ObstacleArray& out) {
        std_msgs::msg::String msg;
        msg.data = out.explain;
        pub_explain_->publish(msg);
    }

    void publish_markers(const ObstacleArray& out, const FusionResult& result) {
        MarkerArray markers;

        Marker gauge;
        gauge.header = out.header;
        gauge.ns = "gauge";
        gauge.id = 0;
        gauge.type = Marker::LINE_LIST;
        gauge.action = Marker::ADD;
        gauge.pose.orientation.w = 1.0;
        gauge.scale.x = 0.06;
        gauge.color.r = 0.25f;
        gauge.color.g = 0.80f;
        gauge.color.b = 1.0f;
        gauge.color.a = 0.9f;
        gauge.frame_locked = true;
        std::vector<Eigen::Vector3f> vertices;
        std::vector<uint32_t> indices;
        pipeline_->gauge().to_mesh(marker_max_range_, vertices, indices);
        if (indices.empty() && vertices.size() >= 12) {
            const size_t n_ring = vertices.size() / 2;
            for (size_t i = 0; i < n_ring; ++i) {
                const size_t j = (i + 1) % n_ring;
                append_point(gauge, vertices[i]);
                append_point(gauge, vertices[j]);
                append_point(gauge, vertices[i + n_ring]);
                append_point(gauge, vertices[j + n_ring]);
                append_point(gauge, vertices[i]);
                append_point(gauge, vertices[i + n_ring]);
            }
        } else {
            for (size_t i = 0; i + 1 < indices.size(); i += 2) {
                append_point(gauge, vertices[indices[i]]);
                append_point(gauge, vertices[indices[i + 1]]);
            }
        }
        markers.markers.push_back(gauge);

        const size_t n = std::min(result.tracks.size(), marker_max_obstacles_);
        std::vector<uint32_t> visible_ids;
        visible_ids.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const Track& t = result.tracks[i];
            const uint8_t severity = severity_of(t);
            markers.markers.push_back(box_marker(out.header, t, severity));
            markers.markers.push_back(edge_marker(out.header, t, severity));
            markers.markers.push_back(label_marker(out.header, t, severity));
            visible_ids.push_back(t.id);
        }
        for (uint32_t id : tracked_ids_) {
            if (std::find(visible_ids.begin(), visible_ids.end(), id) != visible_ids.end()) {
                continue;
            }
            markers.markers.push_back(
                delete_marker(out.header, "obstacle_box", static_cast<int32_t>(id)));
            markers.markers.push_back(
                delete_marker(out.header, "obstacle_edges", static_cast<int32_t>(id)));
            markers.markers.push_back(
                delete_marker(out.header, "obstacle_label", static_cast<int32_t>(id)));
        }
        tracked_ids_ = visible_ids;

        pub_markers_->publish(markers);
    }

    static void append_point(Marker& marker, const Eigen::Vector3f& p) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        marker.points.push_back(point);
    }

    static void marker_colour(Marker& marker, uint8_t severity) {
        if (severity == Obstacle::SEVERITY_CRITICAL) {
            marker.color.r = 1.0f;
            marker.color.g = 0.20f;
            marker.color.b = 0.20f;
        } else if (severity == Obstacle::SEVERITY_WARNING) {
            marker.color.r = 1.0f;
            marker.color.g = 0.65f;
            marker.color.b = 0.10f;
        } else {
            marker.color.r = 0.95f;
            marker.color.g = 0.90f;
            marker.color.b = 0.30f;
        }
    }

    static Marker box_marker(const std_msgs::msg::Header& header, const Track& t,
                             uint8_t severity) {
        Marker m;
        m.header = header;
        m.ns = "obstacle_box";
        m.id = static_cast<int32_t>(t.id);
        m.type = Marker::CUBE;
        m.action = Marker::ADD;
        m.pose.position.x = t.state(0);
        m.pose.position.y = t.state(1);
        m.pose.position.z = t.state(2);
        m.pose.orientation.w = 1.0;
        m.scale.x = std::max(t.extent(0), 0.10f);
        m.scale.y = std::max(t.extent(1), 0.10f);
        m.scale.z = std::max(t.extent(2), 0.10f);
        marker_colour(m, severity);
        m.color.a = 0.18f;
        m.frame_locked = true;
        return m;
    }

    static Marker edge_marker(const std_msgs::msg::Header& header, const Track& t,
                              uint8_t severity) {
        Marker m;
        m.header = header;
        m.ns = "obstacle_edges";
        m.id = static_cast<int32_t>(t.id);
        m.type = Marker::LINE_LIST;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.05;
        marker_colour(m, severity);
        m.color.a = 0.95f;
        m.frame_locked = true;

        const float hx = std::max(t.extent(0), 0.10f) * 0.5f;
        const float hy = std::max(t.extent(1), 0.10f) * 0.5f;
        const float hz = std::max(t.extent(2), 0.10f) * 0.5f;
        const Eigen::Vector3f c = t.state.head<3>();
        const Eigen::Vector3f corner[8] = {
            c + Eigen::Vector3f(-hx, -hy, -hz), c + Eigen::Vector3f(hx, -hy, -hz),
            c + Eigen::Vector3f(hx, hy, -hz),   c + Eigen::Vector3f(-hx, hy, -hz),
            c + Eigen::Vector3f(-hx, -hy, hz),  c + Eigen::Vector3f(hx, -hy, hz),
            c + Eigen::Vector3f(hx, hy, hz),    c + Eigen::Vector3f(-hx, hy, hz)};
        const int edge[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                 {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        for (const auto& e : edge) {
            append_point(m, corner[e[0]]);
            append_point(m, corner[e[1]]);
        }
        return m;
    }

    static Marker label_marker(const std_msgs::msg::Header& header, const Track& t,
                               uint8_t severity) {
        Marker m;
        m.header = header;
        m.ns = "obstacle_label";
        m.id = static_cast<int32_t>(t.id);
        m.type = Marker::TEXT_VIEW_FACING;
        m.action = Marker::ADD;
        m.pose.position.x = t.state(0);
        m.pose.position.y = t.state(1);
        m.pose.position.z = t.state(2) + std::max(t.extent(2), 0.10f) * 0.5f + 0.4f;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.7;
        marker_colour(m, severity);
        m.color.a = 1.0f;
        m.frame_locked = true;
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%.1f m", static_cast<double>(t.nearest_range));
        m.text = buf;
        return m;
    }

    static Marker delete_marker(const std_msgs::msg::Header& header, const std::string& ns,
                                int32_t id) {
        Marker m;
        m.header = header;
        m.ns = ns;
        m.id = id;
        m.action = Marker::DELETE;
        return m;
    }

    Obstacle to_obstacle(const Track& t) {
        Obstacle o;
        o.track_id = t.id;
        o.persistence_frames = t.hits;
        o.position.x = t.state(0);
        o.position.y = t.state(1);
        o.position.z = t.state(2);
        o.extent.x = t.extent(0);
        o.extent.y = t.extent(1);
        o.extent.z = t.extent(2);
        o.volume_m3 = t.volume;
        o.range_m = t.nearest_range;
        o.centroid_range_m = t.state.head<3>().norm();
        o.ttc_s = t.ttc;
        o.confidence = std::min(1.0f, static_cast<float>(t.hits) * 0.1f);
        o.votes_free_space = t.votes_free_space;
        o.votes_no_return = t.votes_no_return;
        o.votes_geometry = t.votes_geometry;
        o.point_count = t.point_count;
        o.severity = severity_of(t);
        o.reason = reason_of(t);
        return o;
    }

    uint8_t severity_of(const Track& t) {
        const bool critical = t.nearest_range < params_.critical_range_m ||
                              (t.ttc >= 0.0f && t.ttc < params_.critical_ttc_s);
        if (critical) {
            return Obstacle::SEVERITY_CRITICAL;
        }
        if (t.nearest_range < params_.warning_range_m) {
            return Obstacle::SEVERITY_WARNING;
        }
        return Obstacle::SEVERITY_INFO;
    }

    static std::string reason_of(const Track& t) {
        std::string reason;
        if (t.votes_free_space != 0) {
            reason += "free_space+";
        }
        if (t.votes_no_return != 0) {
            reason += "no_return+";
        }
        if (t.votes_geometry != 0) {
            reason += "geometry+";
        }
        if (!reason.empty()) {
            reason.pop_back();
        }
        return reason;
    }

    static std::string reason_of(const Cluster& c) {
        std::string reason;
        if (c.votes_free_space != 0) {
            reason += "free_space+";
        }
        if (c.votes_no_return != 0) {
            reason += "no_return+";
        }
        if (c.votes_geometry != 0) {
            reason += "geometry+";
        }
        if (!reason.empty()) {
            reason.pop_back();
        }
        return reason;
    }

    void check_stale() {
        const auto now_tp = std::chrono::steady_clock::now();
        if (last_output_.time_since_epoch().count() > 0) {
            const double idle = std::chrono::duration<double>(now_tp - last_output_).count();
            if (idle < stale_timeout_) {
                return;
            }
        }

        ObstacleArray out;
        out.header.stamp = now();
        out.status = ObstacleArray::STATUS_DEGRADED;
        out.nearest_range_m = -1.0f;
        out.obstacles_detected = 0;
        out.fps = 0.0f;
        out.model_ready = false;
        pub_obstacles_->publish(out);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "полные кадры не поступают, STATUS_DEGRADED");
    }

    struct FrameSlot {
        CleanCloudMsg::ConstSharedPtr cloud;
        AnomalySetMsg::ConstSharedPtr geometry;
        AnomalySetMsg::ConstSharedPtr no_return;
        AnomalySetMsg::ConstSharedPtr free_space;
    };

    static Eigen::Isometry3d to_pose(const geometry_msgs::msg::PoseStamped& msg) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        const Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x,
                                   msg.pose.orientation.y, msg.pose.orientation.z);
        pose.rotate(q.normalized());
        pose.pretranslate(
            Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
        return pose;
    }

    ClearanceGaugeParams gauge_params_;
    FusionParams params_;
    std::string forward_axis_name_;
    double default_dt_ = 0.1;
    double stale_timeout_ = 1.0;
    double train_speed_ = 0.0;
    bool publish_markers_ = true;
    float marker_max_range_ = 60.0f;
    size_t marker_max_obstacles_ = 16;
    std::vector<uint32_t> tracked_ids_;
    uint64_t frames_processed_ = 0;
    size_t max_pending_ = 16;

    std::unique_ptr<FusionPipeline> pipeline_;
    std::map<uint64_t, FrameSlot> slots_;
    std::map<uint64_t, Eigen::Isometry3d> poses_;
    uint64_t last_stamp_ns_ = 0;
    float fps_ = 0.0f;
    std::chrono::steady_clock::time_point last_output_{};

    rclcpp::Subscription<CleanCloudMsg>::SharedPtr sub_clean_;
    rclcpp::Subscription<AnomalySetMsg>::SharedPtr sub_geometry_;
    rclcpp::Subscription<AnomalySetMsg>::SharedPtr sub_no_return_;
    rclcpp::Subscription<AnomalySetMsg>::SharedPtr sub_free_space_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<ObstacleArray>::SharedPtr pub_obstacles_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_explain_;
    rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;
    rclcpp::TimerBase::SharedPtr stale_timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::FusionNode>());
    rclcpp::shutdown();
    return 0;
}
