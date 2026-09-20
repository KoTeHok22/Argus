
#include "fusion_markers.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

namespace argus {

namespace {

constexpr uint8_t kSeverityInfo = 0;
constexpr uint8_t kSeverityWarning = 1;
constexpr uint8_t kSeverityCritical = 2;

void append_point(Marker& marker, const Eigen::Vector3f& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
}

void marker_colour(Marker& marker, uint8_t severity) {
    if (severity == kSeverityCritical) {
        marker.color.r = 1.0f;
        marker.color.g = 0.20f;
        marker.color.b = 0.20f;
    } else if (severity == kSeverityWarning) {
        marker.color.r = 1.0f;
        marker.color.g = 0.65f;
        marker.color.b = 0.10f;
    } else {
        marker.color.r = 0.95f;
        marker.color.g = 0.90f;
        marker.color.b = 0.30f;
    }
}

Marker box_marker(const std_msgs::msg::Header& header, const Track& t, uint8_t severity) {
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

Marker edge_marker(const std_msgs::msg::Header& header, const Track& t, uint8_t severity) {
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

Marker label_marker(const std_msgs::msg::Header& header, const Track& t, uint8_t severity) {
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

Marker delete_marker(const std_msgs::msg::Header& header, const std::string& ns, int32_t id) {
    Marker m;
    m.header = header;
    m.ns = ns;
    m.id = id;
    m.action = Marker::DELETE;
    return m;
}

Marker gauge_marker(const std_msgs::msg::Header& header,
                    const std::vector<Eigen::Vector3f>& vertices,
                    const std::vector<uint32_t>& indices) {
    Marker gauge;
    gauge.header = header;
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
    return gauge;
}

} // namespace

MarkerArray build_frame_markers(const std_msgs::msg::Header& header,
                                const std::vector<Eigen::Vector3f>& gauge_vertices,
                                const std::vector<uint32_t>& gauge_indices,
                                const std::vector<Track>& tracks,
                                const std::vector<uint8_t>& severities, size_t max_obstacles,
                                std::vector<uint32_t>& tracked_ids) {
    MarkerArray markers;
    markers.markers.push_back(gauge_marker(header, gauge_vertices, gauge_indices));

    const size_t n = std::min(tracks.size(), max_obstacles);
    std::vector<uint32_t> visible_ids;
    visible_ids.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const Track& t = tracks[i];
        const uint8_t severity = severities[i];
        markers.markers.push_back(box_marker(header, t, severity));
        markers.markers.push_back(edge_marker(header, t, severity));
        markers.markers.push_back(label_marker(header, t, severity));
        visible_ids.push_back(t.id);
    }
    for (uint32_t id : tracked_ids) {
        if (std::find(visible_ids.begin(), visible_ids.end(), id) != visible_ids.end()) {
            continue;
        }
        markers.markers.push_back(delete_marker(header, "obstacle_box", static_cast<int32_t>(id)));
        markers.markers.push_back(
            delete_marker(header, "obstacle_edges", static_cast<int32_t>(id)));
        markers.markers.push_back(
            delete_marker(header, "obstacle_label", static_cast<int32_t>(id)));
    }
    tracked_ids = visible_ids;
    return markers;
}

} // namespace argus
