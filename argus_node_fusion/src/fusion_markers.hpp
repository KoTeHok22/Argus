
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "argus_core/tracking.hpp"

namespace argus {

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

MarkerArray build_frame_markers(const std_msgs::msg::Header& header,
                                const std::vector<Eigen::Vector3f>& gauge_vertices,
                                const std::vector<uint32_t>& gauge_indices,
                                const std::vector<Track>& tracks,
                                const std::vector<uint8_t>& severities, size_t max_obstacles,
                                std::vector<uint32_t>& tracked_ids);

} // namespace argus
