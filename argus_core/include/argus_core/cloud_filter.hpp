
#pragma once

#include <optional>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "argus_core/types.hpp"

namespace argus {

struct CloudLayout {
    uint32_t point_step = 0;
    uint32_t offset_x = 0, offset_y = 0, offset_z = 0;
    uint32_t offset_intensity = 0;
    uint32_t offset_ring = 0;
    uint32_t offset_timestamp = 0;
    bool has_intensity = false;
    bool has_ring = false;
    bool has_timestamp = false;
    uint32_t n_points = 0;
    std::string error;
};

std::optional<CloudLayout> parse_layout(const sensor_msgs::msg::PointCloud2& msg);

struct FilterParams {
    float min_range = 0.5f;
    float max_range = 400.0f;
    float max_abs_coord = 1.0e4f;
    bool drop_zero_xyz = true;
    bool drop_nonfinite = true;
};

struct FilterStats {
    uint32_t n_raw = 0;
    uint32_t n_valid = 0;
    uint32_t n_rejected = 0;
    uint32_t reject_zero = 0;
    uint32_t reject_nonfinite = 0;
    uint32_t reject_out_of_range = 0;
};

CleanCloud filter_and_index(const sensor_msgs::msg::PointCloud2& msg, const CloudLayout& layout,
                            const FilterParams& params, FilterStats* stats = nullptr);

} // namespace argus
