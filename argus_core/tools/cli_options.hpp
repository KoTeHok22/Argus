#pragma once

#include <cstdint>
#include <string>

namespace argus::cli {

struct Options {
    std::string path;
    std::string forward_axis = "-y";
    std::string ground_method = "z";
    std::string track_centerline_path;

    uint32_t limit = 0;
    uint32_t warmup_frames = 10;
    uint32_t carve_stride = 1;
    uint32_t odom_iterations = 4;
    uint32_t min_cluster_size_near = 15;
    uint32_t min_cluster_size_mid = 6;
    uint32_t min_cluster_size_far = 3;
    uint32_t min_cluster_size_long = 2;
    uint32_t min_hits = 3;
    uint32_t min_hits_far = 5;
    uint32_t diagnostic_near_baseline_window = 200;
    uint32_t frozen_min_observations = 0;
    uint32_t temporal_window = 5;
    uint32_t world_warmup = 10;
    uint32_t world_evidence = 3;
    uint32_t world_max_component_points = 120;
    uint32_t world_max_track_age = 8;

    float verify_min_score = 0.35f;
    float carve_max_range = 90.0f;
    float carve_half_width = 3.0f;
    float gauge_height = 2.10f;
    float gauge_half_width = 1.50f;
    float gauge_sensor_height = 1.075f;
    float gauge_safety_margin = 0.10f;
    float gauge_base_offset = 0.20f;
    float gauge_chamfer = 0.20f;
    float gauge_nose_offset = 0.00f;
    float gauge_max_range = 300.0f;
    float odom_max_range = 120.0f;
    float size_long_range = 300.0f;
    float geom_max_target_range = 45.0f;
    float geom_residual_threshold_far = 1.0f;
    float diagnostic_near_baseline_min = 18.5f;
    float diagnostic_near_baseline_max = 40.0f;
    float profile_residual_threshold = 1.0f;
    float frozen_min_confidence = -1.0f;
    float temporal_threshold = 1.0f;
    float world_voxel = 0.30f;
    float world_fitness = 0.03f;
    float world_max_y_extent = 1.2f;
    float world_max_axis_offset = 0.0f;

    bool fusion_mode = false;
    bool odometry_mode = false;
    bool tunnel_mode = false;
    bool debug_clusters = false;
    bool free_space_vote = false;
    bool verify_clusters = false;
    bool ground_filter = false;
    bool profile_gauge_mode = false;
    bool free_space_freeze = false;
    bool temporal_residual = false;
    bool temporal_vote = false;
    bool world_residual = false;
    bool world_component_mode = false;
    bool world_component_detail = false;
    bool track_corridor_mode = false;
    bool odom_heading = false;
};

enum class ParseStatus { Ok, Help, Error };

struct ParseResult {
    ParseStatus status = ParseStatus::Ok;
    Options options;
    std::string error;
};

ParseResult parse_cli(int argc, char** argv);

std::string usage_text();

} // namespace argus::cli
