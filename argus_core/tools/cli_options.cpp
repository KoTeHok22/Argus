#include "cli_options.hpp"

#include <algorithm>
#include <cstdlib>

namespace argus::cli {
namespace {

uint32_t to_uint(const char* text, uint32_t minimum) {
    return static_cast<uint32_t>(std::max<int>(minimum, std::atoi(text)));
}

float to_float(const char* text) {
    return static_cast<float>(std::atof(text));
}

} // namespace

std::string usage_text() {
    return "usage: offline_detector <frames.bin> [--frames N] [--fusion] [--odometry] "
           "[--tunnel] [--warmup N] [--free-space-vote] [--verify] "
           "[--verify-min-score C] [--carve-stride N] "
           "[--carve-max-range M] [--carve-half-width W] [--forward-axis x|-x|y|-y] "
           "[--gauge-height M] [--gauge-half-width M] [--gauge-sensor-height M] "
           "[--gauge-safety-margin M] [--gauge-base-offset M] "
           "[--gauge-chamfer M] [--gauge-nose-offset M] [--gauge-max-range M] "
           "[--odom-max-range M] [--odom-iterations N] [--ground-method patchworkpp|z] "
           "[--min-cluster-size N] [--min-cluster-size-mid N] "
           "[--min-cluster-size-far N] [--min-hits N] [--min-hits-far N] "
           "[--min-cluster-size-long N] [--size-long-range M] [--geom-max-range M] "
           "[--geom-residual-far M] [--diagnostic-near-baseline-min M] "
           "[--diagnostic-near-baseline-max M] [--diagnostic-near-baseline-window N] "
           "[--temporal-residual] [--temporal-window N] [--temporal-threshold M] "
           "[--world-residual] [--world-voxel M] [--world-fitness M] [--world-warmup N] "
           "[--world-evidence N] [--world-components] [--world-component-detail] "
           "[--world-max-component-points N] [--world-max-y-extent M] "
           "[--world-max-track-age N] [--world-max-axis-offset M] "
           "[--track-corridor --track-centerline PATH] [--odom-heading] "
           "[--profile-gauge] [--profile-residual M] [--free-space-freeze] "
           "[--frozen-min-observations N] [--frozen-min-confidence C]";
}

ParseResult parse_cli(int argc, char** argv) {
    ParseResult result;
    Options& o = result.options;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) {
            o.limit = to_uint(argv[++i], 0);
        } else if (a == "--gauge-height" && i + 1 < argc) {
            o.gauge_height = to_float(argv[++i]);
        } else if (a == "--gauge-half-width" && i + 1 < argc) {
            o.gauge_half_width = to_float(argv[++i]);
        } else if (a == "--gauge-sensor-height" && i + 1 < argc) {
            o.gauge_sensor_height = to_float(argv[++i]);
        } else if (a == "--gauge-safety-margin" && i + 1 < argc) {
            o.gauge_safety_margin = to_float(argv[++i]);
        } else if (a == "--gauge-base-offset" && i + 1 < argc) {
            o.gauge_base_offset = to_float(argv[++i]);
        } else if (a == "--gauge-chamfer" && i + 1 < argc) {
            o.gauge_chamfer = to_float(argv[++i]);
        } else if (a == "--gauge-nose-offset" && i + 1 < argc) {
            o.gauge_nose_offset = to_float(argv[++i]);
        } else if (a == "--gauge-max-range" && i + 1 < argc) {
            o.gauge_max_range = to_float(argv[++i]);
        } else if (a == "--odom-max-range" && i + 1 < argc) {
            o.odom_max_range = to_float(argv[++i]);
        } else if (a == "--odom-iterations" && i + 1 < argc) {
            o.odom_iterations = to_uint(argv[++i], 1);
        } else if (a == "--ground-method" && i + 1 < argc) {
            o.ground_method = argv[++i];
        } else if (a == "--carve-stride" && i + 1 < argc) {
            o.carve_stride = to_uint(argv[++i], 1);
        } else if (a == "--carve-max-range" && i + 1 < argc) {
            o.carve_max_range = to_float(argv[++i]);
        } else if (a == "--carve-half-width" && i + 1 < argc) {
            o.carve_half_width = to_float(argv[++i]);
        } else if (a == "--warmup" && i + 1 < argc) {
            o.warmup_frames = to_uint(argv[++i], 0);
        } else if (a == "--min-cluster-size" && i + 1 < argc) {
            o.min_cluster_size_near = to_uint(argv[++i], 1);
        } else if (a == "--min-cluster-size-mid" && i + 1 < argc) {
            o.min_cluster_size_mid = to_uint(argv[++i], 1);
        } else if (a == "--min-cluster-size-far" && i + 1 < argc) {
            o.min_cluster_size_far = to_uint(argv[++i], 1);
        } else if (a == "--min-cluster-size-long" && i + 1 < argc) {
            o.min_cluster_size_long = to_uint(argv[++i], 1);
        } else if (a == "--size-long-range" && i + 1 < argc) {
            o.size_long_range = to_float(argv[++i]);
        } else if (a == "--min-hits" && i + 1 < argc) {
            o.min_hits = to_uint(argv[++i], 1);
        } else if (a == "--min-hits-far" && i + 1 < argc) {
            o.min_hits_far = to_uint(argv[++i], 1);
        } else if (a == "--free-space-vote") {
            o.free_space_vote = true;
        } else if (a == "--verify") {
            o.verify_clusters = true;
        } else if (a == "--verify-min-score" && i + 1 < argc) {
            o.verify_min_score = to_float(argv[++i]);
        } else if (a == "--fusion") {
            o.fusion_mode = true;
        } else if (a == "--odometry") {
            o.odometry_mode = true;
        } else if (a == "--odom-heading") {
            o.odometry_mode = true;
            o.odom_heading = true;
        } else if (a == "--tunnel") {
            o.tunnel_mode = true;
        } else if (a == "--debug-clusters") {
            o.debug_clusters = true;
        } else if (a == "--geom-max-range" && i + 1 < argc) {
            o.geom_max_target_range = to_float(argv[++i]);
        } else if (a == "--geom-residual-far" && i + 1 < argc) {
            o.geom_residual_threshold_far = to_float(argv[++i]);
        } else if (a == "--diagnostic-near-baseline-min" && i + 1 < argc) {
            o.diagnostic_near_baseline_min = to_float(argv[++i]);
        } else if (a == "--diagnostic-near-baseline-max" && i + 1 < argc) {
            o.diagnostic_near_baseline_max = to_float(argv[++i]);
        } else if (a == "--diagnostic-near-baseline-window" && i + 1 < argc) {
            o.diagnostic_near_baseline_window = to_uint(argv[++i], 1);
        } else if (a == "--ground-filter") {
            o.ground_filter = true;
        } else if (a == "--profile-gauge") {
            o.profile_gauge_mode = true;
        } else if (a == "--profile-residual" && i + 1 < argc) {
            o.profile_residual_threshold = to_float(argv[++i]);
        } else if (a == "--free-space-freeze") {
            o.free_space_freeze = true;
        } else if (a == "--frozen-min-observations" && i + 1 < argc) {
            o.frozen_min_observations = to_uint(argv[++i], 1);
        } else if (a == "--frozen-min-confidence" && i + 1 < argc) {
            o.frozen_min_confidence = to_float(argv[++i]);
        } else if (a == "--temporal-residual") {
            o.temporal_residual = true;
        } else if (a == "--temporal-vote") {
            o.temporal_vote = true;
        } else if (a == "--temporal-window" && i + 1 < argc) {
            o.temporal_window = to_uint(argv[++i], 1);
        } else if (a == "--temporal-threshold" && i + 1 < argc) {
            o.temporal_threshold = to_float(argv[++i]);
        } else if (a == "--world-residual") {
            o.world_residual = true;
        } else if (a == "--world-components") {
            o.world_residual = true;
            o.world_component_mode = true;
        } else if (a == "--world-component-detail") {
            o.world_component_detail = true;
        } else if (a == "--track-corridor") {
            o.world_residual = true;
            o.world_component_mode = true;
            o.track_corridor_mode = true;
        } else if (a == "--track-centerline" && i + 1 < argc) {
            o.track_centerline_path = argv[++i];
        } else if (a == "--track-centerline") {
            result.status = ParseStatus::Error;
            result.error = "--track-centerline requires a file path";
            return result;
        } else if (a == "--world-voxel" && i + 1 < argc) {
            o.world_voxel = to_float(argv[++i]);
        } else if (a == "--world-fitness" && i + 1 < argc) {
            o.world_fitness = to_float(argv[++i]);
        } else if (a == "--world-warmup" && i + 1 < argc) {
            o.world_warmup = to_uint(argv[++i], 1);
        } else if (a == "--world-evidence" && i + 1 < argc) {
            o.world_evidence = to_uint(argv[++i], 1);
        } else if (a == "--world-max-component-points" && i + 1 < argc) {
            o.world_max_component_points = to_uint(argv[++i], 2);
        } else if (a == "--world-max-y-extent" && i + 1 < argc) {
            o.world_max_y_extent = to_float(argv[++i]);
        } else if (a == "--world-max-track-age" && i + 1 < argc) {
            o.world_max_track_age = to_uint(argv[++i], 1);
        } else if (a == "--world-max-axis-offset" && i + 1 < argc) {
            o.world_max_axis_offset = to_float(argv[++i]);
        } else if (a == "--forward-axis" && i + 1 < argc) {
            o.forward_axis = argv[++i];
        } else {
            if (a.rfind("--", 0) == 0) {
                result.status = ParseStatus::Error;
                result.error = "unknown or incomplete option: " + a;
                return result;
            }
            o.path = a;
        }
    }
    if (o.path.empty()) {
        result.status = ParseStatus::Help;
    }
    return result;
}

} // namespace argus::cli
