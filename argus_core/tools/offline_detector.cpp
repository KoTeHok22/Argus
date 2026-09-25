
#include <argus_core/anomaly.hpp>
#include <argus_core/clustering.hpp>
#include <argus_core/fusion.hpp>
#include <argus_core/gauge.hpp>
#include <argus_core/ground.hpp>
#include <argus_core/odometry.hpp>
#include <argus_core/range_image.hpp>
#include <argus_core/track_corridor.hpp>
#include <argus_core/tunnel_model.hpp>
#include <argus_core/tunnel_profile.hpp>
#include <argus_core/types.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr char kMagic[8] = "ARGFRM1";

struct Frame {
    double stamp_s = 0.0;
    uint32_t n_raw = 0;
    std::vector<float> x, y, z, intensity;
    std::vector<uint32_t> raw_idx;
    std::vector<uint32_t> no_return_raw;
};

struct WorldSample {
    Eigen::Vector3f world;
    float forward = 0.0f;
};

struct WorldComponent {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f low = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f high = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
    float forward = std::numeric_limits<float>::max();
    uint32_t points = 0;
};

struct WorldTrack {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f low = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f high = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
    uint32_t last_frame = 0;
    uint32_t first_frame = 0;
    uint32_t hits = 0;
    uint32_t points = 0;
    float forward = 0.0f;
};

std::vector<WorldComponent> world_components(const std::vector<WorldSample>& samples,
                                             float radius) {
    std::vector<WorldComponent> components;
    std::vector<uint8_t> visited(samples.size(), 0);
    const float radius_sq = radius * radius;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (visited[i]) {
            continue;
        }
        WorldComponent component;
        std::vector<size_t> pending{i};
        visited[i] = 1;
        while (!pending.empty()) {
            const size_t current = pending.back();
            pending.pop_back();
            const auto& sample = samples[current];
            component.center += sample.world;
            component.low = component.low.cwiseMin(sample.world);
            component.high = component.high.cwiseMax(sample.world);
            component.forward = std::min(component.forward, sample.forward);
            ++component.points;
            for (size_t j = 0; j < samples.size(); ++j) {
                if (!visited[j] && (samples[j].world - sample.world).squaredNorm() <= radius_sq) {
                    visited[j] = 1;
                    pending.push_back(j);
                }
            }
        }
        component.center /= static_cast<float>(component.points);
        components.push_back(component);
    }
    return components;
}

struct WorldComponentResult {
    bool alert = false;
    float forward = -1.0f;
    uint32_t components = 0;
    uint32_t tracks = 0;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    uint32_t points = 0;
    uint32_t hits = 0;
    uint32_t oversized = 0;
    std::vector<WorldTrack> confirmed;
};

WorldComponentResult update_world_tracks(std::vector<WorldTrack>& tracks,
                                         const std::vector<WorldComponent>& components,
                                         uint32_t frame, float match_radius,
                                         uint32_t max_component_points, float max_world_y_extent_m,
                                         uint32_t max_track_age) {
    WorldComponentResult result;
    result.components = static_cast<uint32_t>(components.size());
    tracks.erase(
        std::remove_if(tracks.begin(), tracks.end(),
                       [frame](const WorldTrack& track) { return frame - track.last_frame > 5; }),
        tracks.end());
    std::vector<uint8_t> matched(tracks.size(), 0);
    for (const auto& component : components) {
        if (component.points > max_component_points) {
            ++result.oversized;
            continue;
        }
        if (component.points < 2) {
            continue;
        }
        int best = -1;
        float best_distance = match_radius * match_radius;
        for (size_t i = 0; i < tracks.size(); ++i) {
            const float distance = (tracks[i].center - component.center).squaredNorm();
            if (!matched[i] && distance < best_distance) {
                best = static_cast<int>(i);
                best_distance = distance;
            }
        }
        if (best < 0) {
            WorldTrack track;
            track.center = component.center;
            track.low = component.low;
            track.high = component.high;
            track.first_frame = frame;
            track.last_frame = frame;
            track.hits = 1;
            track.points = component.points;
            track.forward = component.forward;
            tracks.push_back(track);
            matched.push_back(1);
            continue;
        }
        WorldTrack& track = tracks[best];
        matched[best] = 1;
        track.center = 0.5f * (track.center + component.center);
        track.low = track.low.cwiseMin(component.low);
        track.high = track.high.cwiseMax(component.high);
        track.forward = component.forward;
        track.last_frame = frame;
        ++track.hits;
        track.points += component.points;
    }
    result.tracks = static_cast<uint32_t>(tracks.size());
    for (const auto& track : tracks) {
        const Eigen::Vector3f extent = track.high - track.low;
        if (track.last_frame == frame && track.hits >= 3 &&
            frame - track.first_frame <= max_track_age && track.points >= 15 &&
            extent.z() >= 0.20f && extent.x() <= 1.0f && extent.y() <= max_world_y_extent_m) {
            result.confirmed.push_back(track);
            if (!result.alert || track.forward < result.forward) {
                result.alert = true;
                result.forward = track.forward;
                result.center = track.center;
                result.extent = extent;
                result.points = track.points;
                result.hits = track.hits;
            }
        }
    }
    return result;
}

bool read_u32(std::ifstream& in, uint32_t& v) {
    return static_cast<bool>(in.read(reinterpret_cast<char*>(&v), 4));
}

bool read_frame(std::ifstream& in, Frame& f) {
    if (!in.read(reinterpret_cast<char*>(&f.stamp_s), 8) || !read_u32(in, f.n_raw)) {
        return false;
    }
    uint32_t n_valid = 0, n_no_return = 0;
    if (!read_u32(in, n_valid) || !read_u32(in, n_no_return)) {
        return false;
    }
    f.x.resize(n_valid);
    f.y.resize(n_valid);
    f.z.resize(n_valid);
    f.intensity.resize(n_valid);
    f.raw_idx.resize(n_valid);
    for (uint32_t i = 0; i < n_valid; ++i) {
        if (!in.read(reinterpret_cast<char*>(&f.x[i]), 4) ||
            !in.read(reinterpret_cast<char*>(&f.y[i]), 4) ||
            !in.read(reinterpret_cast<char*>(&f.z[i]), 4) ||
            !in.read(reinterpret_cast<char*>(&f.intensity[i]), 4) ||
            !in.read(reinterpret_cast<char*>(&f.raw_idx[i]), 4)) {
            return false;
        }
    }
    f.no_return_raw.resize(n_no_return);
    if (n_no_return > 0 && !in.read(reinterpret_cast<char*>(f.no_return_raw.data()),
                                    static_cast<std::streamsize>(n_no_return) * 4)) {
        return false;
    }
    return true;
}

argus::CleanCloud to_clean_cloud(const Frame& f) {
    argus::CleanCloud c;
    c.n_raw = f.n_raw;
    c.x = f.x;
    c.y = f.y;
    c.z = f.z;
    c.intensity = f.intensity;
    c.raw_idx = f.raw_idx;
    c.no_return_raw = f.no_return_raw;
    return c;
}

void report(uint32_t frame, const argus::AnomalySet& set, const argus::RangeImage& ri,
            const argus::CleanCloud& cloud) {
    const uint32_t h = ri.height;
    if (set.indices.empty() && set.cells.empty()) {
        std::cout << frame << ',' << set.source << ",0,0,,,,\n";
        return;
    }
    if (!set.cells.empty()) {
        uint32_t az_min = 1u << 30, az_max = 0;
        for (uint32_t cell : set.cells) {
            az_min = std::min(az_min, cell / h);
            az_max = std::max(az_max, cell / h);
        }
        std::cout << frame << ',' << set.source << ",0," << set.cells.size() << ",," << az_min
                  << ',' << az_max << ",\n";
        return;
    }

    std::vector<std::vector<uint32_t>> groups;
    std::vector<float> group_nearest;
    for (uint32_t idx : set.indices) {
        if (idx >= cloud.size()) {
            continue;
        }
        const uint32_t src = cloud.raw_idx.empty() ? idx : cloud.raw_idx[idx];
        const float v = std::sqrt(cloud.x[idx] * cloud.x[idx] + cloud.y[idx] * cloud.y[idx] +
                                  cloud.z[idx] * cloud.z[idx]);
        bool placed = false;
        for (size_t g = 0; g < groups.size(); ++g) {
            if (std::fabs(v - group_nearest[g]) < 3.0f) {
                groups[g].push_back(src);
                group_nearest[g] = std::min(group_nearest[g], v);
                placed = true;
                break;
            }
        }
        if (!placed) {
            groups.push_back({src});
            group_nearest.push_back(v);
        }
    }

    for (size_t g = 0; g < groups.size(); ++g) {
        uint32_t az_min = 1u << 30, az_max = 0;
        for (uint32_t cell : groups[g]) {
            az_min = std::min(az_min, cell / h);
            az_max = std::max(az_max, cell / h);
        }
        std::cout << frame << ',' << set.source << ',' << groups[g].size() << ','
                  << groups[g].size() << ',' << group_nearest[g] << ',' << az_min << ',' << az_max
                  << ",\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string path;
    std::string forward_axis = "-y";
    uint32_t limit = 0;
    bool fusion_mode = false;
    bool odometry_mode = false;
    bool tunnel_mode = false;
    bool debug_clusters = false;
    uint32_t warmup_frames = 10;
    bool free_space_vote = false;
    uint32_t carve_stride = 1;
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
    uint32_t odom_iterations = 4;
    uint32_t min_cluster_size_near = 15;
    uint32_t min_cluster_size_mid = 6;
    uint32_t min_cluster_size_far = 3;
    uint32_t min_cluster_size_long = 2;
    float size_long_range = 300.0f;
    uint32_t min_hits = 3;
    uint32_t min_hits_far = 5;
    float geom_max_target_range = 45.0f;
    float geom_residual_threshold_far = 1.0f;
    float diagnostic_near_baseline_min = 18.5f;
    float diagnostic_near_baseline_max = 40.0f;
    uint32_t diagnostic_near_baseline_window = 200;
    bool ground_filter = false;
    bool profile_gauge_mode = false;
    float profile_residual_threshold = 1.0f;
    bool free_space_freeze = false;
    uint32_t frozen_min_observations = 0;
    float frozen_min_confidence = -1.0f;
    bool temporal_residual = false;
    uint32_t temporal_window = 5;
    float temporal_threshold = 1.0f;
    bool world_residual = false;
    bool world_component_mode = false;
    bool world_component_detail = false;
    bool track_corridor_mode = false;
    std::string track_centerline_path;
    bool odom_heading = false;
    float world_voxel = 0.30f;
    float world_fitness = 0.03f;
    uint32_t world_warmup = 10;
    uint32_t world_evidence = 3;
    uint32_t world_max_component_points = 120;
    float world_max_y_extent = 1.2f;
    uint32_t world_max_track_age = 8;
    float world_max_axis_offset = 0.0f;
    std::string ground_method = "z";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) {
            limit = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (a == "--gauge-height" && i + 1 < argc) {
            gauge_height = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-half-width" && i + 1 < argc) {
            gauge_half_width = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-sensor-height" && i + 1 < argc) {
            gauge_sensor_height = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-safety-margin" && i + 1 < argc) {
            gauge_safety_margin = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-base-offset" && i + 1 < argc) {
            gauge_base_offset = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-chamfer" && i + 1 < argc) {
            gauge_chamfer = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-nose-offset" && i + 1 < argc) {
            gauge_nose_offset = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--gauge-max-range" && i + 1 < argc) {
            gauge_max_range = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--odom-max-range" && i + 1 < argc) {
            odom_max_range = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--odom-iterations" && i + 1 < argc) {
            odom_iterations = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--ground-method" && i + 1 < argc) {
            ground_method = argv[++i];
        } else if (a == "--carve-stride" && i + 1 < argc) {
            carve_stride = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--carve-max-range" && i + 1 < argc) {
            carve_max_range = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--carve-half-width" && i + 1 < argc) {
            carve_half_width = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--warmup" && i + 1 < argc) {
            warmup_frames = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (a == "--min-cluster-size" && i + 1 < argc) {
            min_cluster_size_near = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--min-cluster-size-mid" && i + 1 < argc) {
            min_cluster_size_mid = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--min-cluster-size-far" && i + 1 < argc) {
            min_cluster_size_far = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--min-cluster-size-long" && i + 1 < argc) {
            min_cluster_size_long = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--size-long-range" && i + 1 < argc) {
            size_long_range = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--min-hits" && i + 1 < argc) {
            min_hits = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--min-hits-far" && i + 1 < argc) {
            min_hits_far = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--free-space-vote") {
            free_space_vote = true;
        } else if (a == "--fusion") {
            fusion_mode = true;
        } else if (a == "--odometry") {
            odometry_mode = true;
        } else if (a == "--odom-heading") {
            odometry_mode = true;
            odom_heading = true;
        } else if (a == "--tunnel") {
            tunnel_mode = true;
        } else if (a == "--debug-clusters") {
            debug_clusters = true;
        } else if (a == "--geom-max-range" && i + 1 < argc) {
            geom_max_target_range = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--geom-residual-far" && i + 1 < argc) {
            geom_residual_threshold_far = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--diagnostic-near-baseline-min" && i + 1 < argc) {
            diagnostic_near_baseline_min = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--diagnostic-near-baseline-max" && i + 1 < argc) {
            diagnostic_near_baseline_max = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--diagnostic-near-baseline-window" && i + 1 < argc) {
            diagnostic_near_baseline_window =
                static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--ground-filter") {
            ground_filter = true;
        } else if (a == "--profile-gauge") {
            profile_gauge_mode = true;
        } else if (a == "--profile-residual" && i + 1 < argc) {
            profile_residual_threshold = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--free-space-freeze") {
            free_space_freeze = true;
        } else if (a == "--frozen-min-observations" && i + 1 < argc) {
            frozen_min_observations = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--frozen-min-confidence" && i + 1 < argc) {
            frozen_min_confidence = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--temporal-residual") {
            temporal_residual = true;
        } else if (a == "--temporal-window" && i + 1 < argc) {
            temporal_window = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--temporal-threshold" && i + 1 < argc) {
            temporal_threshold = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-residual") {
            world_residual = true;
        } else if (a == "--world-components") {
            world_residual = true;
            world_component_mode = true;
        } else if (a == "--world-component-detail") {
            world_component_detail = true;
        } else if (a == "--track-corridor") {
            world_residual = true;
            world_component_mode = true;
            track_corridor_mode = true;
        } else if (a == "--track-centerline" && i + 1 < argc) {
            track_centerline_path = argv[++i];
        } else if (a == "--track-centerline") {
            std::cerr << "--track-centerline requires a file path\n";
            return 2;
        } else if (a == "--world-voxel" && i + 1 < argc) {
            world_voxel = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-fitness" && i + 1 < argc) {
            world_fitness = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-warmup" && i + 1 < argc) {
            world_warmup = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--world-evidence" && i + 1 < argc) {
            world_evidence = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--world-max-component-points" && i + 1 < argc) {
            world_max_component_points = static_cast<uint32_t>(std::max(2, std::atoi(argv[++i])));
        } else if (a == "--world-max-y-extent" && i + 1 < argc) {
            world_max_y_extent = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-max-track-age" && i + 1 < argc) {
            world_max_track_age = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--world-max-axis-offset" && i + 1 < argc) {
            world_max_axis_offset = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--forward-axis" && i + 1 < argc) {
            forward_axis = argv[++i];
        } else {
            if (a.rfind("--", 0) == 0) {
                std::cerr << "unknown or incomplete option: " << a << '\n';
                return 2;
            }
            path = a;
        }
    }
    if (path.empty()) {
        std::cerr << "usage: offline_detector <frames.bin> [--frames N] [--fusion] [--odometry] "
                     "[--tunnel] [--warmup N] [--free-space-vote] [--carve-stride N] "
                     "[--carve-max-range M] [--carve-half-width W] [--forward-axis x|-x|y|-y] "
                     "[--gauge-height M] [--gauge-half-width M] [--gauge-sensor-height M] "
                     "[--gauge-safety-margin M] [--gauge-base-offset M] "
                     "[--gauge-chamfer M] [--gauge-nose-offset M] [--gauge-max-range M] "
                     "[--odom-max-range M] [--odom-iterations N] [--ground-method patchworkpp|z] "
                     "[--min-cluster-size N] [--min-cluster-size-mid N] "
                     "[--min-cluster-size-far N] [--min-hits N] [--min-hits-far N] "
                     "[--min-cluster-size-long N] [--size-long-range M] "
                     "[--geom-max-range M] [--geom-residual-far M] "
                     "[--diagnostic-near-baseline-min M] [--diagnostic-near-baseline-max M] "
                     "[--diagnostic-near-baseline-window N] "
                     "[--temporal-residual] [--temporal-window N] "
                     "[--temporal-threshold M] [--world-residual] [--world-voxel M] "
                     "[--world-fitness M] [--world-warmup N] [--world-evidence N] "
                     "[--world-components] [--world-component-detail] "
                     "[--world-max-component-points N] "
                     "[--world-max-y-extent M] "
                     "[--world-max-track-age N] "
                     "[--world-max-axis-offset M] "
                     "[--track-corridor --track-centerline PATH] "
                     "[--odom-heading] "
                     "[--profile-gauge] "
                     "[--profile-residual M] [--free-space-freeze] "
                     "[--frozen-min-observations N] [--frozen-min-confidence C]\n";
        return 2;
    }
    argus::TrackCorridor track_corridor({gauge_half_width, 5.0f, 1.0f});
    if (track_corridor_mode != !track_centerline_path.empty()) {
        std::cerr << "--track-corridor requires --track-centerline and vice versa\n";
        return 2;
    }
    if (world_max_component_points != 120 && !world_component_mode) {
        std::cerr << "--world-max-component-points requires a world component mode\n";
        return 2;
    }
    if (world_component_detail && !world_component_mode) {
        std::cerr << "--world-component-detail requires a world component mode\n";
        return 2;
    }
    if (world_max_track_age != 8 && !world_component_mode) {
        std::cerr << "--world-max-track-age requires a world component mode\n";
        return 2;
    }
    if (!std::isfinite(world_max_axis_offset) || world_max_axis_offset < 0.0f ||
        (world_max_axis_offset > 0.0f && !track_corridor_mode)) {
        std::cerr << "--world-max-axis-offset requires track corridor mode and nonnegative value\n";
        return 2;
    }
    if (!std::isfinite(world_max_y_extent) || world_max_y_extent <= 0.0f ||
        (world_max_y_extent != 1.2f && !world_component_mode)) {
        std::cerr << "--world-max-y-extent requires a world component mode and positive value\n";
        return 2;
    }
    if (track_corridor_mode) {
        std::ifstream centerline_file(track_centerline_path);
        std::vector<Eigen::Vector2f> centerline;
        std::string line;
        while (std::getline(centerline_file, line)) {
            std::istringstream coordinates(line);
            float x = 0.0f;
            float y = 0.0f;
            std::string extra;
            if (!(coordinates >> x >> y) || (coordinates >> extra)) {
                std::cerr << "invalid track centerline (expected two coordinates per line)\n";
                return 2;
            }
            centerline.emplace_back(x, y);
        }
        if (!centerline_file.eof() || !track_corridor.set_centerline(centerline)) {
            std::cerr << "invalid track centerline (expected world x y pairs, <=5 m apart)\n";
            return 2;
        }
    }

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::cerr << "cannot open " << path << "\n";
        return 2;
    }
    char magic[8];
    if (!in.read(magic, 8) || std::memcmp(magic, kMagic, 8) != 0) {
        std::cerr << "bad magic, expected ARGFRM1\n";
        return 2;
    }
    uint32_t n_frames = 0;
    if (!read_u32(in, n_frames)) {
        return 2;
    }
    if (limit > 0 && limit < n_frames) {
        n_frames = limit;
    }

    argus::NoReturnParams nr_params;
    argus::GeometryResidualParams geom_params;
    geom_params.max_target_range_m = geom_max_target_range;
    geom_params.residual_threshold_far_m = geom_residual_threshold_far;
    geom_params.median_window_override_min_range_m = diagnostic_near_baseline_min;
    geom_params.median_window_override_max_range_m = diagnostic_near_baseline_max;
    geom_params.median_half_window_override = diagnostic_near_baseline_window;
    argus::NoReturnDetector no_return(nr_params);
    argus::GeometryResidualDetector geometry(geom_params);
    argus::RangeImageParams ri_params;
    std::vector<std::vector<float>> temporal_history;
    std::unordered_set<int64_t> world_history;
    std::unordered_map<int64_t, uint32_t> world_evidence_counts;
    std::vector<WorldTrack> world_tracks;

    argus::ForwardAxis axis = argus::ForwardAxis::PosX;
    if (!argus::parse_forward_axis(forward_axis, axis)) {
        std::cerr << "bad --forward-axis: " << forward_axis << "\n";
        return 2;
    }
    argus::ClearanceGaugeParams gauge_params;
    gauge_params.forward_axis = axis;
    gauge_params.height = gauge_height;
    gauge_params.half_width = gauge_half_width;
    gauge_params.sensor_height = gauge_sensor_height;
    gauge_params.safety_margin = gauge_safety_margin;
    gauge_params.base_offset = gauge_base_offset;
    gauge_params.chamfer = gauge_chamfer;
    gauge_params.nose_offset = gauge_nose_offset;
    gauge_params.max_range = gauge_max_range;
    argus::FusionParams fusion_params;
    fusion_params.clustering.min_cluster_size_near = min_cluster_size_near;
    fusion_params.clustering.min_cluster_size_mid = min_cluster_size_mid;
    fusion_params.clustering.min_cluster_size_far = min_cluster_size_far;
    fusion_params.clustering.min_cluster_size_long = min_cluster_size_long;
    fusion_params.clustering.size_long_range_m = size_long_range;
    fusion_params.clustering.min_extent_m = 0.20f;
    fusion_params.tracking.min_hits_to_confirm = min_hits;
    fusion_params.tracking.min_hits_to_confirm_far = min_hits_far;
    fusion_params.ground_filter = ground_filter;
    fusion_params.use_free_space = free_space_vote;
    fusion_params.free_space_warmup_frames = warmup_frames;
    fusion_params.model.min_observations = std::max<uint32_t>(1, warmup_frames / 10);
    fusion_params.model.carve_stride_az = carve_stride;
    fusion_params.model.carve_stride_ring = carve_stride;
    fusion_params.model.carve_max_range = carve_max_range;
    fusion_params.model.carve_half_width_m = carve_half_width;
    fusion_params.compute_free_space_violation_rate = true;
    argus::FusionPipeline fusion(argus::ClearanceGauge(gauge_params), fusion_params);
    argus::GroundParams ground_params;
    ground_params.sensor_height = gauge_sensor_height;
    ground_params.forward_axis = axis;
    if (ground_method == "z" || ground_method == "z_threshold") {
        ground_params.method = argus::GroundMethod::ZThreshold;
    }
    argus::GroundSegmenter ground(ground_params);
    argus::OdometryParams odom_params;
    odom_params.forward_axis = axis;
    odom_params.max_range = odom_max_range;
    odom_params.max_iterations = odom_iterations;
    argus::TunnelOdometry odom(odom_params);

    argus::TunnelProfileParams profile_params;
    argus::TunnelProfile profile(profile_params);
    argus::FreeSpaceParams free_space_params;
    free_space_params.min_confidence = 0.85f;
    free_space_params.min_range = 3.0f;
    free_space_params.max_range = 90.0f;
    free_space_params.min_free_observations = std::max<uint32_t>(1, warmup_frames / 10);
    argus::FreeSpaceDetector free_space(const_cast<argus::TunnelModel*>(&fusion.model()),
                                        free_space_params);
    argus::TunnelModelParams frozen_params = fusion_params.model;
    if (frozen_min_observations > 0) {
        frozen_params.min_observations = frozen_min_observations;
        free_space_params.min_free_observations = frozen_min_observations;
    }
    if (frozen_min_confidence >= 0.0f) {
        free_space_params.min_confidence = frozen_min_confidence;
    }
    argus::TunnelModel frozen_model(frozen_params);
    argus::FreeSpaceDetector frozen_free_space(&frozen_model, free_space_params);

    if (tunnel_mode) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "frame,alert,model_ready,model_updated,voxels,fs_rate,fs_points,"
                     "profile_median_m,odom_valid,speed_mps";
        if (profile_gauge_mode) {
            std::cout << ",gauge_observed,gauge_residual,gauge_peak_m,gauge_nearest_m";
        }
        if (free_space_freeze) {
            std::cout << ",gauge_points,gauge_prior_covered,gauge_contradictions,"
                         "gauge_contradiction_nearest_m";
        }
        std::cout << '\n';
    } else if (odometry_mode) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "frame,valid,x,y,z,speed,corr,down,fitness";
        if (odom_heading) {
            std::cout << ",heading_deg";
        }
        std::cout << '\n';
    } else if (world_component_mode) {
        std::cout << "frame,alert,components,tracks,forward_m,candidates,pose_valid,fitness,"
                     "world_x,world_y,world_z,extent_x,extent_y,extent_z,points,hits,oversized\n";
    } else if (fusion_mode) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "frame,alert,clusters,raw,filtered,tracks,forward_m,range_m,"
                     "anom_points,anom_cells\n";
    } else {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "frame,source,points,cells,nearest_m,az_center,az_span,cells_bbox\n";
    }

    for (uint32_t k = 0; k < n_frames; ++k) {
        Frame f;
        if (!read_frame(in, f)) {
            std::cerr << "truncated frame " << k << "\n";
            return 2;
        }
        argus::CleanCloud cloud = to_clean_cloud(f);
        ground.apply(cloud);
        if (odometry_mode) {
            const argus::OdometryResult r = odom.update(cloud, f.stamp_s);
            const Eigen::Vector3d t = r.pose.translation();
            std::cout << k << ',' << (r.valid ? 1 : 0) << ',' << t.x() << ',' << t.y() << ','
                      << t.z() << ',' << r.speed_mps << ',' << r.n_correspondences << ','
                      << r.n_downsampled << ',' << r.fitness;
            if (odom_heading) {
                const Eigen::Vector3d forward_world =
                    r.pose.linear() *
                    Eigen::Vector3d(gauge_params.forward_axis == argus::ForwardAxis::PosX   ? 1.0
                                    : gauge_params.forward_axis == argus::ForwardAxis::NegX ? -1.0
                                                                                            : 0.0,
                                    gauge_params.forward_axis == argus::ForwardAxis::PosY   ? 1.0
                                    : gauge_params.forward_axis == argus::ForwardAxis::NegY ? -1.0
                                                                                            : 0.0,
                                    0.0);
                std::cout << ','
                          << std::atan2(forward_world.x(), -forward_world.y()) *
                                 (180.0 / 3.14159265358979323846);
            }
            std::cout << '\n';
            continue;
        }
        const argus::RangeImage ri = argus::build_range_image(cloud, ri_params);
        if (!ri.valid()) {
            std::cerr << "frame " << k << ": invalid range image\n";
            continue;
        }
        if (tunnel_mode) {
            const argus::OdometryResult o = odom.update(cloud, f.stamp_s);
            const argus::AnomalySet geom = geometry.detect(cloud, ri);
            const argus::AnomalySet nr = no_return.detect(cloud, ri);
            const bool frozen_ready = frozen_model.ready();
            const argus::AnomalySet fs =
                free_space_freeze
                    ? (frozen_ready && o.valid ? frozen_free_space.detect(cloud, ri, o.pose)
                                               : argus::AnomalySet{})
                    : ((fusion.model().ready() && o.valid) ? free_space.detect(cloud, ri, o.pose)
                                                           : argus::AnomalySet{});
            if (free_space_freeze && k < warmup_frames && o.valid) {
                frozen_model.integrate(cloud, o.pose, false);
            }
            const argus::FusionResult r =
                fusion.update(cloud, ri, geom, nr, fs, 0.1f, 0.0f, o.pose, true, o.valid);
            uint32_t gauge_points = 0;
            uint32_t gauge_prior_covered = 0;
            uint32_t gauge_contradictions = 0;
            float contradiction_nearest = -1.0f;
            if (free_space_freeze && k >= warmup_frames && o.valid) {
                for (size_t i = 0; i < cloud.size(); ++i) {
                    if ((!cloud.ground_mask.empty() && cloud.ground_mask[i]) ||
                        !fusion.gauge().contains(cloud.x[i], cloud.y[i], cloud.z[i])) {
                        continue;
                    }
                    const float forward =
                        fusion.gauge().forward_distance(cloud.x[i], cloud.y[i], cloud.z[i]);
                    if (forward < 15.0f || forward > 100.0f) {
                        continue;
                    }
                    ++gauge_points;
                    float confidence = 0.0f;
                    const Eigen::Vector3f point(cloud.x[i], cloud.y[i], cloud.z[i]);
                    if (frozen_model.violates_free_space(point, confidence, o.pose)) {
                        ++gauge_prior_covered;
                    }
                    if (frozen_model.contradicts_free_space(
                            point, confidence, o.pose, free_space_params.min_confidence,
                            free_space_params.min_free_observations)) {
                        ++gauge_contradictions;
                        contradiction_nearest = contradiction_nearest < 0.0f
                                                    ? forward
                                                    : std::min(contradiction_nearest, forward);
                    }
                }
            }
            uint32_t gauge_observed = 0;
            uint32_t gauge_residual = 0;
            float gauge_peak = 0.0f;
            float gauge_nearest = -1.0f;
            if (profile_gauge_mode && profile.valid()) {
                const auto& gauge = fusion.gauge();
                for (size_t i = 0; i < cloud.size(); ++i) {
                    const uint32_t raw = cloud.raw_idx[i];
                    if (raw >= ri.range.size() ||
                        (!cloud.ground_mask.empty() && cloud.ground_mask[i]) ||
                        !gauge.contains(cloud.x[i], cloud.y[i], cloud.z[i])) {
                        continue;
                    }
                    const float distance =
                        gauge.forward_distance(cloud.x[i], cloud.y[i], cloud.z[i]);
                    if (distance < 15.0f || distance > 100.0f) {
                        continue;
                    }
                    const float deviation = profile.deviation(ri, raw / ri.height, raw % ri.height);
                    if (!std::isfinite(deviation)) {
                        continue;
                    }
                    ++gauge_observed;
                    if (deviation > profile_residual_threshold) {
                        ++gauge_residual;
                        gauge_peak = std::max(gauge_peak, deviation);
                        gauge_nearest =
                            gauge_nearest < 0.0f ? distance : std::min(gauge_nearest, distance);
                    }
                }
            }
            profile.update(ri, !r.alert);
            const float profile_median = profile.median_deviation(ri);
            std::cout << k << ',' << (r.alert ? 1 : 0) << ',' << (r.model_ready ? 1 : 0) << ','
                      << (r.model_updated ? 1 : 0) << ',' << fusion.model().voxel_count() << ','
                      << r.free_space_violation_rate << ',' << fs.indices.size() << ','
                      << profile_median << ',' << (o.valid ? 1 : 0) << ',' << o.speed_mps;
            if (profile_gauge_mode) {
                std::cout << ',' << gauge_observed << ',' << gauge_residual << ',' << gauge_peak
                          << ',' << gauge_nearest;
            }
            if (free_space_freeze) {
                std::cout << ',' << gauge_points << ',' << gauge_prior_covered << ','
                          << gauge_contradictions << ',' << contradiction_nearest;
            }
            std::cout << '\n';
            continue;
        }
        if (fusion_mode) {
            argus::OdometryResult world_odom;
            std::vector<WorldSample> world_samples;
            if (world_residual) {
                world_odom = odom.update(cloud, f.stamp_s);
            }
            argus::AnomalySet geom = (temporal_residual || world_residual)
                                         ? argus::AnomalySet{}
                                         : geometry.detect(cloud, ri);
            if (temporal_residual && !temporal_history.empty()) {
                geom.source = "temporal";
                std::vector<uint32_t> cell_to_cloud(ri.range.size(),
                                                    std::numeric_limits<uint32_t>::max());
                for (size_t i = 0; i < cloud.raw_idx.size(); ++i) {
                    if (cloud.raw_idx[i] < cell_to_cloud.size()) {
                        cell_to_cloud[cloud.raw_idx[i]] = static_cast<uint32_t>(i);
                    }
                }
                for (size_t cell = 0; cell < ri.range.size(); ++cell) {
                    const float current = ri.range[cell];
                    if (!std::isfinite(current) ||
                        cell_to_cloud[cell] == std::numeric_limits<uint32_t>::max()) {
                        continue;
                    }
                    std::vector<float> values;
                    for (const auto& frame : temporal_history) {
                        if (cell < frame.size() && std::isfinite(frame[cell])) {
                            values.push_back(frame[cell]);
                        }
                    }
                    if (values.empty()) {
                        continue;
                    }
                    std::sort(values.begin(), values.end());
                    const float baseline = values[values.size() / 2];
                    if (baseline - current <= temporal_threshold || current < 4.0f ||
                        current > geom_max_target_range) {
                        continue;
                    }
                    geom.indices.push_back(cell_to_cloud[cell]);
                    geom.score.push_back(std::min(1.0f, (baseline - current) / 5.0f));
                }
            }
            if (world_residual && world_odom.valid && world_odom.fitness <= world_fitness &&
                world_odom.n_correspondences >= 1000) {
                geom.source = "world";
                const Eigen::Isometry3d pose = world_odom.pose;
                const float inv = 1.0f / std::max(world_voxel, 0.05f);
                const auto voxel_key = [](int64_t ix, int64_t iy, int64_t iz) {
                    return (ix * 73856093LL) ^ (iy * 19349663LL) ^ (iz * 83492791LL);
                };
                const auto seen_nearby = [&](int64_t ix, int64_t iy, int64_t iz) {
                    for (int64_t dx = -1; dx <= 1; ++dx) {
                        for (int64_t dy = -1; dy <= 1; ++dy) {
                            for (int64_t dz = -1; dz <= 1; ++dz) {
                                if (world_history.find(voxel_key(ix + dx, iy + dy, iz + dz)) !=
                                    world_history.end()) {
                                    return true;
                                }
                            }
                        }
                    }
                    return false;
                };
                std::vector<int64_t> keys;
                keys.reserve(cloud.size());
                std::unordered_set<int64_t> frame_keys;
                for (size_t i = 0; i < cloud.size(); ++i) {
                    const float forward =
                        gauge_params.forward_axis == argus::ForwardAxis::PosX   ? cloud.x[i]
                        : gauge_params.forward_axis == argus::ForwardAxis::NegX ? -cloud.x[i]
                        : gauge_params.forward_axis == argus::ForwardAxis::PosY ? cloud.y[i]
                                                                                : -cloud.y[i];
                    const float lateral =
                        gauge_params.forward_axis == argus::ForwardAxis::PosX   ? cloud.y[i]
                        : gauge_params.forward_axis == argus::ForwardAxis::NegX ? -cloud.y[i]
                        : gauge_params.forward_axis == argus::ForwardAxis::PosY ? -cloud.x[i]
                                                                                : cloud.x[i];
                    const bool track_mode = track_corridor_mode;
                    const float range =
                        std::sqrt(cloud.x[i] * cloud.x[i] + cloud.y[i] * cloud.y[i] +
                                  cloud.z[i] * cloud.z[i]);
                    if ((track_mode ? range : forward) < 15.0f ||
                        (track_mode ? range : forward) > geom_max_target_range ||
                        (!track_mode &&
                         !fusion.gauge().contains(cloud.x[i], cloud.y[i], cloud.z[i])) ||
                        (track_mode && !cloud.ground_mask.empty() && cloud.ground_mask[i])) {
                        continue;
                    }
                    const Eigen::Vector3d local(cloud.x[i], cloud.y[i], cloud.z[i]);
                    const Eigen::Vector3d world = pose * local;
                    const int64_t ix = static_cast<int64_t>(std::floor(world.x() * inv));
                    const int64_t iy = static_cast<int64_t>(std::floor(world.y() * inv));
                    const int64_t iz = static_cast<int64_t>(std::floor(world.z() * inv));
                    const int64_t key = voxel_key(ix, iy, iz);
                    keys.push_back(key);
                    const bool first_in_frame = frame_keys.insert(key).second;
                    if (k >= world_warmup && world_component_mode && !seen_nearby(ix, iy, iz)) {
                        float path_distance = 0.0f;
                        float path_offset = 0.0f;
                        const Eigen::Vector2f sensor = pose.translation().head<2>().cast<float>();
                        const Eigen::Vector2f candidate = world.head<2>().cast<float>();
                        if (!track_corridor_mode ||
                            (track_corridor.contains(sensor, candidate, path_distance,
                                                     path_offset) &&
                             fusion.gauge().contains_at_path(path_distance, path_offset,
                                                             cloud.z[i]))) {
                            world_samples.push_back({world.cast<float>(), track_corridor_mode
                                                                              ? path_distance
                                                                              : forward});
                        }
                    } else if (k >= world_warmup && !world_component_mode && first_in_frame &&
                               !seen_nearby(ix, iy, iz) &&
                               ++world_evidence_counts[key] >= world_evidence) {
                        geom.indices.push_back(static_cast<uint32_t>(i));
                        geom.score.push_back(1.0f);
                    }
                }
                if (k < world_warmup) {
                    world_history.insert(keys.begin(), keys.end());
                }
            }
            if (world_component_mode) {
                const auto components = world_components(world_samples, 0.65f);
                if (world_component_detail) {
                    for (const auto& component : components) {
                        const Eigen::Vector3f extent = component.high - component.low;
                        std::cerr << "component," << k << ',' << component.forward << ','
                                  << component.center.x() << ',' << component.center.y() << ','
                                  << component.center.z() << ',' << component.points << ','
                                  << extent.x() << ',' << extent.y() << ',' << extent.z() << '\n';
                    }
                }
                auto result = update_world_tracks(world_tracks, components, k, 0.9f,
                                                  world_max_component_points, world_max_y_extent,
                                                  world_max_track_age);
                if (track_corridor_mode) {
                    const Eigen::Vector2f sensor =
                        world_odom.pose.translation().head<2>().cast<float>();
                    std::vector<WorldTrack> axis_tracks;
                    for (const auto& track : result.confirmed) {
                        float candidate_arc = 0.0f;
                        float lateral_offset = 0.0f;
                        const Eigen::Vector3d local =
                            world_odom.pose.inverse() * track.center.cast<double>();
                        if (track_corridor.contains(sensor, track.center.head<2>(), candidate_arc,
                                                    lateral_offset) &&
                            (world_max_axis_offset == 0.0f ||
                             lateral_offset <= world_max_axis_offset) &&
                            fusion.gauge().contains_at_path(candidate_arc, lateral_offset,
                                                            static_cast<float>(local.z()))) {
                            axis_tracks.push_back(track);
                        }
                    }
                    result.alert = !axis_tracks.empty();
                    result.confirmed = std::move(axis_tracks);
                    if (result.alert) {
                        const auto& track =
                            *std::min_element(result.confirmed.begin(), result.confirmed.end(),
                                              [](const WorldTrack& a, const WorldTrack& b) {
                                                  return a.forward < b.forward;
                                              });
                        result.forward = track.forward;
                        result.center = track.center;
                        result.extent = track.high - track.low;
                        result.points = track.points;
                        result.hits = track.hits;
                    }
                }
                if (world_component_detail) {
                    for (const auto& track : result.confirmed) {
                        const Eigen::Vector3f extent = track.high - track.low;
                        std::cerr << "confirmed," << k << ',' << track.forward << ','
                                  << track.center.x() << ',' << track.center.y() << ','
                                  << track.center.z() << ',' << track.points << ',' << extent.x()
                                  << ',' << extent.y() << ',' << extent.z() << ',' << track.hits
                                  << '\n';
                    }
                }
                std::cout << k << ',' << (result.alert ? 1 : 0) << ',' << result.components << ','
                          << result.tracks << ',' << result.forward << ',' << world_samples.size()
                          << ',' << (world_odom.valid ? 1 : 0) << ',' << world_odom.fitness << ','
                          << result.center.x() << ',' << result.center.y() << ','
                          << result.center.z() << ',' << result.extent.x() << ','
                          << result.extent.y() << ',' << result.extent.z() << ',' << result.points
                          << ',' << result.hits << ',' << result.oversized << '\n';
                continue;
            }
            const argus::AnomalySet nr = no_return.detect(cloud, ri);
            if (debug_clusters) {
                argus::AnomalySet merged;
                merged.indices = geom.indices;
                argus::ClusteringParams raw_cp;
                raw_cp.min_cluster_size_near = 1;
                raw_cp.min_extent_m = 0.0f;
                raw_cp.max_extent_m = 1e9f;
                const auto raw = argus::cluster_anomalies(cloud, ri, merged, raw_cp);
                std::cerr << "frame " << k << ": anom=" << geom.indices.size()
                          << " raw_components=" << raw.size() << "\n";
                for (const auto& c : raw) {
                    std::cerr << "  n=" << c.point_count << " nearest=" << c.nearest_range
                              << " ext=" << (c.max_corner - c.min_corner).transpose()
                              << " centroid=(" << c.centroid.transpose() << ")\n";
                }
                argus::ClusteringParams cp;
                cp.min_cluster_size_near = fusion_params.clustering.min_cluster_size_near;
                cp.min_cluster_size_far = fusion_params.clustering.min_cluster_size_far;
                cp.min_extent_m = fusion_params.clustering.min_extent_m;
                cp.max_extent_m = fusion_params.clustering.max_extent_m;
                const auto cls = argus::cluster_anomalies(cloud, ri, merged, cp);
                std::cerr << "  после фильтров: " << cls.size() << "\n";
                for (const auto& c : cls) {
                    const Eigen::Vector3f ext = c.max_corner - c.min_corner;
                    uint32_t gauge_points = 0;
                    for (uint32_t idx : c.indices) {
                        if (idx < cloud.size() &&
                            fusion.gauge().contains(cloud.x[idx], cloud.y[idx], cloud.z[idx])) {
                            ++gauge_points;
                        }
                    }
                    std::cerr << "    kept n=" << c.point_count << " nearest=" << c.nearest_range
                              << " gauge=" << gauge_points << " fwd=" << c.forward_distance
                              << " ext=" << ext.transpose() << " centroid=("
                              << c.centroid.transpose() << ")\n";
                }
            }
            const argus::FusionResult r = fusion.update(cloud, ri, geom, nr, 0.1f, 0.0f);
            std::cout << k << ',' << (r.alert ? 1 : 0) << ',' << r.clusters.size() << ','
                      << r.n_clusters_raw << ',' << r.n_filtered_out << ',' << r.tracks.size()
                      << ',' << r.nearest_forward_m << ',' << r.nearest_range_m << ','
                      << r.n_anom_points << ',' << r.n_anom_cells << '\n';
            if (temporal_residual) {
                temporal_history.push_back(ri.range);
                if (temporal_history.size() > temporal_window) {
                    temporal_history.erase(temporal_history.begin());
                }
            }
            continue;
        }
        report(k, no_return.detect(cloud, ri), ri, cloud);
        report(k, geometry.detect(cloud, ri), ri, cloud);
    }
    return 0;
}
