
#include <argus_core/anomaly.hpp>
#include <argus_core/clustering.hpp>
#include <argus_core/fusion.hpp>
#include <argus_core/gauge.hpp>
#include <argus_core/ground.hpp>
#include <argus_core/odometry.hpp>
#include <argus_core/range_image.hpp>
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
#include <string>
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
    float gauge_sensor_height = 1.20f;
    float odom_max_range = 120.0f;
    uint32_t odom_iterations = 4;
    uint32_t min_cluster_size_near = 15;
    uint32_t min_cluster_size_mid = 6;
    uint32_t min_cluster_size_far = 3;
    uint32_t min_hits = 3;
    uint32_t min_hits_far = 5;
    float geom_max_target_range = 45.0f;
    bool ground_filter = false;
    bool temporal_residual = false;
    uint32_t temporal_window = 5;
    float temporal_threshold = 1.0f;
    bool world_residual = false;
    float world_voxel = 0.30f;
    float world_fitness = 0.03f;
    uint32_t world_warmup = 10;
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
        } else if (a == "--tunnel") {
            tunnel_mode = true;
        } else if (a == "--debug-clusters") {
            debug_clusters = true;
        } else if (a == "--geom-max-range" && i + 1 < argc) {
            geom_max_target_range = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--ground-filter") {
            ground_filter = true;
        } else if (a == "--temporal-residual") {
            temporal_residual = true;
        } else if (a == "--temporal-window" && i + 1 < argc) {
            temporal_window = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--temporal-threshold" && i + 1 < argc) {
            temporal_threshold = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-residual") {
            world_residual = true;
        } else if (a == "--world-voxel" && i + 1 < argc) {
            world_voxel = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-fitness" && i + 1 < argc) {
            world_fitness = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--world-warmup" && i + 1 < argc) {
            world_warmup = static_cast<uint32_t>(std::max(1, std::atoi(argv[++i])));
        } else if (a == "--forward-axis" && i + 1 < argc) {
            forward_axis = argv[++i];
        } else {
            path = a;
        }
    }
    if (path.empty()) {
        std::cerr << "usage: offline_detector <frames.bin> [--frames N] [--fusion] [--odometry] "
                     "[--tunnel] [--warmup N] [--free-space-vote] [--carve-stride N] "
                     "[--carve-max-range M] [--carve-half-width W] [--forward-axis x|-x|y|-y] "
                     "[--gauge-height M] [--gauge-half-width M] [--gauge-sensor-height M] "
                     "[--odom-max-range M] [--odom-iterations N] [--ground-method patchworkpp|z] "
                     "[--min-cluster-size N] [--min-cluster-size-mid N] "
                     "[--min-cluster-size-far N] [--min-hits N] [--min-hits-far N] "
                     "[--geom-max-range M] [--temporal-residual] [--temporal-window N] "
                     "[--temporal-threshold M] [--world-residual] [--world-voxel M] "
                     "[--world-fitness M] [--world-warmup N]\n";
        return 2;
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
    argus::NoReturnDetector no_return(nr_params);
    argus::GeometryResidualDetector geometry(geom_params);
    argus::RangeImageParams ri_params;
    std::vector<std::vector<float>> temporal_history;
    std::unordered_set<int64_t> world_history;

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
    argus::FusionParams fusion_params;
    fusion_params.clustering.min_cluster_size_near = min_cluster_size_near;
    fusion_params.clustering.min_cluster_size_mid = min_cluster_size_mid;
    fusion_params.clustering.min_cluster_size_far = min_cluster_size_far;
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
    ground_params.sensor_height = 1.2f;
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

    if (tunnel_mode) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "frame,alert,model_ready,model_updated,voxels,fs_rate,fs_points,"
                     "profile_median_m,odom_valid,speed_mps\n";
    } else if (odometry_mode) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "frame,valid,x,y,z,speed,corr,down,fitness\n";
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
                      << r.n_downsampled << ',' << r.fitness << '\n';
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
            const argus::AnomalySet fs = (fusion.model().ready() && o.valid)
                                             ? free_space.detect(cloud, ri, o.pose)
                                             : argus::AnomalySet{};
            const argus::FusionResult r =
                fusion.update(cloud, ri, geom, nr, fs, 0.1f, 0.0f, o.pose, true, o.valid);
            profile.update(ri, !r.alert);
            const float profile_median = profile.median_deviation(ri);
            std::cout << k << ',' << (r.alert ? 1 : 0) << ',' << (r.model_ready ? 1 : 0) << ','
                      << (r.model_updated ? 1 : 0) << ',' << fusion.model().voxel_count() << ','
                      << r.free_space_violation_rate << ',' << fs.indices.size() << ','
                      << profile_median << ',' << (o.valid ? 1 : 0) << ',' << o.speed_mps << '\n';
            continue;
        }
        if (fusion_mode) {
            argus::OdometryResult world_odom;
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
                    const float relative_z = cloud.z[i] + gauge_sensor_height;
                    if (forward < 15.0f || forward > geom_max_target_range ||
                        std::fabs(lateral) > gauge_half_width || relative_z < 0.0f ||
                        relative_z > gauge_height) {
                        continue;
                    }
                    const Eigen::Vector3d local(cloud.x[i], cloud.y[i], cloud.z[i]);
                    const Eigen::Vector3d world = pose * local;
                    const int64_t ix = static_cast<int64_t>(std::floor(world.x() * inv));
                    const int64_t iy = static_cast<int64_t>(std::floor(world.y() * inv));
                    const int64_t iz = static_cast<int64_t>(std::floor(world.z() * inv));
                    const int64_t key = voxel_key(ix, iy, iz);
                    keys.push_back(key);
                    if (k >= world_warmup && !seen_nearby(ix, iy, iz)) {
                        geom.indices.push_back(static_cast<uint32_t>(i));
                        geom.score.push_back(1.0f);
                    }
                }
                world_history.insert(keys.begin(), keys.end());
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
                    std::cerr << "    kept n=" << c.point_count << " nearest=" << c.nearest_range
                              << " fwd=" << c.forward_distance << " ext=" << ext.transpose()
                              << " centroid=(" << c.centroid.transpose() << ")\n";
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
