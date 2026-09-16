// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Офлайн-раннер детекторов (валидация Ф3.5.4): читает бинарный дамп кадров
// ARGFRM1 (scripts/bag2frames.py), строит CleanCloud + RangeImage и прогоняет
// те же детекторы argus_core, что работают в рантайме. Отчёт — CSV в stdout.
//
//   frame,source,points,cells,nearest_m,az_center,az_span,cells_bbox
//
// Формат ARGFRM1 (little-endian): magic "ARGFRM1" | uint32 n_frames;
// на кадр: double stamp_s | uint32 n_raw | n_valid | n_no_return |
//   n_valid x (float x,y,z,intensity; uint32 raw_idx) | n_no_return x uint32.

#include <argus_core/anomaly.hpp>
#include <argus_core/clustering.hpp>
#include <argus_core/fusion.hpp>
#include <argus_core/gauge.hpp>
#include <argus_core/range_image.hpp>
#include <argus_core/types.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
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

    // Компонентная сводка по точкам: собираем клетки, группируем по близости
    // дальности (просто и достаточно для отчёта).
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
    std::string forward_axis = "x";
    uint32_t limit = 0;
    bool fusion_mode = false;
    bool debug_clusters = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--frames" && i + 1 < argc) {
            limit = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (a == "--fusion") {
            fusion_mode = true;
        } else if (a == "--debug-clusters") {
            debug_clusters = true;
        } else if (a == "--forward-axis" && i + 1 < argc) {
            forward_axis = argv[++i];
        } else {
            path = a;
        }
    }
    if (path.empty()) {
        std::cerr << "usage: offline_detector <frames.bin> [--frames N] [--fusion] "
                     "[--forward-axis x|-x|y|-y]\n";
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
    argus::NoReturnDetector no_return(nr_params);
    argus::GeometryResidualDetector geometry(geom_params);
    argus::RangeImageParams ri_params;

    argus::ForwardAxis axis = argus::ForwardAxis::PosX;
    if (!argus::parse_forward_axis(forward_axis, axis)) {
        std::cerr << "bad --forward-axis: " << forward_axis << "\n";
        return 2;
    }
    argus::ClearanceGaugeParams gauge_params;
    gauge_params.forward_axis = axis;
    gauge_params.sensor_height = 1.2f; // допущение A3 (пол ~ -1.2 м в СК сенсора)
    argus::FusionParams fusion_params;
    fusion_params.clustering.min_cluster_size = 15;
    fusion_params.clustering.min_extent_m = 0.1f;
    fusion_params.tracking.min_hits_to_confirm = 3;
    argus::FusionPipeline fusion(argus::ClearanceGauge(gauge_params), fusion_params);

    if (fusion_mode) {
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
        const argus::CleanCloud cloud = to_clean_cloud(f);
        const argus::RangeImage ri = argus::build_range_image(cloud, ri_params);
        if (!ri.valid()) {
            std::cerr << "frame " << k << ": invalid range image\n";
            continue;
        }
        if (fusion_mode) {
            const argus::AnomalySet geom = geometry.detect(cloud, ri);
            const argus::AnomalySet nr = no_return.detect(cloud, ri);
            if (debug_clusters) {
                argus::AnomalySet merged;
                merged.indices = geom.indices;
                argus::ClusteringParams raw_cp;
                raw_cp.min_cluster_size = 1;
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
                cp.min_cluster_size = fusion_params.clustering.min_cluster_size;
                cp.min_extent_m = fusion_params.clustering.min_extent_m;
                cp.max_extent_m = fusion_params.clustering.max_extent_m;
                const auto cls = argus::cluster_anomalies(cloud, ri, merged, cp);
                std::cerr << "  после фильтров: " << cls.size() << "\n";
            }
            const argus::FusionResult r = fusion.update(cloud, ri, geom, nr, 0.1f, 0.0f);
            std::cout << k << ',' << (r.alert ? 1 : 0) << ',' << r.clusters.size() << ','
                      << r.n_clusters_raw << ',' << r.n_filtered_out << ',' << r.tracks.size()
                      << ',' << r.nearest_forward_m << ',' << r.nearest_range_m << ','
                      << r.n_anom_points << ',' << r.n_anom_cells << '\n';
            continue;
        }
        report(k, no_return.detect(cloud, ri), ri, cloud);
        report(k, geometry.detect(cloud, ri), ri, cloud);
    }
    return 0;
}
