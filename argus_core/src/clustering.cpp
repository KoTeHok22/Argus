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
// Кластеризация аномалий на range image (Фаза 4). AnomalySet.indices —
// индексы точек в CleanCloud (см. types.hpp); связность ищется по клеткам
// развёртки (az*height+ring), в которые эти точки отображаются.

#include "argus_core/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace argus {
namespace {

constexpr uint32_t kNoCloudIndex = std::numeric_limits<uint32_t>::max();

} // namespace

float neighbor_distance_at(float base, float d, float ref, bool adaptive) {
    if (!adaptive) {
        return base;
    }
    return base * std::max(1.0f, d / std::max(0.001f, ref));
}

std::vector<Cluster> cluster_anomalies(const CleanCloud& cloud, const RangeImage& ri,
                                       const AnomalySet& merged, const ClusteringParams& p) {
    std::vector<Cluster> out;
    if (merged.indices.empty() || !ri.valid() || cloud.empty()) {
        return out;
    }

    const size_t n_cells = ri.range.size();

    // Карта клетка развёртки -> точка в cloud. Клетка (az, ring) соответствует
    // сырому лучу src = az*height + ring (build_range_image, факт D3), поэтому
    // клетка и сырой индекс совпадают. Перезапись последней точкой — та же
    // семантика, что и в build_range_image.
    std::vector<uint32_t> cell_to_cloud(n_cells, kNoCloudIndex);
    const bool has_raw = !cloud.raw_idx.empty();
    for (size_t i = 0; i < cloud.size(); ++i) {
        const uint32_t src = has_raw ? cloud.raw_idx[i] : static_cast<uint32_t>(i);
        if (src < n_cells) {
            cell_to_cloud[src] = static_cast<uint32_t>(i);
        }
    }

    // Аномальные клетки по индексам CleanCloud.
    std::vector<uint8_t> anomalous(n_cells, 0);
    for (uint32_t idx : merged.indices) {
        if (idx >= cloud.size()) {
            continue;
        }
        const uint32_t src = has_raw ? cloud.raw_idx[idx] : idx;
        if (src < n_cells) {
            anomalous[src] = 1;
        }
    }

    std::vector<uint8_t> visited(n_cells, 0);
    std::vector<uint32_t> stack;

    for (uint32_t idx : merged.indices) {
        if (idx >= cloud.size()) {
            continue;
        }
        const uint32_t seed_src = has_raw ? cloud.raw_idx[idx] : idx;
        if (seed_src >= n_cells || visited[seed_src] || !anomalous[seed_src]) {
            continue;
        }

        Cluster c;
        std::vector<uint32_t> member_cells;
        stack.clear();
        stack.push_back(seed_src);
        visited[seed_src] = 1;

        while (!stack.empty()) {
            const uint32_t cur = stack.back();
            stack.pop_back();
            member_cells.push_back(cur);
            const uint32_t az = cur / ri.height;
            const uint32_t ring = cur % ri.height;

            const uint32_t cur_ci = cell_to_cloud[cur];
            if (cur_ci == kNoCloudIndex || cur_ci >= cloud.size()) {
                continue;
            }
            const Eigen::Vector3f p_cur(cloud.x[cur_ci], cloud.y[cur_ci], cloud.z[cur_ci]);
            const float cur_range = p_cur.norm();

            std::array<std::pair<uint32_t, uint32_t>, 8> nb{};
            const uint32_t nn = neighbors8(ri, az, ring, nb);
            for (uint32_t k = 0; k < nn; ++k) {
                const uint32_t nidx = ri.idx(nb[k].first, nb[k].second);
                if (visited[nidx] || !anomalous[nidx]) {
                    continue;
                }

                // Связность по фактической 3D-дистанции между точками
                // (PLAN.md §9.2): разность дальностей ломается на косых
                // поверхностях, где соседние лучи далеко друг от друга
                // по дальности, но близко в пространстве.
                const uint32_t nb_ci = cell_to_cloud[nidx];
                if (nb_ci == kNoCloudIndex || nb_ci >= cloud.size()) {
                    continue;
                }
                const Eigen::Vector3f p_nb(cloud.x[nb_ci], cloud.y[nb_ci], cloud.z[nb_ci]);
                const float thr = neighbor_distance_at(p.neighbor_distance, cur_range,
                                                       p.reference_range, p.adaptive_scaling);
                if ((p_nb - p_cur).norm() > thr) {
                    continue;
                }

                visited[nidx] = 1;
                stack.push_back(nidx);
            }
        }

        // Метрики кластера в декартовых координатах по точкам клеток.
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        c.min_corner.setConstant(std::numeric_limits<float>::max());
        c.max_corner.setConstant(-std::numeric_limits<float>::max());
        c.nearest_range = std::numeric_limits<float>::max();

        for (uint32_t cell : member_cells) {
            const uint32_t ci = cell_to_cloud[cell];
            if (ci == kNoCloudIndex || ci >= cloud.size()) {
                continue;
            }
            c.indices.push_back(ci);
            const Eigen::Vector3f pt(cloud.x[ci], cloud.y[ci], cloud.z[ci]);
            sum += pt;
            c.min_corner = c.min_corner.cwiseMin(pt);
            c.max_corner = c.max_corner.cwiseMax(pt);
            c.nearest_range = std::min(c.nearest_range, pt.norm());
            c.point_count++;
        }

        if (c.point_count < p.min_cluster_size || c.point_count > p.max_cluster_size) {
            continue;
        }
        c.centroid = sum / static_cast<float>(c.point_count);

        const Eigen::Vector3f ext = c.max_corner - c.min_corner;
        // «Физический размер» кластера = максимальная ось бокса: тонкие
        // поверхности (кромка объекта, стенка) легитимны, фильтр должен
        // отсекать точечный шум и «архитектуру», а не плоские кластеры.
        const float max_ext = ext.maxCoeff();
        if (max_ext < p.min_extent_m || max_ext > p.max_extent_m) {
            continue;
        }
        c.volume = ext.x() * ext.y() * ext.z();
        out.push_back(std::move(c));
    }

    return out;
}

} // namespace argus
