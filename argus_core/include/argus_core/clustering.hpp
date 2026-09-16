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
// Кластеризация аномальных точек на range image (связные компоненты).
// O(N) вместо O(N log N) для Euclidean clustering через KD-tree.
// Порог связности адаптируется по дальности: на 200 м луч шире, чем на 20 м.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "argus_core/range_image.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct ClusteringParams {
    float neighbor_distance = 0.35f; // м, базовый порог связности
    bool adaptive_scaling = true;    // порог растёт с дальностью
    float reference_range = 50.0f;   // м, точка отсчёта адаптации
    uint32_t min_cluster_size = 15;
    uint32_t max_cluster_size = 20000;
    float min_extent_m = 0.20f; // меньше — шум
    float max_extent_m = 5.00f; // больше — архитектура, не объект
};

struct Cluster {
    std::vector<uint32_t> indices; // индексы в CleanCloud
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_corner = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_corner = Eigen::Vector3f::Zero();
    float nearest_range = 0.0f; // м, ближайшая к лидару точка кластера
    float volume = 0.0f;        // м^3 по ограничивающему боксу
    float score = 0.0f;         // агрегированная уверенность
    uint32_t point_count = 0;

    // Голоса детекторов (fusion, PLAN.md §9.4)
    uint8_t votes_free_space = 0;
    uint8_t votes_no_return = 0;
    uint8_t votes_geometry = 0;
};

/// Адаптивный порог связности: neighbor_distance(d) = base * max(1, d/ref).
float neighbor_distance_at(float base, float d, float ref, bool adaptive);

/// Связные компоненты аномальных точек на range image + пересчёт метрик.
/// merged.indices указывают на точки в cloud; связность — по соседству
/// в range image, если фактическая дистанция между точками < порога.
std::vector<Cluster> cluster_anomalies(const CleanCloud& cloud, const RangeImage& ri,
                                       const AnomalySet& merged, const ClusteringParams& p);

} // namespace argus
