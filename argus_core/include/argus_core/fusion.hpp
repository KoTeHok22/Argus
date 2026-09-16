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
// Конвейер fusion (Фаза 4, PLAN.md §9): кластеризация аномалий ->
// габаритная фильтрация (законная геометрия вне габарита отсеивается) ->
// трекинг (подтверждение по кадрам) -> тревога.
// Голосование: детекторы geometry/no_return коррелированы (PLAN.md §9.4),
// поэтому решение — подтверждённый трек кластера в габарите; счётчики
// голосов заполняются для объяснимости.

#pragma once

#include <cstdint>
#include <vector>

#include "argus_core/clustering.hpp"
#include "argus_core/gauge.hpp"
#include "argus_core/range_image.hpp"
#include "argus_core/tracking.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct FusionParams {
    bool require_confirmed_track = true; // тревога только для подтверждённых треков
    float critical_range_m = 50.0f;
    float warning_range_m = 150.0f;
    float critical_ttc_s = 3.0f;
    // Временная замена Ф1.2.3 (Patchwork++, не реализован): точки путевой
    // структуры (рельсы, контактный рельс, постель) исключаются из
    // кластеризации. Отсекаются ТОЧКИ, не кластеры — объект на путях
    // сохраняет верхние точки и остаётся детектируемым.
    bool ground_filter = true;
    float ground_clearance_m = 0.45f; // z над головкой рельса ниже — путевая структура
    float rail_zone_m = 2.0f; // |латераль| меньше — зона колеи
    ClusteringParams clustering;
    TrackingParams tracking;
};

struct FusionResult {
    std::vector<Cluster> clusters; // кластеры в габарите после фильтрации
    std::vector<Track> tracks; // подтверждённые треки текущего кадра
    uint32_t n_anom_points = 0;  // аномальных точек на входе (geometry)
    uint32_t n_anom_cells = 0;   // аномальных клеток без точек (no_return)
    uint32_t n_clusters_raw = 0; // кластеров до габаритной фильтрации
    uint32_t n_filtered_out = 0; // кластеров, отсеянных габаритом
    bool alert = false;
    float nearest_forward_m = -1.0f; // продольная дистанция ближайшей цели, -1 если нет
    float nearest_range_m = -1.0f;
    float ttc_s = -1.0f;
};

class FusionPipeline {
public:
    FusionPipeline(ClearanceGauge gauge, const FusionParams& params);

    /// Обработать кадр. geometry/no_return — выходы детекторов этого кадра.
    FusionResult update(const CleanCloud& cloud, const RangeImage& ri, const AnomalySet& geometry,
                        const AnomalySet& no_return, float dt, float train_speed_mps);

    const ClearanceGauge& gauge() const { return gauge_; }

private:
    ClearanceGauge gauge_;
    FusionParams p_;
    ObstacleTracker tracker_;
};

} // namespace argus
