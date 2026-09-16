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
// Трекинг кластеров: nearest-neighbor ассоциация + Kalman (константная
// скорость). Без трекинга одиночный выброс = ложная тревога, а ложная
// тревога в метро = экстренное торможение (PLAN.md §9.3).

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "argus_core/clustering.hpp"

namespace argus {

struct TrackingParams {
    float max_association_distance = 2.0f; // м между кадрами
    uint32_t min_hits_to_confirm = 3; // кадров подряд для подтверждения
    uint32_t max_misses_to_keep = 4;  // кадров пропуска до сброса
    float process_noise = 0.5f;       // Q
    float measurement_noise = 0.3f;   // R
};

struct Track {
    uint32_t id = 0;
    Eigen::Matrix<float, 6, 1> state = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 6> covariance = Eigen::Matrix<float, 6, 6>::Identity();
    uint32_t hits = 0;
    uint32_t misses = 0;
    bool confirmed = false;

    float nearest_range = 0.0f;
    float ttc = -1.0f; // с; -1 = не применимо (не сближаемся)
    float volume = 0.0f;
    uint8_t votes_free_space = 0;
    uint8_t votes_no_return = 0;
    uint8_t votes_geometry = 0;
};

/// Nearest-neighbor ассоциация + фильтр Калмана на центроиде.
class ObstacleTracker {
public:
    explicit ObstacleTracker(const TrackingParams& p);

    /// Обновить треки кластерами текущего кадра. Вернуть подтверждённые треки.
    /// train_speed_mps — скорость поезда для расчёта TTC.
    std::vector<Track> update(const std::vector<Cluster>& clusters, float dt,
                              float train_speed_mps);

    const std::vector<Track>& all_tracks() const { return tracks_; }

private:
    TrackingParams p_;
    std::vector<Track> tracks_;
    uint32_t next_id_ = 1;
};

/// TTC = продольная дистанция / скорость сближения. -1 если не сближаются.
float compute_ttc(const Track& t, float train_speed_mps);

} // namespace argus
