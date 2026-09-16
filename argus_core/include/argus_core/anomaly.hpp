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
// Детекторы аномалий. Три независимых признака (PLAN.md §8):
//   A. free_space — точка попала в клетку, надёжно наблюдавшуюся пустой.
//   B. no_return  — пропали возвраты там, где стена стабильно отражает.
//   C. geometry   — отклонение от гладкого профиля тоннеля.
// Голосование трёх разных по природе признаков (PLAN.md §9.4).

#pragma once

#include <memory>
#include <string>

#include "argus_core/range_image.hpp"
#include "argus_core/types.hpp"

namespace argus {

/// Базовый интерфейс детектора. Каждый детектор независим.
class IAnomalyDetector {
public:
    virtual ~IAnomalyDetector() = default;

    /// Обработать кадр: индексы аномальных точек в cloud + уверенность.
    virtual AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) = 0;

    /// Обновить внутреннее состояние (для детекторов с историей).
    virtual void update(const CleanCloud& cloud, const RangeImage& ri) {
        (void)cloud;
        (void)ri;
    }

    virtual std::string name() const = 0;
};

// --- Детектор B: потеря возвратов (не зависит ни от чего, кроме RangeImage) ---

struct NoReturnParams {
    uint32_t min_missing_run = 8; // минимум подряд идущих «дыр» по азимуту
    float max_missing_run_deg = 15.0f; // тень объекта компактна; длинные серии
                                       // — слепые секторы развёртки, не тревожат
    uint32_t azimuth_window = 5; // по скольким столбцам судить о норме
    float min_baseline_hits = 0.70f; // доля возвратов в окне = норма
    float min_range = 3.0f;          // м, ближе — не рассматриваем
};

/// Детектор потери возвратов. Реализовать в Ф3.5.1 (первый по плану).
class NoReturnDetector : public IAnomalyDetector {
public:
    explicit NoReturnDetector(const NoReturnParams& p);
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) override;
    std::string name() const override { return "no_return"; }

private:
    NoReturnParams p_;
};

// --- Детектор C: геометрический остаток (пороги-якоря проверены офлайн) ---

struct GeometryResidualParams {
    uint32_t median_half_window = 100; // окно скользящей медианы = 2*100+1 = 201 az
    float residual_threshold_m = 1.0f; // клетка ближе базовой линии на > порога
    uint32_t min_cells = 50; // минимум клеток в связной компоненте
    float min_fill = 0.3f;   // плотность заполнения bbox компоненты
    float min_range = 4.0f; // отсекать близкие возвраты пола/потолка
    float max_target_range_m = 45.0f; // цель ближе этой дальности (по F-D)
    // Дискриминатор F-E: транзиентные срабатывания (край платформы при
    // движении) не накапливают серию; стоящий объект стабилен по кадрам.
    uint32_t min_stable_frames = 2; // 1 = без проверки стабильности
    uint32_t az_tolerance = 20;     // допуск центра по азимуту, клеток
    float range_tolerance_m = 3.0f; // допуск ближайшей дальности, м
};

/// Детектор отклонения от гладкого профиля тоннеля. Ф3.5.2.
/// Локальная скользящая медиана по азимуту (глобальная кольцевая медиана
/// не работает — массовые ложные срабатывания на платформе, см. PROJECT_STATUS
/// §7.2). Компоненты отфильтровываются по плотности, дальности и стабильности.
class GeometryResidualDetector : public IAnomalyDetector {
public:
    explicit GeometryResidualDetector(const GeometryResidualParams& p);
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) override;
    std::string name() const override { return "geometry"; }

private:
    struct Track {
        float az_center = 0.0f;
        float nearest = 0.0f;
        uint32_t streak = 0;
    };

    GeometryResidualParams p_;
    std::vector<Track> tracks_; // кандидаты предыдущего кадра
};

// --- Детектор A: нарушение свободного пространства (нужна модель тоннеля) ---

struct FreeSpaceParams {
    float min_confidence = 0.70f; // порог уверенности «клетка пуста»
    float min_range = 1.0f;
    float max_range = 150.0f; // дальше разреженность делает признак шумным
    bool require_multiple_observations = true;
};

/// Форвард-декларация: модель тоннеля реализуется в Ф2.3.2.
class TunnelModel;

/// Детектор «точка в ранее пустом». Ф3.5.3.
class FreeSpaceDetector : public IAnomalyDetector {
public:
    FreeSpaceDetector(TunnelModel* model, const FreeSpaceParams& p);
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) override;
    std::string name() const override { return "free_space"; }

private:
    TunnelModel* model_; // не владеем
    FreeSpaceParams p_;
};

} // namespace argus
