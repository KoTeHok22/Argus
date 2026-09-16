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
// Базовые типы данных Argus. Единый контракт между всеми модулями.
// Инварианты (проверяются тестами, см. PLAN.md §0.5):
//   1. Все координаты в CleanCloud конечны.
//   2. ring < rings и az_idx < az_steps для каждой точки.
//   3. Индекс в RangeImage всегда az * height + ring.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace argus {

/// Облако после sanity-фильтра. SoA-разметка для векторизации.
struct CleanCloud {
    std::vector<float> x, y, z;        // метры, frame_id лидара
    std::vector<float> intensity;      // как в сообщении
    std::vector<uint16_t> ring;        // 0..(rings-1)
    std::vector<uint32_t> azimuth_idx; // 0..(az_steps-1)

    /// Индекс выжившей точки в сыром буфере кадра; параллелен x/y/z.
    /// Пустой = отбраковки не было, сырой индекс совпадает с i.
    std::vector<uint32_t> raw_idx;
    /// Сырые индексы лучей, вернувших ровно (0,0,0), — «нет возврата».
    std::vector<uint32_t> no_return_raw;

    uint32_t n_raw = 0; // размер сырого буфера; 0 = отбраковки не было
    uint32_t rings = 0;          // число колец (обычно 128)
    uint32_t az_steps = 0;       // азимутальных шагов (2400 или 7200)
    uint8_t echo_multiplier = 1; // 1 обычный режим
    double stamp_s = 0.0;        // время кадра, секунды
    std::string frame_id;

    size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
};

/// Range image: 2D-представление развёртки лидара.
/// NaN в range означает «нет возврата».
struct RangeImage {
    uint32_t width = 0;       // az_steps
    uint32_t height = 0;      // rings
    std::vector<float> range; // метры; NaN = нет возврата
    std::vector<float> intensity;
    /// Маска «луч не вернулся вообще»: клетка развёртки, чей сырой луч
    /// вернул ровно (0,0,0). Заполняется в build_range_image из
    /// CleanCloud.no_return_raw (признак для детектора потери возвратов).
    std::vector<uint8_t> no_return_mask;

    /// Единственно допустимый способ индексации (PLAN.md §0.5, инвариант 3).
    size_t idx(uint32_t az, uint32_t ring) const { return static_cast<size_t>(az) * height + ring; }

    bool valid() const {
        return width > 0 && height > 0 && range.size() == static_cast<size_t>(width) * height &&
               no_return_mask.size() == range.size();
    }
};

/// Результат работы одного детектора аномалий.
struct AnomalySet {
    std::vector<uint32_t> indices; // индексы в CleanCloud
    std::vector<float> score;      // 0..1, уверенность признака
    std::string source;            // "free_space" | "no_return" | "geometry"
};

// Габарит движения поезда: класс ClearanceGauge и ClearanceGaugeParams
// живут в argus_core/gauge.hpp (параметризованная версия).

} // namespace argus
