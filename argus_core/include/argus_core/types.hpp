// Copyright 2026 Argus Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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
  std::vector<float> x, y, z;          // метры, frame_id лидара
  std::vector<float> intensity;        // как в сообщении
  std::vector<uint16_t> ring;          // 0..(rings-1)
  std::vector<uint32_t> azimuth_idx;   // 0..(az_steps-1)

  uint32_t rings = 0;                  // число колец (обычно 128)
  uint32_t az_steps = 0;               // азимутальных шагов (2400 или 7200)
  uint8_t echo_multiplier = 1;         // 1 обычный режим, 3 тройное эхо
  double stamp_s = 0.0;                // время кадра, секунды
  std::string frame_id;

  size_t size() const { return x.size(); }
  bool empty() const { return x.empty(); }
};

/// Range image: 2D-представление развёртки лидара.
/// NaN в range означает «нет возврата».
struct RangeImage {
  uint32_t width = 0;                  // az_steps
  uint32_t height = 0;                 // rings
  std::vector<float> range;            // метры; NaN = нет возврата
  std::vector<float> intensity;
  /// Маска «луч не вернулся вообще» — заполняется ДО sanity-фильтра,
  /// потому что ровно (0,0,0) несут признак для детектора потери возвратов.
  std::vector<uint8_t> no_return_mask;

  /// Единственно допустимый способ индексации (PLAN.md §0.5, инвариант 3).
  size_t idx(uint32_t az, uint32_t ring) const {
    return static_cast<size_t>(az) * height + ring;
  }

  bool valid() const {
    return width > 0 && height > 0 &&
           range.size() == static_cast<size_t>(width) * height &&
           no_return_mask.size() == range.size();
  }
};

/// Результат работы одного детектора аномалий.
struct AnomalySet {
  std::vector<uint32_t> indices;       // индексы в CleanCloud
  std::vector<float> score;            // 0..1, уверенность признака
  std::string source;                  // "free_space" | "no_return" | "geometry"
};

/// Габарит движения поезда как твёрдое тело (усечённая призма вдоль оси X).
struct ClearanceGauge {
  float half_width = 1.45f;            // м, половина ширины
  float height = 3.60f;                // м, от головки рельса
  float nose_offset = 0.0f;            // м, от лидара до носа поезда
  float chamfer = 0.20f;               // м, фаска по углам
  float max_range = 300.0f;            // м, предел проверки

  /// Точка внутри габарита? X — вперёд, Y — влево, Z — вверх.
  bool contains(float x, float y, float z) const;
};

}  // namespace argus
