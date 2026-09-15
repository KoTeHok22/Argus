// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Range image: представление развёртки 2400x128 (факт D3: ring = i mod 128).
// Главное преимущество: O(1) доступ к соседям вместо O(log N) в KD-tree.

#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include "argus_core/types.hpp"

namespace argus {

struct RangeImageParams {
  uint32_t rings_fallback = 128;      // если ring-поле отсутствует
  uint32_t az_steps_fallback = 2400;  // 307200 / 128
  bool prefer_ring_field = true;
  uint32_t max_rings = 256;
};

/// Построить range image. Один проход O(N).
/// ring = i % rings, az = i / rings (факт D3).
/// no_return_mask заполняется из точек, отброшенных фильтром как (0,0,0).
RangeImage build_range_image(const CleanCloud& cloud,
                             const RangeImageParams& params);

/// Значение дальности; NaN если вне границ.
float sample(const RangeImage& ri, int az, int ring);

/// Соседи по 8-связности с цикличностью по азимуту (0 и az_steps-1 соседи).
/// Возвращает число реальных соседей (меньше 8 у вертикальных краёв).
uint32_t neighbors8(const RangeImage& ri, uint32_t az, uint32_t ring,
                    std::array<std::pair<uint32_t, uint32_t>, 8>& out);

/// Медианный фильтр по азимуту — основа вычитания гладкого фона тоннеля
/// (профиль стен, факт D5/D6 в ИССЛЕДОВАНИЕ.md §1.6.3).
RangeImage azimuth_median_filter(const RangeImage& ri, uint32_t half_window);

/// Автоопределение числа колец: кольцо = индекс i mod rings.
/// Ищет максимальный монотонно-нарастающий префикс ring-поля.
uint32_t detect_rings(const CleanCloud& cloud, uint32_t max_rings);

}  // namespace argus
