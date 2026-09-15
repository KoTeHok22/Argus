// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0

#include "argus_core/range_image.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus {

uint32_t detect_rings(const CleanCloud& cloud, uint32_t max_rings) {
  // Развёртка записана позиционно: ring = i % rings (факт D3).
  // Если в поле ring мало уникальных малых значений, определяем rings
  // как первый повтор первого значения (период развёртки).
  if (cloud.size() < 16) {
    return cloud.rings > 0 ? cloud.rings : 1;
  }

  // Кандидат: период повторения значения ring[0].
  const uint16_t first = cloud.ring[0];
  for (uint32_t period = 1; period <= max_rings && period < cloud.size();
       ++period) {
    if (cloud.ring[period] == first) {
      // Проверяем кандидата на первых 4 периодах.
      const uint32_t check =
          std::min<uint32_t>(4 * period, static_cast<uint32_t>(cloud.size()));
      bool ok = true;
      for (uint32_t i = 0; i < check; ++i) {
        if (cloud.ring[i] != cloud.ring[i % period]) {
          ok = false;
          break;
        }
      }
      if (ok) {
        return period;
      }
    }
  }

  // Fallback: явное поле или дефолт.
  if (cloud.rings > 0) {
    return cloud.rings;
  }
  uint16_t max_ring = 0;
  for (uint16_t r : cloud.ring) {
    max_ring = std::max(max_ring, r);
  }
  return static_cast<uint32_t>(max_ring) + 1;
}

RangeImage build_range_image(const CleanCloud& cloud,
                             const RangeImageParams& params) {
  RangeImage ri;

  uint32_t rings = 0;
  if (params.prefer_ring_field && !cloud.ring.empty()) {
    rings = detect_rings(cloud, params.max_rings);
  }
  if (rings == 0) {
    rings = params.rings_fallback;
  }

  ri.height = rings;
  ri.width = std::max<uint32_t>(1, static_cast<uint32_t>(cloud.size()) / rings);

  const size_t total = static_cast<size_t>(ri.width) * ri.height;
  ri.range.assign(total, std::numeric_limits<float>::quiet_NaN());
  ri.intensity.assign(total, 0.0f);
  ri.no_return_mask.assign(total, 0);

  for (size_t i = 0; i < cloud.size(); ++i) {
    const uint32_t az = static_cast<uint32_t>(i) % ri.width;
    const uint32_t ring = static_cast<uint32_t>(i) / ri.width;
    const size_t idx = ri.idx(az, ring);

    const float x = cloud.x[i], y = cloud.y[i], z = cloud.z[i];
    ri.range[idx] = std::sqrt(x * x + y * y + z * z);
    ri.intensity[idx] = cloud.intensity[i];
  }

  // Точки, отброшенные фильтром как (0,0,0), — это «нет возврата».
  // Индексы не сохранились, поэтому маска достраивается вызывающей стороной
  // из FilterStats.reject_zero при необходимости (Ф3.5.1).
  // Здесь маска остаётся пустой (0) — контракт уточняется в фазе 3.
  return ri;
}

float sample(const RangeImage& ri, int az, int ring) {
  if (az < 0 || ring < 0) return std::numeric_limits<float>::quiet_NaN();
  const auto u = static_cast<uint32_t>(az);
  const auto v = static_cast<uint32_t>(ring);
  if (u >= ri.width || v >= ri.height) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return ri.range[ri.idx(u, v)];
}

uint32_t neighbors8(const RangeImage& ri, uint32_t az, uint32_t ring,
                    std::array<std::pair<uint32_t, uint32_t>, 8>& out) {
  uint32_t n = 0;
  // Цикличность по азимуту: 0 и width-1 — соседи (факт D3, полный оборот).
  const uint32_t w = ri.width;
  const uint32_t az_prev = (az + w - 1) % w;
  const uint32_t az_next = (az + 1) % w;

  for (int dr = -1; dr <= 1; ++dr) {
    const int r = static_cast<int>(ring) + dr;
    if (r < 0 || r >= static_cast<int>(ri.height)) {
      continue;  // вертикальный край: соседей меньше
    }
    const uint32_t rv = static_cast<uint32_t>(r);
    if (dr == 0) {
      out[n++] = {az_prev, rv};
      out[n++] = {az_next, rv};
    } else {
      out[n++] = {az_prev, rv};
      out[n++] = {az, rv};
      out[n++] = {az_next, rv};
    }
  }
  return n;
}

RangeImage azimuth_median_filter(const RangeImage& ri, uint32_t half_window) {
  RangeImage out = ri;
  if (half_window == 0 || ri.width == 0) {
    return out;
  }

  // Медиана по окну азимута для каждого кольца. NaN пропускаются.
  std::vector<float> window;
  for (uint32_t az = 0; az < ri.width; ++az) {
    for (uint32_t ring = 0; ring < ri.height; ++ring) {
      window.clear();
      for (uint32_t d = 0; d <= half_window; ++d) {
        // Окно с обеих сторон, циклично по азимуту.
        const uint32_t a1 = (az + d) % ri.width;
        const uint32_t a2 = (az + ri.width - d) % ri.width;
        for (uint32_t a : {a1, a2}) {
          const float v = ri.range[ri.idx(a, ring)];
          if (std::isfinite(v)) {
            window.push_back(v);
          }
        }
      }
      if (window.empty()) {
        out.range[out.idx(az, ring)] =
            std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      std::sort(window.begin(), window.end());
      out.range[out.idx(az, ring)] = window[window.size() / 2];
    }
  }
  return out;
}

}  // namespace argus
