// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Реализации облачного фильтра. См. cloud_filter.hpp для контрактов.

#include "argus_core/cloud_filter.hpp"

#include <cstring>

#include <sensor_msgs/msg/point_field.hpp>

namespace argus {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace {

/// Найти поле по имени. Вернуть nullptr если отсутствует.
const PointField* find_field(const PointCloud2& msg, const std::string& name) {
  for (const auto& f : msg.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

/// Поле float32 нужного размера, иначе nullptr.
const PointField* find_float32(const PointCloud2& msg, const std::string& name) {
  const PointField* f = find_field(msg, name);
  if (f != nullptr && f->datatype == PointField::FLOAT32 && f->count >= 1) {
    return f;
  }
  return nullptr;
}

}  // namespace

std::optional<CloudLayout> parse_layout(const PointCloud2& msg) {
  CloudLayout out;
  out.point_step = msg.point_step;

  if (msg.point_step == 0) {
    out.error = "point_step == 0";
    return std::nullopt;
  }

  const PointField* x = find_float32(msg, "x");
  const PointField* y = find_float32(msg, "y");
  const PointField* z = find_float32(msg, "z");
  if (x == nullptr || y == nullptr || z == nullptr) {
    out.error = "missing x/y/z float32 fields";
    return std::nullopt;
  }
  out.offset_x = x->offset;
  out.offset_y = y->offset;
  out.offset_z = z->offset;

  if (const PointField* i = find_float32(msg, "intensity")) {
    out.has_intensity = true;
    out.offset_intensity = i->offset;
  }

  if (const PointField* r = find_field(msg, "ring");
      r != nullptr && r->datatype == PointField::UINT16 && r->count >= 1) {
    out.has_ring = true;
    out.offset_ring = r->offset;
  }

  if (const PointField* t = find_field(msg, "timestamp");
      t != nullptr && t->datatype == PointField::FLOAT64 && t->count >= 1) {
    out.has_timestamp = true;
    out.offset_timestamp = t->offset;
  }

  // Границы буфера: ширина * высота * point_step должна помещаться в data.
  const uint64_t declared =
      static_cast<uint64_t>(msg.width) * msg.height * msg.point_step;
  if (declared > msg.data.size()) {
    out.error = "data buffer smaller than width*height*point_step";
    return std::nullopt;
  }
  out.n_points = static_cast<uint32_t>(msg.width) * msg.height;

  // Известные раскладки (факт D1): point_step 26 — hesai, либо любая
  // согласованная раскладка. Угадывание запрещено: всё, что нужно,
  // уже прочитано из fields[].
  return out;
}

CleanCloud filter_and_index(const PointCloud2& msg, const CloudLayout& layout,
                            const FilterParams& params, FilterStats* stats) {
  FilterStats local;
  FilterStats& s = (stats != nullptr) ? *stats : local;
  s = FilterStats{};

  CleanCloud out;
  out.stamp_s = static_cast<double>(msg.header.stamp.sec) +
                static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
  out.frame_id = msg.header.frame_id;

  const size_t n = layout.n_points;
  out.x.reserve(n);
  out.y.reserve(n);
  out.z.reserve(n);
  out.intensity.reserve(n);
  out.ring.reserve(n);
  out.azimuth_idx.reserve(n);

  const uint8_t* base = msg.data.data();
  const uint32_t ps = layout.point_step;

  for (size_t i = 0; i < n; ++i) {
    ++s.n_raw;
    const uint8_t* p = base + i * ps;

    float x = 0.0f, y = 0.0f, z = 0.0f;
    std::memcpy(&x, p + layout.offset_x, sizeof(float));
    std::memcpy(&y, p + layout.offset_y, sizeof(float));
    std::memcpy(&z, p + layout.offset_z, sizeof(float));

    // Отбраковка NaN/Inf (факт D4).
    if (params.drop_nonfinite &&
        (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))) {
      ++s.reject_nonfinite;
      ++s.n_rejected;
      continue;
    }

    // Отбраковка «нет возврата» (факт D4). ВАЖНО: эти точки также
    // помечаются в no_return_mask детектором потери возвратов.
    if (params.drop_zero_xyz && x == 0.0f && y == 0.0f && z == 0.0f) {
      ++s.reject_zero;
      ++s.n_rejected;
      continue;
    }

    // Отбраковка мусора: |coord| > 1e4 (факт D4).
    if (std::abs(x) > params.max_abs_coord ||
        std::abs(y) > params.max_abs_coord ||
        std::abs(z) > params.max_abs_coord) {
      ++s.reject_out_of_range;
      ++s.n_rejected;
      continue;
    }

    const float d2 = x * x + y * y + z * z;
    const float min2 = params.min_range * params.min_range;
    const float max2 = params.max_range * params.max_range;
    if (d2 < min2 || d2 > max2) {
      ++s.reject_out_of_range;
      ++s.n_rejected;
      continue;
    }

    // Позиционная разметка развёртки: ring = i % rings, az = i / rings.
    // Число колец уточняется вызывающей стороной через detect_rings().
    // Здесь az_steps временно 0 — индексы достроятся в build_range_image.
    out.x.push_back(x);
    out.y.push_back(y);
    out.z.push_back(z);

    float intensity = 0.0f;
    if (layout.has_intensity) {
      std::memcpy(&intensity, p + layout.offset_intensity, sizeof(float));
    }
    out.intensity.push_back(intensity);

    uint16_t ring = 0;
    if (layout.has_ring) {
      std::memcpy(&ring, p + layout.offset_ring, sizeof(uint16_t));
    }
    out.ring.push_back(ring);
  }

  s.n_valid = static_cast<uint32_t>(out.size());
  return out;
}

}  // namespace argus
