// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Фильтрация PointCloud2 и построение CleanCloud.
// Факты данных (PLAN.md §2.1):
//   D1: point_step=26, поля x@0 y@4 z@8 intensity@12 ring@16 timestamp@18 —
//       но ВСЕГДА читать fields[] динамически.
//   D4: ~70% точек мусор (|coord|>1e4), 6-10% ровно (0,0,0).
//   D2: SDK-структура LidarPointXYZIRT = 23 байта — НЕ совпадает с раскладкой
//       ROS-сообщения. sizeof(LidarPointXYZIRT) в этом коде запрещён.

#pragma once

#include <optional>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "argus_core/types.hpp"

namespace argus {

/// Раскладка полей, извлечённая ИЗ сообщения, а не из догадок.
struct CloudLayout {
  uint32_t point_step = 0;
  uint32_t offset_x = 0, offset_y = 0, offset_z = 0;
  uint32_t offset_intensity = 0;
  uint32_t offset_ring = 0;
  uint32_t offset_timestamp = 0;
  bool has_intensity = false;
  bool has_ring = false;
  bool has_timestamp = false;
  uint32_t n_points = 0;       // после проверки границ буфера
  std::string error;           // заполнено, если раскладка отвергнута
};

/// Распознать раскладку полей. Возвращает nullopt при неизвестной раскладке.
/// Никогда не угадывает offsets.
std::optional<CloudLayout> parse_layout(
    const sensor_msgs::msg::PointCloud2& msg);

/// Параметры sanity-фильтра.
struct FilterParams {
  float min_range = 0.5f;        // м
  float max_range = 400.0f;      // м
  float max_abs_coord = 1.0e4f;  // отбраковка мусора (факт D4)
  bool drop_zero_xyz = true;     // (0,0,0) = нет возврата
  bool drop_nonfinite = true;
};

/// Счётчики отбраковки для диагностики (поле Diagnostics.msg).
struct FilterStats {
  uint32_t n_raw = 0;
  uint32_t n_valid = 0;
  uint32_t n_rejected = 0;
  uint32_t reject_zero = 0;
  uint32_t reject_nonfinite = 0;
  uint32_t reject_out_of_range = 0;
};

/// Главная функция слоя данных.
/// Гарантии: все выходные точки конечны, |coord| <= max_abs_coord,
/// min_range <= |p| <= max_range, ring < rings, az_idx < az_steps.
/// ring/azimuth определяются позиционно: ring = i % rings, az = i / rings.
CleanCloud filter_and_index(
    const sensor_msgs::msg::PointCloud2& msg,
    const CloudLayout& layout,
    const FilterParams& params,
    FilterStats* stats = nullptr);

}  // namespace argus
