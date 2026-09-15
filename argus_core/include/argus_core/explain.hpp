// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Лог объяснимости (PLAN.md §9.5). Требование ТЗ — понятность алгоритма.
// Формат строки стабилизирован и покрыт регрессионным тестом.

#pragma once

#include <string>

#include "argus_core/clustering.hpp"
#include "argus_core/tracking.hpp"

namespace argus {

/// Причины срабатывания для одной тревоги.
struct AlertReason {
  float free_space_confidence = -1.0f;  // -1 = детектор не сработал
  float geometry_deviation_m = -1.0f;
  float geometry_sigma_m = -1.0f;
  uint8_t votes_free_space = 0;
  uint8_t votes_no_return = 0;
  uint8_t votes_geometry = 0;
  uint8_t total_votes = 0;              // из 3
};

/// Стабильный формат записи тревоги. Менять только вместе с тестом.
std::string format_alert(const Track& track, const AlertReason& reason);

/// Формат строки: никогда не меняется в горячем цикле, только на событие.
std::string format_clear(uint64_t frames_processed, float fps);

}  // namespace argus
