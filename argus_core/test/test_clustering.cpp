// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Тесты кластеризации: два объекта не сливаются, разрыв не распадает,
// min_cluster_size работает.

#include <gtest/gtest.h>

#include <cmath>

#include "argus_core/clustering.hpp"

namespace {

argus::RangeImage make_ri(uint32_t width, uint32_t height) {
  argus::RangeImage ri;
  ri.width = width;
  ri.height = height;
  ri.range.assign(static_cast<size_t>(width) * height,
                  std::numeric_limits<float>::quiet_NaN());
  ri.intensity.assign(static_cast<size_t>(width) * height, 0.0f);
  ri.no_return_mask.assign(static_cast<size_t>(width) * height, 0);
  return ri;
}

}  // namespace

TEST(Clustering, AdaptiveThresholdGrowsWithRange) {
  const float base = argus::neighbor_distance_at(0.35f, 20.0f, 50.0f, true);
  const float far = argus::neighbor_distance_at(0.35f, 200.0f, 50.0f, true);
  EXPECT_FLOAT_EQ(base, 0.35f);          // ближе reference — без роста
  EXPECT_GT(far, base);                  // дальше reference — порог растёт
  const float fixed = argus::neighbor_distance_at(0.35f, 200.0f, 50.0f, false);
  EXPECT_FLOAT_EQ(fixed, 0.35f);         // без адаптации порог константен
}

TEST(Clustering, EmptyInputEmptyOutput) {
  auto ri = make_ri(100, 16);
  argus::CleanCloud cloud;
  argus::AnomalySet merged;
  argus::ClusteringParams p;
  EXPECT_TRUE(argus::cluster_anomalies(cloud, ri, merged, p).empty());
}

// Полные сценарии слияния/разделения — Ф4.7.2, после того как детекторы
// начнут возвращать реальные AnomalySet с позициями развёртки.
