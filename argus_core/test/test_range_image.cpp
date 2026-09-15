// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Тесты range image: форма, порядок развёртки ring=i%128, NaN, цикличность.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "argus_core/range_image.hpp"

namespace {

argus::CleanCloud make_cloud(size_t n_points, uint32_t rings) {
  argus::CleanCloud c;
  c.rings = rings;
  for (size_t i = 0; i < n_points; ++i) {
    const float d = 10.0f + static_cast<float>(i % 7);
    c.x.push_back(d);
    c.y.push_back(0.0f);
    c.z.push_back(0.0f);
    c.intensity.push_back(1.0f);
    c.ring.push_back(static_cast<uint16_t>(i % rings));
    c.azimuth_idx.push_back(static_cast<uint32_t>(i / rings));
  }
  return c;
}

}  // namespace

TEST(RangeImage, Shape) {
  const uint32_t rings = 4;
  auto cloud = make_cloud(4 * 10, rings);  // 10 азимутов x 4 кольца
  argus::RangeImageParams p;
  p.rings_fallback = rings;
  p.az_steps_fallback = 10;

  const argus::RangeImage ri = argus::build_range_image(cloud, p);
  EXPECT_EQ(ri.height, rings);
  EXPECT_EQ(ri.width, 10u);
  EXPECT_EQ(ri.range.size(), static_cast<size_t>(ri.width) * ri.height);
  EXPECT_TRUE(ri.valid());
}

TEST(RangeImage, OrderingRingEqualsIModRings) {
  const uint32_t rings = 8;
  auto cloud = make_cloud(rings * 5, rings);
  argus::RangeImageParams p;
  p.rings_fallback = rings;

  const argus::RangeImage ri = argus::build_range_image(cloud, p);
  // ring = i % rings (факт D3): точка i=0 (ring 0) и i=8 (ring 0)
  // попадают в один и тот же ring-индекс разных азимутов.
  for (uint32_t az = 0; az < ri.width; ++az) {
    for (uint32_t ring = 0; ring < ri.height; ++ring) {
      const size_t i = ri.idx(az, ring);
      const size_t src = static_cast<size_t>(az) * rings + ring;
      const float expected = cloud.x[src];  // d = 10 + (src % 7)
      EXPECT_NEAR(ri.range[i], expected, 1e-4f);
    }
  }
}

TEST(RangeImage, DetectRingsFindsPeriod) {
  const uint32_t rings = 16;
  auto cloud = make_cloud(rings * 4, rings);
  EXPECT_EQ(argus::detect_rings(cloud, 256), rings);
}

TEST(RangeImage, Neighbors8WrapAroundAzimuth) {
  argus::RangeImage ri;
  ri.width = 2400;
  ri.height = 4;
  ri.range.assign(ri.width * ri.height, 1.0f);
  ri.intensity.assign(ri.width * ri.height, 0.0f);
  ri.no_return_mask.assign(ri.width * ri.height, 0);

  // Азимут 0: сосед az-1 должен быть width-1 (цикличность по азимуту).
  std::array<std::pair<uint32_t, uint32_t>, 8> nb{};
  const uint32_t n = argus::neighbors8(ri, 0, 1, nb);
  EXPECT_EQ(n, 8u);
  bool has_wrap = false;
  for (uint32_t k = 0; k < n; ++k) {
    if (nb[k].first == ri.width - 1) has_wrap = true;
  }
  EXPECT_TRUE(has_wrap);

  // Вертикальный край: ring=0 имеет 5 соседей (нет row -1).
  const uint32_t n0 = argus::neighbors8(ri, 0, 0, nb);
  EXPECT_EQ(n0, 5u);
  const uint32_t nlast = argus::neighbors8(ri, 0, ri.height - 1, nb);
  EXPECT_EQ(nlast, 5u);
}

TEST(RangeImage, MedianFilterKeepsFlatProfile) {
  argus::RangeImage ri;
  ri.width = 8;
  ri.height = 2;
  ri.range.assign(ri.width * ri.height, 25.5f);  // гладкий профиль (факт D5)
  ri.intensity.assign(ri.width * ri.height, 0.0f);
  ri.no_return_mask.assign(ri.width * ri.height, 0);

  const argus::RangeImage med = argus::azimuth_median_filter(ri, 3);
  for (size_t i = 0; i < med.range.size(); ++i) {
    EXPECT_NEAR(med.range[i], 25.5f, 1e-5f);
  }
}
