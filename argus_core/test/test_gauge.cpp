// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Тесты габарита: краевые случаи — граница, фаска, max_range.

#include <gtest/gtest.h>

#include "argus_core/gauge.hpp"

namespace {

argus::ClearanceGauge default_gauge() {
  argus::ClearanceGaugeParams p;
  p.safety_margin = 0.0f;  // для точных проверок границ
  return argus::ClearanceGauge(p);
}

}  // namespace

TEST(Gauge, CenterInside) {
  const auto g = default_gauge();
  EXPECT_TRUE(g.contains(50.0f, 0.0f, 2.0f));
}

TEST(Gauge, OutsideByWidth) {
  const auto g = default_gauge();
  EXPECT_FALSE(g.contains(50.0f, 1.451f, 2.0f));
  EXPECT_FALSE(g.contains(50.0f, -1.451f, 2.0f));
}

TEST(Gauge, BoundaryExact) {
  const auto g = default_gauge();
  EXPECT_TRUE(g.contains(50.0f, 1.45f, 2.0f));   // точная граница по ширине
  EXPECT_TRUE(g.contains(50.0f, 0.0f, 3.60f));   // точная граница по высоте
}

TEST(Gauge, ChamferCutsTopCorner) {
  const auto g = default_gauge();
  // Угловая точка выше начала фаски, на полной ширине — снаружи.
  EXPECT_FALSE(g.contains(50.0f, 1.45f, 3.59f));
  // На ширине, уменьшенной фаской, — внутри.
  EXPECT_TRUE(g.contains(50.0f, 1.30f, 3.59f));
}

TEST(Gauge, OutsideByHeight) {
  const auto g = default_gauge();
  EXPECT_FALSE(g.contains(50.0f, 0.0f, 0.15f));  // ниже base_offset
  EXPECT_FALSE(g.contains(50.0f, 0.0f, 3.61f));  // выше height
}

TEST(Gauge, OutsideByRange) {
  const auto g = default_gauge();
  EXPECT_FALSE(g.contains(-0.01f, 0.0f, 2.0f));  // позади носа
  EXPECT_TRUE(g.contains(299.0f, 0.0f, 2.0f));
}

TEST(Gauge, ForwardDistance) {
  const auto g = default_gauge();
  EXPECT_FLOAT_EQ(g.forward_distance(47.5f, 0.0f, 2.0f), 47.5f);
  EXPECT_FLOAT_EQ(g.forward_distance(47.5f, 5.0f, 2.0f), -1.0f);  // вне
}

TEST(Gauge, DistanceTo) {
  const auto g = default_gauge();
  EXPECT_FLOAT_EQ(g.distance_to(50.0f, 0.0f, 2.0f), 0.0f);       // внутри
  EXPECT_FLOAT_EQ(g.distance_to(50.0f, 2.45f, 2.0f), 1.0f);      // сбоку в 1 м
  EXPECT_FLOAT_EQ(g.distance_to(40.0f, 0.0f, 4.6f), 1.0f);       // сверху в 1 м
}
