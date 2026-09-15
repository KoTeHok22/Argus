// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0
//
// Тесты cloud_filter: раскладка 26 байт, отбраковка мусора (факты D1, D2, D4).

#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "argus_core/cloud_filter.hpp"

namespace {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr uint32_t kPointStep = 26;

/// Собрать сообщение раскладки Hesai ROS-драйвера (факт D1):
/// x@0 y@4 z@8 intensity@12 ring@16 timestamp@18.
PointCloud2 make_msg(size_t n) {
  PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(n);
  msg.point_step = kPointStep;
  msg.is_dense = false;

  auto add = [&msg](const std::string& name, uint32_t offset,
                    uint8_t datatype) {
    PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = 1;
    msg.fields.push_back(f);
  };
  add("x", 0, PointField::FLOAT32);
  add("y", 4, PointField::FLOAT32);
  add("z", 8, PointField::FLOAT32);
  add("intensity", 12, PointField::FLOAT32);
  add("ring", 16, PointField::UINT16);
  add("timestamp", 18, PointField::FLOAT64);

  msg.data.assign(n * kPointStep, 0);
  msg.row_step = static_cast<uint32_t>(n) * kPointStep;
  return msg;
}

void put_point(PointCloud2& msg, size_t i, float x, float y, float z,
               float intensity, uint16_t ring) {
  uint8_t* p = msg.data.data() + i * kPointStep;
  std::memcpy(p + 0, &x, 4);
  std::memcpy(p + 4, &y, 4);
  std::memcpy(p + 8, &z, 4);
  std::memcpy(p + 12, &intensity, 4);
  std::memcpy(p + 16, &ring, 2);
  const double ts = 123.5;
  std::memcpy(p + 18, &ts, 8);
}

}  // namespace

TEST(CloudFilter, ParseLayoutHesai26Bytes) {
  auto msg = make_msg(4);
  const auto layout = argus::parse_layout(msg);
  ASSERT_TRUE(layout.has_value());
  EXPECT_EQ(layout->point_step, kPointStep);
  EXPECT_EQ(layout->offset_x, 0u);
  EXPECT_EQ(layout->offset_y, 4u);
  EXPECT_EQ(layout->offset_z, 8u);
  EXPECT_EQ(layout->offset_intensity, 12u);
  EXPECT_EQ(layout->offset_ring, 16u);
  EXPECT_EQ(layout->offset_timestamp, 18u);
  EXPECT_EQ(layout->n_points, 4u);
}

TEST(CloudFilter, RejectsMissingZ) {
  auto msg = make_msg(2);
  msg.fields.pop_back();  // убрали timestamp — ок; уберём z вручную
  for (auto& f : msg.fields) {
    if (f.name == "z") {
      f.datatype = PointField::FLOAT64;  // не float32 — отвергнуть
    }
  }
  const auto layout = argus::parse_layout(msg);
  EXPECT_FALSE(layout.has_value());
}

TEST(CloudFilter, RejectsUnknownLayout) {
  auto msg = make_msg(2);
  msg.fields.clear();  // ни одного поля
  const auto layout = argus::parse_layout(msg);
  EXPECT_FALSE(layout.has_value());
}

TEST(CloudFilter, DropsJunkZeroAndNonfiniteKeepsValid) {
  const size_t n = 6;
  auto msg = make_msg(n);
  // 0: мусор |x| > 1e4
  put_point(msg, 0, 5.0e5f, 0.0f, 0.0f, 0.0f, 0);
  // 1: ровно (0,0,0) — нет возврата
  put_point(msg, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0);
  // 2: NaN
  const float nan = std::numeric_limits<float>::quiet_NaN();
  put_point(msg, 2, nan, 1.0f, 1.0f, 0.0f, 0);
  // 3: слишком близко
  put_point(msg, 3, 0.1f, 0.0f, 0.0f, 0.0f, 0);
  // 4-5: валидные
  put_point(msg, 4, 10.0f, 0.0f, 0.0f, 5.0f, 1);
  put_point(msg, 5, 0.0f, 20.0f, 3.0f, 7.0f, 2);

  const auto layout = argus::parse_layout(msg);
  ASSERT_TRUE(layout.has_value());

  argus::FilterParams params;  // дефолты: [0.5, 400] м, |coord| <= 1e4
  argus::FilterStats stats;
  const argus::CleanCloud cloud = argus::filter_and_index(msg, *layout, params, &stats);

  EXPECT_EQ(stats.n_raw, n);
  EXPECT_EQ(stats.reject_out_of_range, 2u);  // мусор + слишком близко
  EXPECT_EQ(stats.reject_zero, 1u);
  EXPECT_EQ(stats.reject_nonfinite, 1u);
  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_FLOAT_EQ(cloud.x[0], 10.0f);
  EXPECT_FLOAT_EQ(cloud.y[1], 20.0f);
}

TEST(CloudFilter, InvariantsHold) {
  const size_t n = 3;
  auto msg = make_msg(n);
  put_point(msg, 0, 10.0f, 0.0f, 1.0f, 5.0f, 0);
  put_point(msg, 1, 12.0f, 0.0f, 1.0f, 5.0f, 1);
  put_point(msg, 2, 14.0f, 0.0f, 1.0f, 5.0f, 2);

  const auto layout = argus::parse_layout(msg);
  ASSERT_TRUE(layout.has_value());
  const argus::CleanCloud cloud =
      argus::filter_and_index(msg, *layout, argus::FilterParams{}, nullptr);

  ASSERT_EQ(cloud.size(), 3u);
  for (size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_TRUE(std::isfinite(cloud.x[i]));
    EXPECT_TRUE(std::isfinite(cloud.y[i]));
    EXPECT_TRUE(std::isfinite(cloud.z[i]));
  }
}
