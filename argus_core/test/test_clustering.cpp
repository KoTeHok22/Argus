// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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
    ri.range.assign(static_cast<size_t>(width) * height, std::numeric_limits<float>::quiet_NaN());
    ri.intensity.assign(static_cast<size_t>(width) * height, 0.0f);
    ri.no_return_mask.assign(static_cast<size_t>(width) * height, 0);
    return ri;
}

} // namespace

TEST(Clustering, AdaptiveThresholdGrowsWithRange) {
    const float base = argus::neighbor_distance_at(0.35f, 20.0f, 50.0f, true);
    const float far = argus::neighbor_distance_at(0.35f, 200.0f, 50.0f, true);
    EXPECT_FLOAT_EQ(base, 0.35f); // ближе reference — без роста
    EXPECT_GT(far, base);         // дальше reference — порог растёт
    const float fixed = argus::neighbor_distance_at(0.35f, 200.0f, 50.0f, false);
    EXPECT_FLOAT_EQ(fixed, 0.35f); // без адаптации порог константен
}

TEST(Clustering, EmptyInputEmptyOutput) {
    auto ri = make_ri(100, 16);
    argus::CleanCloud cloud;
    argus::AnomalySet merged;
    argus::ClusteringParams p;
    EXPECT_TRUE(argus::cluster_anomalies(cloud, ri, merged, p).empty());
}

// Семантика AnomalySet.indices: индексы в CleanCloud отображаются в клетки
// развёртки через raw_idx; кластер собирается по клеткам, метрики — по точкам.
TEST(Clustering, CleanCloudIndicesMapThroughRawIdx) {
    const uint32_t w = 100, h = 8;
    argus::CleanCloud cloud;
    cloud.rings = h;
    cloud.n_raw = w * h;
    std::vector<uint32_t> box_cloud_idx;
    for (uint32_t src = 0; src < cloud.n_raw; ++src) {
        if (src % 10 == 7) continue; // эмуляция отбраковки фильтром
        const uint32_t az = src / h, ring = src % h;
        const bool in_box = az >= 30 && az <= 45 && ring >= 3 && ring <= 5;
        const float d = in_box ? 16.9f : 25.5f;
        cloud.x.push_back(d);
        cloud.y.push_back(0.0f);
        cloud.z.push_back(0.0f);
        cloud.intensity.push_back(1.0f);
        cloud.raw_idx.push_back(src);
        if (in_box) box_cloud_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }
    ASSERT_FALSE(box_cloud_idx.empty());

    argus::RangeImageParams rp;
    rp.rings_fallback = h;
    const auto ri = argus::build_range_image(cloud, rp);
    ASSERT_TRUE(ri.valid());

    argus::AnomalySet merged;
    merged.indices = box_cloud_idx;
    argus::ClusteringParams p;
    p.min_cluster_size = 15;
    p.min_extent_m = 0.0f;
    p.max_extent_m = 100.0f;

    const auto clusters = argus::cluster_anomalies(cloud, ri, merged, p);
    ASSERT_EQ(clusters.size(), 1u);
    const auto& c = clusters[0];
    EXPECT_EQ(c.point_count, box_cloud_idx.size());
    EXPECT_EQ(c.indices.size(), box_cloud_idx.size());
    for (uint32_t idx : c.indices) {
        EXPECT_FLOAT_EQ(cloud.x[idx], 16.9f);
    }
    EXPECT_NEAR(c.nearest_range, 16.9f, 1e-4f);
}

// Полные сценарии слияния/разделения — Ф4.7.2, после интеграции fusion.
