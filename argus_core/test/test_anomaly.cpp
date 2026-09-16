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
// Тесты детекторов аномалий (Фаза 3) на синтетических range image:
// плотный бокс на стене обнаруживается, платформенные полосы и
// транзиенты не тревожат, дыры потери возвратов детектируются.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "argus_core/anomaly.hpp"
#include "argus_core/range_image.hpp"

namespace {

constexpr uint32_t kW = 480;
constexpr uint32_t kH = 8;
constexpr float kWallRange = 25.5f; // доминирующий «стенный» возврат (факт D5)
constexpr float kBoxRange = 16.9f; // ближайшая поверхность препятствия (F-D)

argus::CleanCloud make_wall_cloud(float box_range, uint32_t az0, uint32_t az1, uint32_t r0,
                                  uint32_t r1) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool in_box = az >= az0 && az <= az1 && ring >= r0 && ring <= r1;
            const float d = in_box ? box_range : kWallRange;
            c.x.push_back(d);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    return c;
}

argus::GeometryResidualParams make_geom_params(uint32_t min_stable_frames) {
    argus::GeometryResidualParams p;
    p.median_half_window = 100;
    p.residual_threshold_m = 1.0f;
    p.min_cells = 50;
    p.min_fill = 0.3f;
    p.min_range = 4.0f;
    p.max_target_range_m = 45.0f;
    p.min_stable_frames = min_stable_frames;
    p.az_tolerance = 20;
    p.range_tolerance_m = 3.0f;
    return p;
}

argus::RangeImage ri_of(const argus::CleanCloud& cloud) {
    argus::RangeImageParams p;
    p.rings_fallback = kH;
    return argus::build_range_image(cloud, p);
}

} // namespace

TEST(GeometryDetector, DenseBoxOnWallDetected) {
    // Плотный бокс 50 az x 3 кольца на стене: устойчивый кластер в проёме.
    const auto cloud = make_wall_cloud(kBoxRange, 200, 249, 3, 5);
    const auto ri = ri_of(cloud);
    ASSERT_TRUE(ri.valid());

    argus::GeometryResidualDetector det(make_geom_params(1));
    const argus::AnomalySet set = det.detect(cloud, ri);

    ASSERT_FALSE(set.indices.empty());
    ASSERT_EQ(set.indices.size(), set.score.size());
    for (uint32_t idx : set.indices) {
        ASSERT_LT(idx, cloud.size());
        // Все аномалии — точки бокса, а не стены.
        EXPECT_FLOAT_EQ(cloud.x[idx], kBoxRange);
    }
}

TEST(GeometryDetector, StableBoxNeedsTwoFrames) {
    // Стоящий объект детектируется со второго кадра (min_stable_frames=2).
    const auto cloud = make_wall_cloud(kBoxRange, 200, 249, 3, 5);
    const auto ri = ri_of(cloud);

    argus::GeometryResidualDetector det(make_geom_params(2));
    const argus::AnomalySet first = det.detect(cloud, ri);
    EXPECT_TRUE(first.indices.empty());

    const argus::AnomalySet second = det.detect(cloud, ri);
    EXPECT_FALSE(second.indices.empty());
}

TEST(GeometryDetector, TransientBandDoesNotAlarm) {
    // Транзиент (край платформы проносится): дальность скачет между кадрами
    // на > range_tolerance_m, серия не накапливается — тревоги нет.
    const auto cloud_f1 = make_wall_cloud(8.0f, 100, 199, 3, 3);
    const auto cloud_f2 = make_wall_cloud(24.0f, 100, 199, 3, 3);
    const auto ri_f1 = ri_of(cloud_f1);
    const auto ri_f2 = ri_of(cloud_f2);

    argus::GeometryResidualDetector det(make_geom_params(2));
    const argus::AnomalySet a1 = det.detect(cloud_f1, ri_f1);
    const argus::AnomalySet a2 = det.detect(cloud_f2, ri_f2);
    EXPECT_TRUE(a1.indices.empty());
    EXPECT_TRUE(a2.indices.empty());
}

TEST(GeometryDetector, SparseBandDoesNotAlarm) {
    // Разреженные горизонтальные полосы (уступы): изолированные клетки
    // дают компоненты < min_cells — тревоги нет.
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool band = ring == 3 && az >= 150 && az <= 320 && az % 4 == 0;
            const float d = band ? 20.0f : kWallRange;
            c.x.push_back(d);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    const auto ri = ri_of(c);

    argus::GeometryResidualDetector det(make_geom_params(1));
    const argus::AnomalySet set = det.detect(c, ri);
    EXPECT_TRUE(set.indices.empty());
}

TEST(GeometryDetector, EmptyWallNoAlarm) {
    // Гладкая стена без препятствий: ни одного срабатывания.
    const auto cloud = make_wall_cloud(kWallRange, 1, 0, 0, 0);
    const auto ri = ri_of(cloud);

    argus::GeometryResidualDetector det(make_geom_params(1));
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.indices.empty());
}

namespace {

/// Облако с «дырами»: лучи-нули и отброшенные лучи задаются явно
/// (эмуляция фильтрации без сборки PointCloud2).
struct HoleSpec {
    uint32_t az0, az1, ring;
};

argus::CleanCloud make_cloud_with_holes(const std::vector<HoleSpec>& zero_holes,
                                        const std::vector<HoleSpec>& dropped) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    const auto is_hole = [&zero_holes, &dropped](uint32_t az, uint32_t ring, bool* zero) {
        for (const HoleSpec& h : zero_holes) {
            if (az >= h.az0 && az <= h.az1 && ring == h.ring) {
                *zero = true;
                return true;
            }
        }
        for (const HoleSpec& h : dropped) {
            if (az >= h.az0 && az <= h.az1 && ring == h.ring) {
                return true;
            }
        }
        return false;
    };

    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            bool zero = false;
            if (is_hole(az, ring, &zero)) {
                if (zero) {
                    c.no_return_raw.push_back(az * kH + ring);
                }
                continue; // отброшенные лучи не попадают в cloud
            }
            c.x.push_back(kWallRange);
            c.y.push_back(0.0f);
            c.z.push_back(0.0f);
            c.intensity.push_back(1.0f);
            c.raw_idx.push_back(az * kH + ring);
        }
    }
    return c;
}

} // namespace

TEST(NoReturnDetector, MissingRunWithLiveBaselineDetected) {
    // Серия из 10 «дыр» подряд при живой базовой линии — детектируется.
    const auto cloud = make_cloud_with_holes({{200, 209, 4}}, {});
    const auto ri = ri_of(cloud);
    ASSERT_TRUE(ri.valid());

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);

    ASSERT_EQ(set.cells.size(), 10u);
    ASSERT_EQ(set.cells_score.size(), 10u);
    for (size_t k = 0; k < set.cells.size(); ++k) {
        const uint32_t cell = set.cells[k];
        EXPECT_EQ(cell % ri.height, 4u); // только кольцо с дырами
        EXPECT_EQ(ri.no_return_mask[cell], 1u);
        EXPECT_GE(set.cells_score[k], 0.7f);
    }
}

TEST(NoReturnDetector, ShortRunNotFlagged) {
    // Серия короче min_missing_run — не тревожит.
    const auto cloud = make_cloud_with_holes({{200, 202, 4}}, {});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}

TEST(NoReturnDetector, DeadRingNotFlagged) {
    // Полностью «дырявое» кольцо (нет базовой линии) — не похоже на объект.
    const auto cloud = make_cloud_with_holes({{0, kW - 1, 4}}, {});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}

TEST(NoReturnDetector, DeadBaselineNotFlagged) {
    // Возвраты пропали и вокруг серии (базовая линия мертва) — не тревожит:
    // это слепой сектор, а не близкий объект. Края серии — лучи, отброшенные
    // фильтром (NaN без маски), а не «дыры».
    const auto cloud = make_cloud_with_holes({{200, 209, 4}}, {{195, 199, 4}, {210, 214, 4}});
    const auto ri = ri_of(cloud);

    argus::NoReturnParams p;
    argus::NoReturnDetector det(p);
    const argus::AnomalySet set = det.detect(cloud, ri);
    EXPECT_TRUE(set.cells.empty());
}
