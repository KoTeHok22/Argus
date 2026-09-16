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
// Тесты fusion (Фаза 4): препятствие в габарите даёт подтверждённую тревогу,
// боковой рельеф отсеивается габаритом, клетки no_return не кластеризуются.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "argus_core/fusion.hpp"
#include "argus_core/range_image.hpp"

namespace {

constexpr uint32_t kW = 480;
constexpr uint32_t kH = 8;
constexpr float kWallRange = 25.5f;
constexpr float kBoxRange = 16.9f;

/// Сцена «стена впереди + бокс»: ось тоннеля -Y (факт F-F).
/// Латераль (x лидара) задаётся параметром — 0.0 в габарите, 4.8 сбоку.
argus::CleanCloud make_scene(float box_lateral_x) {
    argus::CleanCloud c;
    c.rings = kH;
    c.n_raw = kW * kH;
    for (uint32_t az = 0; az < kW; ++az) {
        for (uint32_t ring = 0; ring < kH; ++ring) {
            const bool in_box = az >= 200 && az <= 249 && ring >= 3 && ring <= 5;
            const float lateral = in_box ? box_lateral_x + ring * 0.2f : -0.35f + ring * 0.1f;
            const float along = in_box ? kBoxRange + (az - 200) * 0.01f : kWallRange;
            const float z = 0.4f + ring * 0.1f;
            // вперёд = -Y: точка (x=латераль, y=-дальность)
            c.x.push_back(lateral);
            c.y.push_back(-along);
            c.z.push_back(z);
            c.intensity.push_back(1.0f);
            c.ring.push_back(static_cast<uint16_t>(ring));
            c.azimuth_idx.push_back(az);
        }
    }
    return c;
}

argus::RangeImage ri_of(const argus::CleanCloud& cloud) {
    argus::RangeImageParams p;
    p.rings_fallback = kH;
    return argus::build_range_image(cloud, p);
}

argus::AnomalySet geometry_of_box(const argus::CleanCloud& cloud) {
    argus::AnomalySet set;
    set.source = "geometry";
    for (size_t i = 0; i < cloud.size(); ++i) {
        // только бокс (16.9 м), не стена (25.5 м)
        const bool in_box = cloud.y[i] <= -kBoxRange - 0.001f && cloud.y[i] > -18.0f;
        if (in_box) {
            set.indices.push_back(static_cast<uint32_t>(i));
            set.score.push_back(0.8f);
        }
    }
    return set;
}

argus::FusionParams fusion_params() {
    argus::FusionParams p;
    p.clustering.min_cluster_size = 15;
    p.clustering.min_extent_m = 0.1f;
    p.clustering.max_extent_m = 50.0f;
    p.tracking.min_hits_to_confirm = 2;
    return p;
}

argus::ClearanceGauge gauge_neg_y() {
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    return argus::ClearanceGauge(gp);
}

} // namespace

TEST(Fusion, ObstacleAheadTriggersConfirmedAlert) {
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);
    const auto geometry = geometry_of_box(cloud);
    ASSERT_FALSE(geometry.indices.empty());

    argus::FusionPipeline pipe(gauge_neg_y(), fusion_params());

    const argus::FusionResult f1 = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_FALSE(f1.alert); // 1 кадр — трек ещё не подтверждён

    const argus::FusionResult f2 = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_TRUE(f2.alert);
    EXPECT_FALSE(f2.tracks.empty());
    EXPECT_TRUE(f2.tracks[0].confirmed);
    EXPECT_GT(f2.nearest_forward_m, 16.0f);
    EXPECT_LT(f2.nearest_forward_m, 17.5f);
    EXPECT_EQ(f2.n_filtered_out, 0u);
}

TEST(Fusion, LateralWallFilteredByGauge) {
    // Тот же бокс, но сбоку (латераль ~4.8 м) — законная геометрия вне
    // габарита: кластеры отсеются, тревоги нет.
    const auto cloud = make_scene(4.8f);
    const auto ri = ri_of(cloud);
    const auto geometry = geometry_of_box(cloud);

    argus::FusionPipeline pipe(gauge_neg_y(), fusion_params());
    pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    const argus::FusionResult f2 = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);
    EXPECT_FALSE(f2.alert);
    EXPECT_TRUE(f2.clusters.empty());
    EXPECT_GT(f2.n_filtered_out, 0u);
}

TEST(Fusion, NoReturnCellsCountedNotClustered) {
    // Дыры no_return не имеют точек: учитываются счётчиком, кластеров
    // и тревоги не дают.
    const auto cloud = make_scene(0.0f);
    const auto ri = ri_of(cloud);

    argus::AnomalySet no_return;
    no_return.source = "no_return";
    for (uint32_t az = 200; az <= 209; ++az) {
        no_return.cells.push_back(ri.idx(az, 4));
        no_return.cells_score.push_back(1.0f);
    }

    argus::FusionPipeline pipe(gauge_neg_y(), fusion_params());
    const argus::FusionResult r = pipe.update(cloud, ri, {}, no_return, 0.1f, 0.0f);
    EXPECT_EQ(r.n_anom_cells, 10u);
    EXPECT_TRUE(r.clusters.empty());
    EXPECT_FALSE(r.alert);
}

TEST(Fusion, GroundFilterDropsRailLikePoints) {
    // Рельсы: низкие точки (z_rel < ground_clearance_m) в зоне колеи
    // не кластеризуются; объект над рельсом остаётся детектируемым.
    argus::CleanCloud cloud;
    cloud.rings = kH;
    cloud.n_raw = kW * kH;
    std::vector<uint32_t> rail_idx, object_idx;
    for (uint32_t az = 0; az < 200; ++az) {
        // Рельсы: латераль -1.5, z на 0.3 над головкой рельса (z = -1.2+0.3)
        cloud.x.push_back(-1.5f);
        cloud.y.push_back(-static_cast<float>(az) * 0.05f - 3.0f);
        cloud.z.push_back(-0.9f);
        cloud.intensity.push_back(1.0f);
        cloud.ring.push_back(static_cast<uint16_t>(az % kH));
        cloud.azimuth_idx.push_back(az);
        rail_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }
    for (uint32_t az = 200; az <= 249; ++az) {
        // Объект: z_rel = 1.0 м над головкой рельса (z = -1.2 + 1.0),
        // протяжённость по оси движения ~1 м.
        cloud.x.push_back(0.0f);
        cloud.y.push_back(-16.9f - static_cast<float>(az - 200) * 0.02f);
        cloud.z.push_back(-0.2f);
        cloud.intensity.push_back(1.0f);
        cloud.ring.push_back(static_cast<uint16_t>(az % kH));
        cloud.azimuth_idx.push_back(az);
        object_idx.push_back(static_cast<uint32_t>(cloud.size() - 1));
    }

    argus::RangeImageParams rp;
    rp.rings_fallback = kH;
    const auto ri = argus::build_range_image(cloud, rp);

    argus::AnomalySet geometry;
    geometry.source = "geometry";
    geometry.indices = rail_idx;
    geometry.indices.insert(geometry.indices.end(), object_idx.begin(), object_idx.end());
    geometry.score.assign(geometry.indices.size(), 0.8f);

    argus::FusionParams p = fusion_params();
    p.clustering.min_cluster_size = 5;
    p.clustering.min_extent_m = 0.1f;
    p.tracking.min_hits_to_confirm = 1;
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    gp.sensor_height = 1.2f; // как в конфиге: пол на -1.2 м в СК сенсора
    argus::FusionPipeline pipe(argus::ClearanceGauge(gp), p);
    const argus::FusionResult r = pipe.update(cloud, ri, geometry, {}, 0.1f, 0.0f);

    // Кластер объекта остался, рельсы не дали ложного кластера.
    ASSERT_EQ(r.clusters.size(), 1u);
    EXPECT_NEAR(r.clusters[0].forward_distance, 16.9f, 0.5f);
    EXPECT_EQ(r.clusters[0].point_count, object_idx.size());
    EXPECT_TRUE(r.alert);
}

TEST(Fusion, GaugeAxisParsedAndApplied) {
    argus::ForwardAxis axis = argus::ForwardAxis::PosX;
    ASSERT_TRUE(argus::parse_forward_axis("-y", axis));
    EXPECT_EQ(axis, argus::ForwardAxis::NegY);
    ASSERT_TRUE(argus::parse_forward_axis("+x", axis));
    EXPECT_EQ(axis, argus::ForwardAxis::PosX);
    EXPECT_FALSE(argus::parse_forward_axis("diagonal", axis));

    // Препятствие в 16.9 м по -Y: габарит с осью -Y видит его на 16.9 вперёд.
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    const argus::ClearanceGauge g(gp);
    EXPECT_FLOAT_EQ(g.forward_distance(0.5f, -16.9f, 1.0f), 16.9f);
    EXPECT_FLOAT_EQ(g.distance_to(0.5f, -16.9f, 1.0f), 0.0f);
    // Та же точка для габарита +X — далеко сбоку.
    argus::ClearanceGaugeParams gpx;
    const argus::ClearanceGauge gx(gpx);
    EXPECT_EQ(gx.forward_distance(0.5f, -16.9f, 1.0f), -1.0f);
}

TEST(Fusion, GaugeSensorHeightShiftsZBounds) {
    // Пол в СК сенсора на z ~ -1.2 (допущение A3): точка на уровне пола
    // ниже габарита, а точка на 0.5 м над полом — внутри (z от рельса 0.5).
    argus::ClearanceGaugeParams gp;
    gp.forward_axis = argus::ForwardAxis::NegY;
    gp.sensor_height = 1.2f;
    const argus::ClearanceGauge g(gp);
    EXPECT_FLOAT_EQ(g.forward_distance(0.0f, -16.9f, -0.7f), 16.9f); // 0.5 м над полом
    EXPECT_EQ(g.forward_distance(0.0f, -16.9f, -1.2f), -1.0f);       // уровень пола
    // Без сдвига та же точка пола была бы «внутри» — фиксируем сдвиг.
    argus::ClearanceGaugeParams g0;
    g0.forward_axis = argus::ForwardAxis::NegY;
    const argus::ClearanceGauge without_shift(g0);
    EXPECT_EQ(without_shift.forward_distance(0.0f, -16.9f, -1.2f), -1.0f);
    EXPECT_FLOAT_EQ(without_shift.forward_distance(0.0f, -16.9f, -0.7f), -1.0f);
}
