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
// Тесты трекинга: подтверждение, сохранение ID, пропуски, TTC.

#include <gtest/gtest.h>

#include "argus_core/tracking.hpp"

namespace {

argus::Cluster make_cluster(float x, float y, float z) {
    argus::Cluster c;
    c.centroid = Eigen::Vector3f(x, y, z);
    c.nearest_range = std::sqrt(x * x + y * y + z * z);
    c.volume = 0.5f;
    c.point_count = 50;
    return c;
}

} // namespace

TEST(Tracking, ConfirmsAfterMinHits) {
    argus::TrackingParams p;
    p.min_hits_to_confirm = 3;
    argus::ObstacleTracker tr(p);

    const auto out1 = tr.update({make_cluster(10, 0, 0)}, 0.1f, 10.0f);
    const auto out2 = tr.update({make_cluster(11, 0, 0)}, 0.1f, 10.0f);
    const auto out3 = tr.update({make_cluster(12, 0, 0)}, 0.1f, 10.0f);
    EXPECT_TRUE(out1.empty());
    EXPECT_TRUE(out2.empty());
    ASSERT_EQ(out3.size(), 1u);
    EXPECT_EQ(out3[0].hits, 3u);
}

TEST(Tracking, KeepsIdAcrossFrames) {
    argus::TrackingParams p;
    p.min_hits_to_confirm = 1;
    argus::ObstacleTracker tr(p);

    const auto a = tr.update({make_cluster(10, 0, 0)}, 0.1f, 10.0f);
    ASSERT_EQ(a.size(), 1u);
    const uint32_t id = a[0].id;
    const auto b = tr.update({make_cluster(11, 0, 0)}, 0.1f, 10.0f);
    ASSERT_EQ(b.size(), 1u);
    EXPECT_EQ(b[0].id, id); // ID сохраняется между кадрами
}

TEST(Tracking, SurvivesMissesThenResets) {
    argus::TrackingParams p;
    p.min_hits_to_confirm = 1;
    p.max_misses_to_keep = 4;
    argus::ObstacleTracker tr(p);

    tr.update({make_cluster(10, 0, 0)}, 0.1f, 10.0f);
    // 2 пропуска: трек ещё жив (допускает max_misses_to_keep = 4),
    // тревога держится в окне пропусков — снимается при misses > max_misses.
    const auto after_misses = tr.update({}, 0.1f, 10.0f);
    ASSERT_EQ(tr.all_tracks().size(), 1u);
    const auto after_misses2 = tr.update({}, 0.1f, 10.0f);
    ASSERT_EQ(tr.all_tracks().size(), 1u);
    EXPECT_EQ(after_misses.size() + after_misses2.size(), 2u);
    // 5 пропусков подряд: трек сброшен.
    for (int i = 0; i < 5; ++i) {
        tr.update({}, 0.1f, 10.0f);
    }
    EXPECT_TRUE(tr.all_tracks().empty());
}

TEST(Tracking, TtcClosingAndReceding) {
    argus::Track t;
    t.state(0) = 50.0f; // объект в 50 м
    t.state(3) = 0.0f;  // объект стоит

    EXPECT_NEAR(argus::compute_ttc(t, 10.0f), 5.0f, 1e-4f); // 50/10

    // Объект удаляется быстрее, чем поезд едет: не сближаемся.
    t.state(3) = 12.0f;
    EXPECT_FLOAT_EQ(argus::compute_ttc(t, 10.0f), -1.0f);
}
