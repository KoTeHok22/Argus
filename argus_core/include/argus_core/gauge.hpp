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
// Габарит движения поезда — твёрдое тело, усечённая призма вдоль оси X.
// Профиль поперёк оси (вид спереди):
//
//        +------------------------+   <- height
//       /                          /
//      /                          /    <- chamfer (фаска по верхним углам)
//     |                          |
//     |                          |
//     +--------------------------+
//     <--------- 2*half_width ---->
//
// Соглашение об осях: X — вперёд, Y — влево, Z — вверх (в СК лидара).

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "argus_core/types.hpp"

namespace argus {

struct ClearanceGaugeParams {
    float half_width = 1.45f; // м. УТОЧНИТЬ по ТТХ подвижного состава
    float height = 3.60f;     // м от головки рельса
    float base_offset = 0.20f; // м от головки рельса до низа габарита
    float chamfer = 0.20f;     // м, фаска по верхним углам
    float nose_offset = 0.00f; // м от лидара до носа поезда
    float max_range = 300.0f;  // м, предел проверки
    float safety_margin = 0.10f; // м, дополнительный запас
};

class ClearanceGauge {
public:
    explicit ClearanceGauge(const ClearanceGaugeParams& p);

    /// Точка внутри габарита (с учётом safety_margin)?
    bool contains(float x, float y, float z) const;

    /// Продольная дистанция до точки, если внутри; -1 если вне.
    float forward_distance(float x, float y, float z) const;

    /// Ближайшая дистанция до бокса габарита (0 если точка внутри).
    float distance_to(float x, float y, float z) const;

    /// Mesh габарита до заданной дальности — для визуализации в RViz2.
    void to_mesh(float max_range, std::vector<Eigen::Vector3f>& vertices,
                 std::vector<uint32_t>& indices) const;

    const ClearanceGaugeParams& params() const { return p_; }

private:
    ClearanceGaugeParams p_;
    // Пороги фаски: z, при котором начинается сужение, и соответствующий
    // запас по ширине на уровне z.
    float chamfer_start_z_ = 0.0f;
    float chamfer_height_ = 0.0f;
};

} // namespace argus
