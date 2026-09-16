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
// Габарит движения поезда — твёрдое тело, усечённая призма вдоль оси
// движения. Профиль поперёк оси (вид спереди):
//
//        +------------------------+   <- height
//       /                          /
//      /                          /    <- chamfer (фаска по верхним углам)
//     |                          |
//     |                          |
//     +--------------------------+
//     <--------- 2*half_width ---->
//
// Ось «вперёд» конфигурируется (факт F-F: в doubleT-бэгах ось тоннеля — Y,
// поезд движется в сторону -Y). По умолчанию — +X (PLAN.md §9.1).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "argus_core/types.hpp"

namespace argus {

/// Ось «вперёд» в СК лидара. Латеральная ось = forward, повёрнутый на +90°.
enum class ForwardAxis : uint8_t { PosX, NegX, PosY, NegY };

/// "x"/"+x" -> PosX, "-x" -> NegX, "y"/"+y" -> PosY, "-y" -> NegY.
/// Неизвестная строка -> false, out не меняется.
bool parse_forward_axis(const std::string& name, ForwardAxis& out);

struct ClearanceGaugeParams {
    float half_width = 1.45f; // м. УТОЧНИТЬ по ТТХ подвижного состава
    float height = 3.60f;     // м от головки рельса
    float base_offset = 0.20f; // м от головки рельса до низа габарита
    float chamfer = 0.20f;     // м, фаска по верхним углам
    float nose_offset = 0.00f; // м от лидара до носа поезда вдоль оси движения
    float max_range = 300.0f;    // м, предел проверки
    float safety_margin = 0.10f; // м, дополнительный запас
    // Высота сенсора над головкой рельса, м. Габарит отсчитывается от
    // головки рельса, а точки приходят в СК сенсора: z-границы сдвигаются
    // на -sensor_height. 0 = сенсор на уровне головки рельса.
    // По данным platform-бэга пол в СК сенсора на z ~ -1.2 м (допущение A3).
    float sensor_height = 0.0f;
    ForwardAxis forward_axis = ForwardAxis::PosX; // ось «вперёд» (факт F-F)
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

    /// Компоненты осей «вперёд»/«влево» в СК лидара (для внешних фильтров).
    float forward_x() const { return fx_; }
    float forward_y() const { return fy_; }
    float left_x() const { return lx_; }
    float left_y() const { return ly_; }

private:
    ClearanceGaugeParams p_;
    // Пороги фаски: z, при котором начинается сужение, и соответствующий
    // запас по ширине на уровне z.
    float chamfer_start_z_ = 0.0f;
    float chamfer_height_ = 0.0f;
    // Компоненты осей «вперёд» и «влево» в СК лидара.
    float fx_ = 1.0f, fy_ = 0.0f; // forward
    float lx_ = 0.0f, ly_ = 1.0f; // left

    /// Координаты точки в системе габарита: gx — вперёд, gy — влево.
    void to_gauge(float x, float y, float& gx, float& gy) const;
    /// Обратное преобразование (для mesh).
    void from_gauge(float gx, float gy, float& x, float& y) const;
};

} // namespace argus
