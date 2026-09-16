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

#include "argus_core/gauge.hpp"

#include <algorithm>
#include <cmath>

namespace argus {

bool ClearanceGauge::contains(float x, float y, float z) const {
    // Продольные границы: от носа до предела проверки (X — вперёд).
    const float x_min = p_.nose_offset - p_.safety_margin;
    const float x_max = p_.max_range;
    if (x < x_min || x > x_max) {
        return false;
    }

    // Поперечный профиль с фаской по верхним углам.
    const float hw = p_.half_width + p_.safety_margin;
    const float z_min = p_.base_offset - p_.safety_margin;
    const float z_max = p_.height + p_.safety_margin;
    if (z < z_min || z > z_max) {
        return false;
    }
    if (std::abs(y) > hw) {
        return false;
    }

    // Фаска: выше chamfer_start_z_ ширина линейно сужается до (hw - chamfer).
    if (z > chamfer_start_z_ && chamfer_height_ > 0.0f) {
        const float t = (z - chamfer_start_z_) / chamfer_height_;
        const float hw_at_z = hw - p_.chamfer * t;
        if (std::abs(y) > hw_at_z) {
            return false;
        }
    }
    return true;
}

float ClearanceGauge::forward_distance(float x, float y, float z) const {
    if (!contains(x, y, z)) {
        return -1.0f;
    }
    return x;
}

float ClearanceGauge::distance_to(float x, float y, float z) const {
    if (contains(x, y, z)) {
        return 0.0f;
    }

    const float hw = p_.half_width + p_.safety_margin;
    const float z_min = p_.base_offset - p_.safety_margin;
    const float z_max = p_.height + p_.safety_margin;

    // Расстояние до бокса без фаски; фаска упрощаем (консервативно
    // занижаем дистанцию — для тревог это безопаснее).
    const float dx = std::max(0.0f, p_.nose_offset - x);
    const float dy = std::max(0.0f, std::abs(y) - hw);
    const float dz = std::max(0.0f, std::max(z_min - z, z - z_max));
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void ClearanceGauge::to_mesh(float max_range, std::vector<Eigen::Vector3f>& vertices,
                             std::vector<uint32_t>& indices) const {
    // Упрощённый wireframe-профиль: 2 сечения (нос и хвост) по 4 угла с фаской.
    // Полная отрисовка — в Ф4.7.1 (маркеры RViz2).
    vertices.clear();
    indices.clear();

    const float hw = p_.half_width;
    const float z0 = p_.base_offset;
    const float z1 = p_.height - p_.chamfer;
    const float z2 = p_.height;
    const float hw_top = p_.half_width - p_.chamfer;

    for (int side = 0; side < 2; ++side) {
        const float x = (side == 0) ? p_.nose_offset : max_range;
        // 8 вершин профиля: нижний прямоугольник + верх с фаской.
        vertices.emplace_back(x, -hw, z0);
        vertices.emplace_back(x, hw, z0);
        vertices.emplace_back(x, hw, z1);
        vertices.emplace_back(x, hw_top, z2);
        vertices.emplace_back(x, -hw_top, z2);
        vertices.emplace_back(x, -hw, z1);
    }
}

ClearanceGauge::ClearanceGauge(const ClearanceGaugeParams& p) : p_(p) {
    // Фаска занимает верхнюю часть профиля высотой chamfer.
    chamfer_start_z_ = p_.height - p_.chamfer;
    chamfer_height_ = std::max(0.001f, p_.chamfer);
}

} // namespace argus
