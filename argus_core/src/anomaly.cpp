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
// Каркас детекторов. Реализации — Ф3.5.1 (no_return), Ф3.5.2 (geometry),
// Ф3.5.3 (free_space) по PLAN.md §8. Здесь — минимальные безопасные версии:
// пустой AnomalySet (система не даёт ложных тревог, пока детекторы не готовы).

#include "argus_core/anomaly.hpp"

namespace argus {

NoReturnDetector::NoReturnDetector(const NoReturnParams& p) : p_(p) {}

AnomalySet NoReturnDetector::detect(const CleanCloud&, const RangeImage&) {
    // Ф3.5.1: искать серии «дыр» по азимуту в no_return_mask.
    AnomalySet out;
    out.source = name();
    return out;
}

GeometryResidualDetector::GeometryResidualDetector(const GeometryResidualParams& p) : p_(p) {}

AnomalySet GeometryResidualDetector::detect(const CleanCloud&, const RangeImage&) {
    // Ф3.5.2: вычесть azimuth_median_filter и найти выбросы > sigma_threshold.
    AnomalySet out;
    out.source = name();
    return out;
}

void GeometryResidualDetector::update(const CleanCloud&, const RangeImage&) {
    // Обновление профиля тоннеля с адаптивной альфой (Ф2.3.4).
}

FreeSpaceDetector::FreeSpaceDetector(TunnelModel* model, const FreeSpaceParams& p)
    : model_(model), p_(p) {}

AnomalySet FreeSpaceDetector::detect(const CleanCloud&, const RangeImage&) {
    // Ф3.5.3: точка в клетке, надёжно наблюдавшейся пустой (violates_free_space).
    AnomalySet out;
    out.source = name();
    return out;
}

} // namespace argus
