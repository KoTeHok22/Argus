# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Описание сообщений Argus.

# Одно обнаруженное препятствие.
# Публикуется в составе ObstacleArray нодой argus_node_fusion.

std_msgs/Header header

# Идентификация
uint32 track_id            # устойчивый ID трека между кадрами
uint32 persistence_frames  # сколько кадров объект подтверждён подряд

# Позиция и геометрия (в system coordinates лидара)
geometry_msgs/Point   position        # центроид кластера
geometry_msgs/Vector3 extent          # габаритные размеры по осям, м
float32 volume_m3                     # объём по ограничивающему боксу, м^3

# Дистанции
float32 range_m            # расстояние до ближайшей точки кластера, м
float32 centroid_range_m   # расстояние до центроида, м
float32 ttc_s              # time to collision, с; <0 если не сближаемся

# Уверенность и происхождение
float32 confidence         # 0..1, агрегированная уверенность
uint8   votes_free_space   # 1 если детектор свободного пространства сработал
uint8   votes_no_return    # 1 если детектор потери возвратов сработал
uint8   votes_geometry     # 1 если детектор геометрического остатка сработал
uint32  point_count        # число точек в кластере

# Классификация важности
uint8 severity
uint8 SEVERITY_INFO     = 0
uint8 SEVERITY_WARNING  = 1
uint8 SEVERITY_CRITICAL = 2

# Человекочитаемое объяснение (лог объяснимости)
string reason
