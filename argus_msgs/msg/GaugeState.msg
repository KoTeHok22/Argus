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

# Состояние габарита движения поезда. Публикуется (/argus/gauge).
# Используется визуализацией (RViz2) и оценщиком тревог.

std_msgs/Header header

# Параметры габарита (см. config/gauge_default.yaml)
float32 half_width      # половина ширины, м
float32 height          # высота от головки рельса, м
float32 nose_offset     # смещение от лидара до носа поезда, м
float32 max_range       # предел проверки, м

# Двумерный профиль габарита (вид спереди) для отрисовки
geometry_msgs/Polygon profile_2d

# Динамика поезда
float32 train_speed_mps
float32 braking_distance_m
