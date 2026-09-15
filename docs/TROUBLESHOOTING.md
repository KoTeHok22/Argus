# Troubleshooting

## `раскладка отвергнута: missing x/y/z float32 fields`

Сообщение пришло с неожиданной раскладкой полей. Система не угадывает.
Проверьте топик: у бэга `doubleT_obstacle` вход —
`/sensing/lidar/hesai128/pointcloud`, у остальных — `/lidar_points`.

## `data buffer smaller than width*height*point_step`

Усечённый или битый bag. Сверьте размер с `ros2 bag info`.

## RViz2 не показывает облако

1. Проверьте Fixed Frame: должен совпадать с `frame_id` сообщения
   (`hesai_lidar` либо `lidar_livox`).
2. Проверьте топик: `ros2 topic hz /argus/clean`.

## `colcon build` падает на argus_core

Требуются `libeigen3-dev`, `libpcl-dev`. В dev-образе всё установлено;
вне Docker — `rosdep install --from-paths src --ignore-src -y`.

## Данные не в `data/`

Датасет и материалы проекта не хранятся в git. Содержимое `data/` —
см. README.
