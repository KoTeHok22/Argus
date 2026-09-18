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

Скрипты разведки, метаданные бэгов и материалы проекта лежат в git.
Бинарники бэгов (`.db3`, `.zst`, `.head`) не коммитятся — они больше
лимита GitHub; их нужно положить в `data/` вручную.

## `/argus/anom_fs` молчит

Детектор свободного пространства публикуется из `argus_tunnel_model`, и
ему нужны три вещи одновременно:

1. `/argus/pose` — без одометрии кадры не обрабатываются;
2. прогретая карта — до `detector_free_space.warmup_frames` кадров голос
   не подаётся намеренно;
3. луч, реально прошедший сквозь клетку в прошлых кадрах.

Проверьте `ros2 topic hz /argus/pose` и `ros2 topic echo
/argus/diagnostics_model --field model_voxel_count`.

## `STATUS_DEGRADED` на fusion

Fusion ждёт `clean` + `anom_g` + `anom_nr` с одинаковым stamp. Если
preprocess молчит (не тот топик лидара), fusion уходит в деградацию:
`ros2 topic hz /argus/clean`.

## Ложные срабатывания на платформе

Платформа — законная геометрия рядом с габаритом. Если тревоги идут на `doubleT_platform`, проверьте:

1. `gauge.half_width` — боковой габарит не должен захватывать платформенный край (`safety_margin` его больше не расширяет);
2. `ground_segmentation.rail_max_height` — рельсовая и платформенная структура в колее до 0.55 м маскируется как земля;
3. `fusion.use_free_space` — на платформе признак свободного пространства самый шумный, его можно выключить без потери основного детектора.

