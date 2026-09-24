# Troubleshooting

## `раскладка отвергнута: missing x/y/z float32 fields`

Сообщение пришло с неожиданной раскладкой полей. Система не угадывает.
Проверьте топик: у бэга `doubleT_obstacle` вход —
`/sensing/lidar/hesai128/pointcloud`, у остальных — `/lidar_points`.

## `data buffer smaller than width*height*point_step`

Усечённый или битый bag. Сверьте размер с `ros2 bag info`.

## RViz2 не показывает облако

1. Проверьте Fixed Frame: `lidar_livox` (кадр бэга `doubleT_obstacle`).
2. Облако для RViz — `/argus/cloud`, не `/argus/clean` и не сырой топик бэга.
   `ros2 topic hz /argus/cloud`. Маркеры: `/argus/markers`.

## `colcon build` падает на argus_core

Требуются `libeigen3-dev`, `libpcl-dev`. В dev-образе всё установлено;
вне Docker — `rosdep install --from-paths src --ignore-src -y`.

## Данные не в `data/`

Скрипты разведки, метаданные бэгов и материалы проекта лежат в git.
Бинарники бэгов (`.db3`, `.zst`, `.head`) не коммитятся — они больше
лимита GitHub; их нужно положить в `data/` вручную.

## Модель тоннеля не успевает за 10 Гц

Стоимость `TunnelModel::integrate` растёт вместе с картой: на `doubleT_platform`
кадр 0 стоит ~0.6 с, к сотому кадру — больше 1 с при 0.2 м вокселе. Причины:

1. `tunnel_model.max_range` = 150 м против `detector_free_space.max_range` = 90 м:
   карта копила клетки, которые детектор не читает;
2. луч идёт по всей длине коридора, включая дальние стены.

Что помогает:

- `tunnel_model.carve_max_range` (по умолчанию 90 м) — не высекать за пределы
  рабочего диапазона детектора;
- `tunnel_model.carve_half_width_m` (по умолчанию 3.0 м) — высекать только
  колею: `|x|` — латераль, стены стоят на ~6.5 м. Это даёт ~2.7× на платформе;
- `tunnel_model.carve_stride_az/_ring` — прореживание лучей с поворотом фазы по
  кадрам (перекрытие набирается за несколько кадров).

Полный прогон `--tunnel` на 300 кадрах платформы после T18 проходит целиком
(блочная сетка). Для сдачи достаточно `--fusion` / `detect`: голос карты выключен.

## `/argus/anom_fs` молчит

Детектор свободного пространства публикуется из `argus_tunnel_model`, и
ему нужны три вещи одновременно:

1. `/argus/pose` — без одометрии кадры не обрабатываются;
2. прогретая карта — до `detector_free_space.warmup_frames` кадров голос
   не подаётся намеренно;
3. луч, реально прошедший сквозь клетку в прошлых кадрах.

Проверьте `ros2 topic hz /argus/pose` и `ros2 topic echo
/argus/diagnostics_model --field model_voxel_count`.

## `detect` молчит, fusion пишет STATUS_DEGRADED

Каждый кадр Hesai ~24 МБ. Без увеличенного `/dev/shm` Fast-DDS не отдаёт облако.

```bash
docker run --rm --shm-size=256m -v /path/to/bags:/data argus detect /data/<bag> 0
```

На Docker Desktop (Windows) SHM часто недоступен: в логе `SHM Transport is not supported`, кадры идут через UDP с задержкой. На Linux-хосте жюри `--shm-size=256m` обязателен.

## `STATUS_DEGRADED` на fusion

Fusion ждёт `clean` + `anom_g` + `anom_nr` с одинаковым stamp. Если
preprocess молчит (не тот топик лидара), fusion уходит в деградацию:
`ros2 topic hz /argus/clean`.

## Ложные срабатывания на платформе

Платформа — законная геометрия рядом с габаритом. Если тревоги идут на `doubleT_platform`, проверьте:

1. `gauge.half_width` — боковой габарит не должен захватывать платформенный край (`safety_margin` его больше не расширяет);
2. `ground_segmentation.rail_max_height` — рельсовая и платформенная структура в колее до 0.55 м маскируется как земля;
3. `fusion.use_free_space` — на платформе признак свободного пространства самый шумный, его можно выключить без потери основного детектора.

## Настройки габарита из UI не влияют на прогон

Проверьте, что прогон запущен кнопкой «Прогнать» после сохранения параметров. UI создаёт полный snapshot YAML рядом с CSV и передаёт его в ROS через `params_file`. `/api/params` должен показывать активный файл, а не только отдельный блок `gauge`. Production default YAML при этом не изменяется.

## World-space эксперимент тревожит на повороте

`--world-components` и `--track-corridor` не являются production-командами. Полилиния должна быть независимой осью пути с оценкой ошибки; гладкая форма и ограничение длины сегментов не доказывают, что она совпадает с рельсами. T68 получил до 8/100 тревог на platform при неверной кривой оси. Не расширяйте `world-max-axis-offset` для компенсации неизвестной траектории.
