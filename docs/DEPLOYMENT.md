# Сборка, запуск и параметры

## Требования

Ubuntu 22.04, ROS 2 Humble, Docker. GPU не нужен — всё работает на CPU.

## Сборка

```bash
./scripts/fetch_third_party.sh          # подтягивает patchwork-plusplus по argus.repos
docker build -f docker/Dockerfile -t argus .
```

## Режимы контейнера

```bash
docker run --rm --shm-size=256m -v /path/to/bags:/data argus detect /data/<bag> [rate] [params.yaml]
docker run --rm --shm-size=256m -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix argus demo /data/<bag>
docker run --rm --shm-size=256m -v /path/to/bags:/data argus eval /data/<bag>
docker run --rm --shm-size=256m -p 8080:8080 -v /path/to/bags:/data argus ui
docker run --rm argus shell
```

| Команда | Назначение |
|---|---|
| `detect <bag> [rate] [params]` | Прогон записи, печать `/argus/obstacles`, выход после конца записи |
| `demo <bag>` | То же плюс RViz2 |
| `eval <bag>` | Отчёт по прогону |
| `ui` | Браузерная панель на порту 8080 |
| `shell` | Оболочка внутри образа |
| `help` | Справка (по умолчанию) |

`--shm-size=256m` обязателен: один кадр занимает ~24 МБ. Топик лидара читается из
`metadata.yaml`.

## Панель без Docker

```bash
./scripts/run_ui.sh
# Windows:
$env:PYTHONPATH="argus_web"; python -m argus_web
```

## Проверки

```bash
scripts/ci/all.sh      # lint + build + test
```

## Параметры (`argus_launch/config/argus_params.yaml`)

Значения ниже — production-профиль. Офлайн CLI `offline_detector` принимает те же
параметры флагами (`--gauge-height`, `--geom-max-range`, `--ground-filter`, …).

### `cloud`
| Параметр | Значение | Смысл |
|---|---|---|
| `input_topic` | `/lidar_points` | входной топик (переопределяется из bag) |
| `qos_depth` | 10 | глубина очереди подписки |
| `min_range` / `max_range` | 0.5 / 400.0 | диапазон валидной дальности |
| `max_abs_coord` | 1.0e4 | порог мусорных координат |
| `drop_zero_xyz` | true | отбрасывать `(0,0,0)` |
| `drop_nonfinite` | true | отбрасывать `nan`/`inf` |
| `echo_multiplier` | 0 | множитель эха (0 = авто) |

### `range_image`
| Параметр | Значение | Смысл |
|---|---|---|
| `rings` | 0 | число колец (0 = определить автоматически) |
| `az_steps` | 0 | число азимутальных шагов (0 = авто) |
| `prefer_ring_field` | true | использовать поле `ring` для раскладки |

### `ground_segmentation`
| Параметр | Значение | Смысл |
|---|---|---|
| `enabled` | true | вычитать пол/рельсы |
| `method` | `z_threshold` | метод (`patchworkpp` — опция) |
| `sensor_height` | −1.075 | высота лидара над рельсом (подтверждена организаторами) |
| `rail_zone_m` | 2.0 | ширина зоны рельсов |
| `rail_max_height` | 0.55 | порог рельса над полом |
| `max_ground_z_rel` | 0.60 | порог пола относительно сенсора |
| `min_range` / `max_range` | 1.0 / 80.0 | зона вычитания земли |
| `forward_axis` | `-y` | направление движения |

### `odometry`
| Параметр | Значение | Смысл |
|---|---|---|
| `enabled` | true | включить одометрию |
| `method` | `4dof_icp` | 4 степени свободы |
| `lock_lateral` / `lock_roll` | true | блокировать поперечный сдвиг и крен |
| `lock_vertical` | false | высота не блокируется |
| `max_range` / `min_range` | 120.0 / 1.0 | зона ICP |
| `voxel_size` | 0.5 | вокселизация перед ICP |
| `max_speed_mps` / `max_accel_mps2` | 30.0 / 2.0 | физические ограничения |
| `max_iterations` | 4 | итераций ICP |
| `max_correspondence_m` | 2.0 | порог соответствия |
| `rest_translation_m` | 0.20 | порог «стоим» |

### `tunnel_model`
| Параметр | Значение | Смысл |
|---|---|---|
| `enabled` | true | строить карту нормы |
| `voxel_size` | 0.20 | размер вокселя |
| `min_observations` | 3 | наблюдений до «занят» |
| `free_hits_to_clear` | 2 | проходов луча до «свободен» |
| `max_range` / `carve_max_range` | 150.0 / 90.0 | дальности карты и высекания |
| `prior_map_path` | `""` | внешняя априорная карта (опция) |
| `process_period_s` | 0.05 | период обновления |

### `sensor_health`
| Параметр | Значение | Смысл |
|---|---|---|
| `max_no_return_ratio` | 0.6 | порог доли «нет возврата» |
| `min_valid_ratio` | 0.10 | минимальная доля валидных точек |
| `valid_drop_factor` | 0.5 | падение относительно baseline |
| `warmup_frames` | 30 | прогрев |
| `bad_frames_to_raise` / `good_frames_to_clear` | 20 / 20 | гистерезис |

### `detector_geometry`
| Параметр | Значение | Смысл |
|---|---|---|
| `median_half_window` | 100 | окно базового профиля |
| `median_window_override_min/max_range_m` | 18.5 / 40.0 | зона отдельного окна |
| `median_half_window_override` | 200 | широкое окно для 20 м |
| `residual_threshold_m` / `_far_m` | 1.0 / 1.0 | порог residual |
| `min_cells` / `min_fill` | 50 / 0.30 | гейты ближней цели |
| `min_range` / `max_target_range_m` | 4.0 / 150.0 | рабочий диапазон |
| `far_range_m` | 60.0 | граница «дальних» |
| `min_cells_far` / `min_fill_far` | 8 / 0.12 | гейты дальней цели |
| `min_stable_frames` | 2 | кадров подтверждения |
| `az_tolerance` / `range_tolerance_m` | 20 / 3.0 | допуски слежения |

### `detector_no_return`
| Параметр | Значение | Смысл |
|---|---|---|
| `min_missing_run` | 8 | минимальная серия пропавших лучей |
| `max_missing_run_deg` | 15.0 | максимальная (шире — стена, не объект) |
| `azimuth_window` | 5 | окно поиска живых лучей по краям |
| `min_baseline_hits` | 0.70 | доля живых лучей у краёв |
| `min_range` | 3.0 | минимальная дальность фона |

### `detector_temporal`
| Параметр | Значение | Смысл |
|---|---|---|
| `window_frames` | 5 | кадров истории |
| `residual_threshold_m` | 0.5 | порог временного residual |
| `min/max_range_m` | 4.0 / 45.0 | рабочий диапазон |
| `max_frame_gap_s` | 0.5 | разрыв, сбрасывающий историю |

### `detector_free_space`
| Параметр | Значение | Смысл |
|---|---|---|
| `enabled` | true | включить детектор |
| `min_confidence` | 0.85 | порог уверенности |
| `min_free_observations` | 10 | наблюдений до «свободен» |
| `warmup_frames` | 60 | прогрев карты |

### `gauge`
| Параметр | Значение | Смысл |
|---|---|---|
| `forward_axis` | `-y` | направление движения |
| `half_width` | 1.50 | полуширина габарита |
| `height` | 2.10 | высота над головкой рельса |
| `base_offset` | 0.20 | нижняя граница |
| `chamfer` | 0.20 | фаска |
| `sensor_height` | 1.075 | высота лидара |
| `safety_margin` | 0.10 | запас (по высоте и носу, не по ширине) |
| `max_range` | 300.0 | максимальная дальность габарита |

### `clustering`
| Параметр | Значение | Смысл |
|---|---|---|
| `neighbor_distance` | 0.35 | радиус соседства |
| `adaptive_scaling` | true | рост радиуса с дальностью |
| `min_cluster_size_near/mid/far/long` | 15 / 6 / 3 / 2 | лесенка размера по дальности |
| `size_*_range_m` | 60 / 120 / 200 / 300 | границы лесенки |
| `min_extent_m` / `max_extent_m` | 0.20 / 5.00 | допустимый размер кластера |

### `tracking`
| Параметр | Значение | Смысл |
|---|---|---|
| `max_association_distance` | 2.0 | радиус ассоциации |
| `min_hits_to_confirm` / `_far` | 3 / 5 | кадров подтверждения |
| `confirm_near/far_range_m` | 60.0 / 120.0 | границы адаптации |
| `max_misses_to_keep` | 4 | пропусков до удаления трека |

### `fusion`
| Параметр | Значение | Смысл |
|---|---|---|
| `use_free_space` | false | голос свободного пространства (выкл. в проде) |
| `use_temporal_candidates` | false | временные кандидаты (выкл. в проде) |
| `require_confirmed_track` | true | тревога только по подтверждённому треку |
| `critical_range_m` / `warning_range_m` | 50.0 / 150.0 | границы статусов |
| `critical_ttc_s` | 3.0 | порог TTC |
| `strict_track_freshness_range_m` | 90.0 | дальность строгой свежести |
| `max_confirmed_track_misses` | 4 | пропусков до снятия тревоги |
| `sync_tolerance_ms` / `pose_tolerance_ms` | 30 / 100 | допуски синхронизации |

### `vehicle` (надзор скорости)
| Параметр | Значение | Смысл |
|---|---|---|
| `speed_topic` | `""` | топик скорости (пусто — из `default_speed_mps`) |
| `default_speed_mps` | 0.0 | скорость по умолчанию |
| `speed_stale_s` | 1.0 | устаревание скорости |
| `braking_decel_mps2` | 1.3 | замедление |
| `reaction_s` | 0.5 | время реакции |
| `braking_margin_m` | 10.0 | запас тормозного пути |

### `watchdog` и `recorder`
| Параметр | Значение | Смысл |
|---|---|---|
| `heartbeat_timeout_s` | 2.0 | таймаут heartbeat |
| `critical_nodes` | preprocess, detector, fusion | узлы, влияющие на CLEAR |
| `recorder.output_dir` | `/tmp/argus_events` | каталог JSONL |
| `recorder.max_file_mb` / `max_files` | 64 / 24 | ротация журнала |

## Полный сценарий ТЗ

```bash
git clone <repo> && cd Argus
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
docker run --rm --shm-size=256m -v "$PWD/Datas/dataset/for_hackathon":/data \
    argus detect /data/<recording> 0
```

Ожидаемый вывод — строки `ALERT BLOCKED` около 16.9 м.
