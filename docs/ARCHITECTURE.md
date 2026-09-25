# Архитектура

## Общая схема

```
ROS 2 bag (sqlite3)          ┌──────────────────────────── argus_node_preprocess ───────────────────────────┐
  PointCloud2  ─────────────▶│ cloud_filter → ground → range_image → no_return_mask → sensor_health      │
                             └───────────────┬──────────────────────────────────────────────────────────┘
                                             │ CleanCloud + RangeImage + Diagnostics
              ┌──────────────────────────────┼───────────────────────────────┐
              ▼                              ▼                               ▼
   argus_node_detector           argus_node_odometry             argus_node_tunnel_model
   geometry / no_return /       4DoF ICP, quality gate           воксельная карта нормы +
   temporal residual             → pose (только quality_ok)      free-space
              │                              │                               │
              └──────────────┬───────────────┴───────────────────────────────┘
                             ▼
                    argus_node_fusion
        clustering → obstacle tracker → FusionPipeline → supervision/gauge_state
                             │
        ┌────────────────────┼───────────────────────┬────────────────────┐
        ▼                    ▼                       ▼                    ▼
  /argus/obstacles      /argus/gauge_state     /argus/system_health   RViz2 / UI
                                                        ▲
                                          argus_node_watchdog (heartbeat 2 Гц)
                                                        ▲
                                          argus_node_recorder (JSONL решений)
```

## Пакеты

| Пакет | Роль |
|---|---|
| `argus_core` | Вся алгоритмика на C++: фильтр, ground, range image, детекторы, кластеризация, трекинг, fusion, одометрия, модель тоннеля, надзор, классификация. CLI `offline_detector`, тесты GoogleTest |
| `argus_msgs` | Сообщения ROS 2: `Obstacle`, `ObstacleArray`, `Diagnostics`, `GaugeState` |
| `argus_node_preprocess` | Очистка облака, вычитание пола/рельсов, построение range image, маска «нет возврата», здоровье сенсора |
| `argus_node_detector` | Геометрический residual, no-return, временной residual |
| `argus_node_odometry` | Одометрия 4DoF ICP с гейтом качества |
| `argus_node_tunnel_model` | Воксельная карта нормального тоннеля и детектор free-space |
| `argus_node_fusion` | Кластеризация, трекинг, fusion, маркеры, gauge state, надзор скорости |
| `argus_node_watchdog` | Heartbeat всех нод и публикация `/argus/system_health` |
| `argus_node_recorder` | JSONL-журнал решений с ротацией |
| `argus_launch` | Launch-файлы и `config/argus_params.yaml` |
| `argus_eval` | Метрики и отчёт по прогону |
| `argus_web` | Браузерная панель: загрузка записи, эфир облака, отчёт |
| `docker` | Multi-stage образ, `entrypoint.sh` с режимами, FastDDS-профиль |

## Потоки данных и топики

| Топик | Тип | Кто публикует |
|---|---|---|
| `/lidar_points` (или из `metadata.yaml`) | `PointCloud2` | bag |
| `/argus/cloud` | `PointCloud2` | preprocess (для RViz) |
| `/argus/obstacles` | `ObstacleArray` | fusion |
| `/argus/gauge_state` | `GaugeState` | fusion |
| `/argus/system_health` | `Diagnostics` | watchdog |
| `/argus/heartbeat` | `Diagnostics` | все ноды (2 Гц) |
| `/argus/markers`, `/argus/model` | `MarkerArray`, `PointCloud2` | fusion |

## Синхронизация кадров

`argus_core/frame_sync.cpp` (`FrameSynchronizer`) сопоставляет каналы одного кадра по
штампу в допуске 30 мс и позу по ближайшему штампу в допуске 100 мс. Это устранило
массовые DEGRADED из-за точного совпадения штампов. Истёкшие неполные кадры считаются
по каналам (`sync: arrivals/snapped/expired …`).

## Одометрия

`TunnelOdometry` — point-to-plane ICP, 4 степени свободы (курс, продольный и поперечный
сдвиг, высота; блокируются крен и рыскание). Поза публикуется только при
`quality_ok`: `fitness ≤ max_fitness` и число соответствий ≥ `min_correspondences_quality`.
При плохой позе world-space признаки отключаются, система не строит тревогу на
недостоверной геометрии.

## Модель нормального тоннеля

`TunnelModel` — воксельная карта занятости со счётчиком наблюдений. Воксель считается
свободным, если луч прошёл через него `free_hits_to_clear` раз, и занятым при
`min_observations` попаданиях. `FreeSpaceDetector` ищет точки, противоречащие
подтверждённому свободному пространству. В production этот голос выключен
(`fusion.use_free_space: false`), потому что на платформе давал ложные тревоги.

## Принятие решения

`FusionPipeline::update` собирает кандидатов из геометрического, no-return и (опц.)
временного и free-space каналов, группирует их кластеризацией, подтверждает трекингом
и проверяет принадлежность габариту поезда. Тревога требует подтверждённого трека
(`require_confirmed_track`) и свежести дальнего трека (после 4 пропусков на 90 м+
трек перестаёт тревожить).

## Отказоустойчивость (fail-safe)

- `argus_node_watchdog` публикует `DEAD:<узлы>`, если heartbeat узла пропал;
  fusion понижает `CLEAR` до `DEGRADED` и называет молчащий узел.
- `SensorHealthMonitor` следит за долей «нет возврата» и валидных точек; загрязнение
  лидара не даёт молчаливой слепоты.
- При отсутствии кадров публикуется `clear_range=0`, `speed_limit=0`.
- `scripts/fault_injection_test.sh` — живой E2E: kill детектора → `DEAD:argus_detector`
  → `STATUS_DEGRADED` (PASS).

## Развёртывание

Один Docker-образ, точка входа `docker/entrypoint.sh` с режимами `detect`, `demo`,
`eval`, `ui`, `shell`, `help`. Подробности — `docs/DEPLOYMENT.md`.
