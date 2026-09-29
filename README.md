# Argus

Обнаружение препятствий в габарите поезда по 3D-лидару в железнодорожном тоннеле.

Система читает облако точек, вычитает пол и рельсы, ищет геометрические отклонения от модели тоннеля и подтверждает их во времени. Работает как ROS 2 конвейер и как автономный офлайн-детектор.

## Быстрый старт

Одна команда поднимает панель в браузере:

```bash
docker compose -f docker/docker-compose.yml up --build ui
```

Открыть `http://localhost:8080`. Первый запуск собирает образ — базовый `osrf/ros` тянется из сети, дальше всё из кеша.

Прогон записи детектором:

```bash
docker compose -f docker/docker-compose.yml run --rm argus \
    detect /data/<recording>
```

Сервис `argus` монтирует `data/recordings` как `/data`, поэтому путь внутри контейнера начинается сразу с имени записи.

Без compose:

```bash
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
docker run --rm --shm-size=256m \
     -v "$PWD/data/recordings":/data \
     argus detect /data/<recording>
```

`--shm-size=256m` нужен для крупных сообщений: кадр лидара ~24 МБ.

Панель без Docker — `./scripts/run_ui.sh`; в Windows:

```powershell
$env:PYTHONPATH="argus_web"; python -m argus_web
```

## Режимы запуска

Контейнер принимает команду первым аргументом:

| Команда | Что делает |
|---|---|
| `detect <recording> [rate] [params]` | Проигрывает запись, печатает тревоги, выходит по концу записи |
| `demo <recording>` | То же плюс RViz2 |
| `eval <recording>` | Прогон оценки и отчёт |
| `ui` | Панель в браузере на порту 8080 |
| `shell` | Оболочка внутри образа |
| `help` | Справка (по умолчанию) |

Топик лидара читается из `metadata.yaml` записи, вручную задавать не нужно.

## Сборка из исходников (ROS 2 Humble)

```bash
./scripts/fetch_third_party.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch argus_launch argus.launch.py bag:=/path/to/recording
```

`argus.launch.py` принимает: `bag`, `lidar_topic` (по умолчанию `/lidar_points`), `rate`, `loop`, `params_file`, `rviz`, `publish_markers`.

Разработка и CI идут в отдельном образе с полным набором инструментов:

```bash
docker compose -f docker/docker-compose.dev.yml run --rm dev bash
```

## Проверки

```bash
scripts/ci/all.sh      # lint + build + test
```

В dev-контейнере: `colcon test` + `colcon test-result --verbose`. На текущем дереве проходит **173 теста, 0 ошибок, 0 падений** (148 unit-кейсов в `argus_core` и `argus_eval`).

Линт проверяет формат C++ через `clang-format --dry-run --Werror` и отсутствие бинарников данных в git.

## Что система делает

1. Читает `PointCloud2` по фактической раскладке `fields[]`, а не по жёстко заданной.
2. Отбрасывает мусор и помечает лучи без возврата.
3. Режет пол и рельсы порогом по Z в колее. Patchwork++ есть в сборке, но метод по умолчанию — `z_threshold`.
4. Ищет геометрические отклонения: геометрия, отсутствие возврата, профиль тоннеля, свободное пространство.
5. Подтверждает находки во времени и стабилизирует выход через трекинг.
6. Формирует тревогу и диагностические события.

4DoF-одометрия (`method: 4dof_icp`, `lock_lateral`) работает и публикует позу. Карта свободного пространства строится, но её голос в детекцию по умолчанию выключен (`use_free_space: false`): на платформе он даёт ложные тревоги, а детекция от него не зависит.

## Топики

| Топик | Тип | Кто публикует |
|---|---|---|
| `/argus/cloud` | `PointCloud2` | preprocess |
| `/argus/clean` | `CleanCloud` | preprocess |
| `/argus/anom_g`, `/argus/anom_nr`, `/argus/anom_temporal`, `/argus/anom_fs` | `AnomalySet` | detector |
| `/argus/obstacles` | `ObstacleArray` | fusion |
| `/argus/pose` | `PoseStamped` | odometry |
| `/argus/model` | `PointCloud2` | tunnel_model |
| `/argus/markers` | `MarkerArray` | fusion |
| `/argus/explain` | `Diagnostics` | fusion |
| `/argus/diagnostics*` | `Diagnostics` | каждая нода |

Входной лидарный топик определяется по метаданным записи.

## Габарит

Габарит задаётся блоком `gauge` в `argus_launch/config/argus_params.yaml`: `half_width`, `height`, `nose_offset`, `sensor_height`, `max_range`, `safety_margin`, `ground_clearance_m`, `marker_max_range_m`.

UI сохраняет выбранный блок `gauge` в полный snapshot YAML и передаёт его в ROS-прогон; офлайн-CLI принимает те же геометрические параметры. `safety_margin` не расширяет боковую ширину — это сознательное правило против ложных тревог на платформе.

## Структура репозитория

| Путь | Назначение |
|---|---|
| `argus_core` | Алгоритмы и C++ тесты, плюс CLI `offline_detector` |
| `argus_msgs` | ROS 2 сообщения |
| `argus_node_*` | Ноды: preprocess, odometry, tunnel_model, detector, fusion |
| `argus_launch` | Launch-файлы, конфиги, RViz |
| `argus_eval` | Метрики и оценка |
| `argus_web` | Панель в браузере |
| `docker` | Образы, compose, точка входа |
| `scripts` | Сборка, CI, синтетика |
| `third_party` | `argus.repos`; код patchworkpp подтягивается скриптом |

## Документы

| Документ | О чём |
|---|---|
| [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md) | Что пробовали, что отвергли и почему |

## Ограничения

Результаты зависят от геометрии сцены, полноты измерений и качества локализации. Исследовательские режимы в штатный конвейер не включены. Демонстрационные сценарии и параметры наборов данных намеренно не публикуются.

## Требования

Ubuntu 22.04, ROS 2 Humble, Docker. GPU не нужен.
