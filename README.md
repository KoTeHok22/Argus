# Argus

Посторонний объект в габарите поезда по облаку 3D-лидара.

Инструмент обработки пространственных данных.

На каждом кадре система отвечает: есть ли препятствие в габарите 3×2.1 м и на каком расстоянии. Не нейросеть и не детектор «объектов вообще»: модель нормы тоннеля и отклонения от неё.

## Быстрый старт

```bash
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
docker run --rm --shm-size=256m \
    -v "$PWD/data/recordings":/data argus \
    detect /data/doubleT_obstacle
```

`--shm-size=256m` обязателен. Кадр Hesai ~24 МБ; без флага Fast-DDS не отдаёт `PointCloud2`, fusion уходит в `STATUS_DEGRADED` и тревоги нет.

Панель в браузере (загрузка `.zst`, эфир облака, отчёт):

```bash
docker run --rm --shm-size=256m -p 8080:8080 \
    -v "$PWD/data":/data \
    -v "$PWD/results":/ws/results \
    argus ui
```

Открыть `http://localhost:8080`. Без Docker:

```bash
./scripts/run_ui.sh
```

Windows:

```powershell
$env:PYTHONPATH="argus_web"; python -m argus_web
```

Ожидаемый лог на `doubleT_obstacle`: `ALERT BLOCKED` около 16.9 м. Топик лидара читается из `metadata.yaml` (у этого бэга он не `/lidar_points`).

```bash
docker run --rm --shm-size=256m -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/data/recordings":/data argus demo /data/doubleT_obstacle
```

RViz показывает `/argus/cloud`, габарит и бокс 16.9 м. Запись экрана: `docs/video/argus_demo.mp4`.

Без Docker (ROS 2 Humble):

```bash
./scripts/fetch_third_party.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch argus_launch argus.launch.py \
    bag:=data/recordings/doubleT_obstacle \
    lidar_topic:=/sensing/lidar/hesai128/pointcloud
```

## Что система делает

1. Читает `PointCloud2` по фактической раскладке `fields[]`.
2. Отбрасывает мусор и помечает лучи без возврата.
3. Режет пол и рельсы порогом по Z в колее (Patchwork++ остаётся опцией).
4. Ищет геометрический остаток относительно профиля стен и серии дыр в возвратах.
5. Оставляет только то, что попало в габарит поезда, и подтверждает треком.

Карта свободного пространства и 4DoF-одометрия есть, но голос карты по умолчанию выключен: на платформе он даёт ложные тревоги, а детекция F-D от него не зависит.

## Что проверено

| Данные | Результат |
|---|---|
| `doubleT_obstacle`, 200 кадров | 197 тревог с кадра 3, 16.8 м впереди |
| `doubleT_platform`, 200 кадров | 0 тревог |
| `new_data` голова `_0`/`_1` | узкий тоннель, трогание, 0 тревог |
| `new_data` `_29`…`_31` | стоянка у широкого сечения, 0 тревог |
| `new_data` `_110`/`_111` | 5 тревог на 5 м — плоский край, не вклейка |

Конфиг: `argus_launch/config/argus_params.yaml`, SHA-256 `9352a301b50514745a43b531debde2f6e27e21176c84554a3c1024f55906a344`.

Живой `detect` на полном `doubleT_obstacle` (201 кадр): **198 BLOCKED** на 16.9 м, путь тревоги **28.5 мс / p95 32 мс**. Офлайн-одометрия на platform: 87 мс/кадр.

## Предел дальности

Подтверждённая тревога — **16.9 м** на реальном препятствии. Конфиг режет цель дальше **45 м** (`detector_geometry.max_target_range_m`). Поднимать этот порог нечего: на прямом участке объект в проёме оказывается дальше боковых стен, и признак «ближе локальной стены» даёт отрицательный остаток. Синтетическая тележка на 60 м это показала: стена за ней есть, остаток отрицательный, тревоги нет. Генератор вклейки не создаёт точки в пустых лучах, поэтому 150–200 м им не измеряются. Это предел метода, не недокрученный порог. Подробности: `docs/EXPERIMENTS.md`.

## Документы

| Документ | О чём |
|---|---|
| [`docs/ALGORITHM.md`](docs/ALGORITHM.md) | Как устроен конвейер |
| [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md) | Что пробовали и что отвергли |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Пакеты и граф нод |
| [`docs/DATA_FORMAT.md`](docs/DATA_FORMAT.md) | Раскладка PointCloud2 |
| [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) | Сборка и запуск |
| [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) | Частые отказы |

## Требования

Ubuntu 22.04, ROS 2 Humble, Docker. GPU не нужен.
