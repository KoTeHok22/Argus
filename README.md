# Argus

Посторонний объект в контролируемой зоне объект-носительа по облаку 3D-сенсора.

Инструмент обработки пространственных данных.

На каждом кадре система отвечает: есть ли препятствие в настроенном контролируемой зоне объект-носительа и на каком расстоянии. Стандартный профиль — 3×2.1 м. Не нейросеть и не детектор «объектов вообще»: модель нормы окружения и отклонения от неё.

## Быстрый старт

```bash
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
docker run --rm --shm-size=256m \
    -v "$PWD/data/recordings":/data argus \
    detect /data/scn-1
```

`--shm-size=256m` обязателен. Кадр сенсора ~24 МБ; без флага Fast-DDS не отдаёт `PointCloud2`, fusion уходит в `STATUS_DEGRADED` и тревоги нет.

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

Ожидаемый лог на `scn-1`: `ALERT BLOCKED` около 16.9 м. Топик сенсора читается из `metadata.yaml` (у этого бэга он не `топик сенсора`).

```bash
docker run --rm --shm-size=256m -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/data/recordings":/data argus demo /data/scn-1
```

RViz показывает `/argus/cloud`, контролируемая зона и бокс 16.9 м. Запись экрана: `docs/video/argus_demo.mp4`.

Без Docker (ROS 2 Humble):

```bash
./scripts/fetch_third_party.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch argus_launch argus.launch.py \
    bag:=data/recordings/scn-1 \
    sensor_topic:=/sensing/sensor/сенсор/pointcloud
```

## Что система делает

1. Читает `PointCloud2` по фактической раскладке `fields[]`.
2. Отбрасывает мусор и помечает лучи без возврата.
3. Режет пол и направляющие порогом по Z в колее (Patchwork++ остаётся опцией).
4. Ищет геометрический остаток относительно профиля стен и серии дыр в возвратах.
5. Оставляет только то, что попало в контролируемая зона объект-носительа, и подтверждает треком.

Карта свободного пространства и 4DoF-одометрия есть, но голос карты по умолчанию выключен: на `scn-2` он даёт ложные тревоги, а детекция F-D от него не зависит.

## Что проверено

| Данные | Результат |
|---|---|
| `scn-1`, 200 кадров | 197 тревог с кадра 3, 16.8 м впереди |
| `scn-2`, 200 кадров | 0 тревог |
| `cap` голова `_0`/`_1` | узкий окружениь, трогание, 0 тревог |
| `cap` `_29`…`_31` | стоянка у широкого сечения, 0 тревог |
| `cap` `_110`/`_111` | 5 тревог на 5 м — плоский край, не вклейка |
| Диагностические цели 20/60/100 м | Сигнал иногда выделяется после прогрева, но не принят для production |
| World-space режим с неверной кривой оси | До 8/100 диагностических тревог на scn-2; защита от стены на повороте не доказана |

Конфиг: `argus_launch/config/argus_params.yaml`, SHA-256 `9352a301b50514745a43b531debde2f6e27e21176c84554a3c1024f55906a344`.

Живой `detect` на полном `scn-1` (201 кадр): **198 BLOCKED** на 16.9 м, путь тревоги **28.5 мс / p95 32 мс**. Офлайн-одометрия на scn-2: 87 мс/кадр.

## контролируемая зона и предел дальности

контролируемая зона задаётся в `argus_launch/config/argus_params.yaml`. UI сохраняет выбранный блок `gauge` в полном snapshot YAML и передаёт его в ROS-прогон; offline CLI принимает те же геометрические параметры. Полный профиль учитывает ширину, высоту, скос, нос, максимальную дальность и зазор. `safety_margin` не расширяет боковую ширину: это сознательное правило против ложных тревог на `scn-2`.

Подтверждённая production-тревога — **16.9 м** на реальном препятствии. Штатный геометрический признак ограничен 45 м (`detector_geometry.max_target_range_m`). Экспериментальный world-space режим после T58–T67 иногда выделяет синтетические цели на 20/60/100 м, но это не рабочий recall. T68 показал, что формально допустимая, но неверная кривая оси даёт до **8/100** диагностических тревог на scn-2 и **2/100** на неразмеченном `cap-23`. Поэтому world-space режим остаётся офлайн-исследованием и не подключён к ROS. Подробности: `docs/range_feasibility_report.md` и `docs/EXPERIMENTS.md`.

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
