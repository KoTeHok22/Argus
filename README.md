# Argus

Инструменты обработки пространственных данных и анализа последовательностей измерений.

Проект включает конвейер обработки данных, оценку геометрических отклонений, визуализацию и средства пакетной проверки. Подробности о входных данных и демонстрационные сценарии намеренно не публикуются.

## Быстрый старт

```bash
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
docker run --rm --shm-size=256m \
     -v "$PWD/data":/data argus \
     detect /data/<recording>
```

`--shm-size=256m` может потребоваться для крупных сообщений при запуске в Docker.

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

Входной канал определяется по метаданным записи.

```bash
docker run --rm --shm-size=256m -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/data":/data argus demo /data/<recording>
```

RViz показывает обработанные данные и диагностическую визуализацию.

Без Docker (ROS 2 Humble):

```bash
./scripts/fetch_third_party.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch argus_launch argus.launch.py \
    bag:=/path/to/recording \
    lidar_topic:=/lidar_points
```

## Что система делает

1. Читает `PointCloud2` по фактической раскладке `fields[]`.
2. Отбрасывает мусор и помечает лучи без возврата.
3. Режет пол и рельсы порогом по Z в колее (Patchwork++ остаётся опцией).
4. Ищет геометрические отклонения и анализирует их во времени.
5. Формирует результаты и диагностические события.

Карта свободного пространства и 4DoF-одометрия есть, но голос карты по умолчанию выключен: на платформе он даёт ложные тревоги, а детекция F-D от него не зависит.

## Что проверено

| Данные | Результат |
|---|---|
| Контрольный набор A | 197 событий в тестовом прогоне |
| Контрольный набор B | Ложных событий не зарегистрировано |
| Дополнительные тестовые записи | Проверены разные режимы движения и условия съёмки |
| Дальние тестовые цели | Исследовательские результаты не включены в штатный режим |

Конфиг: `argus_launch/config/argus_params.yaml`, SHA-256 `9352a301b50514745a43b531debde2f6e27e21176c84554a3c1024f55906a344`.

В тестовом запуске обработка выполнялась в реальном времени; расширенные метрики и параметры наборов данных не публикуются.

## Габарит и предел дальности

Габарит задаётся в `argus_launch/config/argus_params.yaml`. UI сохраняет выбранный блок `gauge` в полном snapshot YAML и передаёт его в ROS-прогон; offline CLI принимает те же геометрические параметры. Полный профиль учитывает ширину, высоту, скос, нос, максимальную дальность и зазор. `safety_margin` не расширяет боковую ширину: это сознательное правило против ложных тревог на платформе.

Пределы применимости зависят от геометрии сцены, полноты измерений и качества локализации. Исследовательские режимы не включены в штатный конвейер.

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
