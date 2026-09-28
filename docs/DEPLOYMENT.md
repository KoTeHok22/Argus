# Развёртывание

## Требования

Docker. Целевая среда — Ubuntu 22.04, ROS 2 Humble. GPU не нужен.

## Сборка

```bash
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
```

Образ не содержит датасет. Patchwork++ копируется из `third_party/patchworkpp` (SHA `3e6903a`). KISS-ICP и Bonxai в образ не входят. Перед сборкой один раз: `./scripts/fetch_third_party.sh`.

## Детекция одной командой

```bash
docker run --rm --shm-size=256m \
    -v "$PWD/data/recordings":/data argus \
    detect /data/scn-1 0
```

`--shm-size=256m` обязателен: кадр сенсора ~24 МБ, стандартных 64 МБ `/dev/shm` в Docker не хватает. Без флага облако не доходит, fusion пишет `STATUS_DEGRADED`.

Второй аргумент — скорость bag (`1` — реальное время). Топик сенсора берётся из `metadata.yaml`. На `scn-1` это `/sensing/sensor/сенсор/pointcloud`.

Выход: строки `ALERT BLOCKED` с дистанции и в конце `frames=… blocked_alerts=…`.

## Панель в браузере

```bash
docker run --rm --shm-size=256m -p 8080:8080 \
    -v "$PWD/data":/data \
    -v "$PWD/results":/ws/results \
    argus ui
```

Или `docker compose -f docker/docker-compose.yml up ui`. Открыть `http://localhost:8080`.

На экране «Запись» можно положить `.zst` или выбрать уже распакованный bag. «Эфир» рисует облако из sqlite bag, без ROS. «Прогнать» создаёт snapshot полного YAML с выбранным `gauge` и запускает тот же `detect`; отчёт появляется из CSV.

Без Docker: `./scripts/run_ui.sh` (нужны Python 3 и numpy).

## С визуализацией

```bash
docker run --rm --shm-size=256m -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/data/recordings":/data argus \
    demo /data/scn-1
```

## Без Docker

```bash
./scripts/fetch_third_party.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch argus_launch argus.launch.py \
    bag:=data/recordings/scn-1 \
    sensor_topic:=/sensing/sensor/сенсор/pointcloud
```

Для остальных бэгов `recordings` топик по умолчанию `топик сенсора`.

## Параметры

Все пороги — в `argus_launch/config/argus_params.yaml`. UI-runner передаёт snapshot этого файла через `params_file`. Хэш production-конфига: SHA-256 `9352a301b50514745a43b531debde2f6e27e21176c84554a3c1024f55906a344`.

Диагностические флаги offline world-space runner (`--world-components`, `--track-corridor`) не входят в ROS-команду и не меняют production YAML. Они требуют внешней оси пути, а T68 показал чувствительность к ошибочной кривой оси.

`fusion.publish_markers` (по умолчанию true) включает `/argus/markers`. `/argus/explain` — текст тревоги. `/argus/cloud` — облако для RViz (не сырой топик бэга). Видео G8: `docs/video/argus_demo.mp4`.

## Сторонние компоненты

| Проект | Лицензия | В проде |
|---|---|---|
| Patchwork++ | BSD-2-Clause | Да, опция земли. По умолчанию порог по Z |
| Eigen, PCL | MPL-2.0 / BSD | Да |
| KISS-ICP | MIT | Нет, свой 4DoF ICP |
| Bonxai | MPL-2.0 | Нет, своя блочная карта |

GPL (ERASOR, TRAVEL) и код без лицензии (Removert) не линкуются.
