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
    detect /data/doubleT_obstacle 0
```

`--shm-size=256m` обязателен: кадр лидара ~24 МБ, стандартных 64 МБ `/dev/shm` в Docker не хватает. Без флага облако не доходит, fusion пишет `STATUS_DEGRADED`.

Второй аргумент — скорость bag (`1` — реальное время). Топик лидара берётся из `metadata.yaml`. На `doubleT_obstacle` это `/sensing/lidar/hesai128/pointcloud`.

Выход: строки `ALERT BLOCKED` с дистанции и в конце `frames=… blocked_alerts=…`.

## С визуализацией

```bash
docker run --rm --shm-size=256m -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/data/recordings":/data argus \
    demo /data/doubleT_obstacle
```

## Без Docker

```bash
./scripts/fetch_third_party.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch argus_launch argus.launch.py \
    bag:=data/recordings/doubleT_obstacle \
    lidar_topic:=/sensing/lidar/hesai128/pointcloud
```

Для остальных бэгов `recordings` топик по умолчанию `/lidar_points`.

## Параметры

Все пороги — в `argus_launch/config/argus_params.yaml`. Хэш сдачи: SHA-256 `c401b96d401d0d7db5348dd3effe7b6dcd1ecccf0bbf0c53d206f77092efefbb`.

## Сторонние компоненты

| Проект | Лицензия | В проде |
|---|---|---|
| Patchwork++ | BSD-2-Clause | Да, опция земли. По умолчанию порог по Z |
| Eigen, PCL | MPL-2.0 / BSD | Да |
| KISS-ICP | MIT | Нет, свой 4DoF ICP |
| Bonxai | MPL-2.0 | Нет, своя блочная карта |

GPL (ERASOR, TRAVEL) и код без лицензии (Removert) не линкуются.
