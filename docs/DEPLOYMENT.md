# Развёртывание

## Требования

- Docker (любая ОС хоста)
- Целевая среда: Ubuntu 22.04, ROS 2 Humble
- GPU опционален (NVIDIA + nvidia-container-toolkit)

## Сборка

```bash
docker build -f docker/Dockerfile -t argus .
```

## Запуск детекции

```bash
docker run --rm -v "$PWD/data/recordings":/data argus \
    detect /data/roundT_doubleT
```

## С визуализацией

```bash
docker run --rm -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/data/recordings":/data argus \
    demo /data/roundT_doubleT
```

## Без Docker (dev)

```bash
# Зависимости
./scripts/fetch_third_party.sh

# Сборка
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Запуск
source install/setup.bash
ros2 launch argus_launch argus.launch.py bag:=<bag_dir>
```

## Параметры

Все параметры — в `argus_launch/config/argus_params.yaml` (единая точка
истины). Назначение каждого — комментарий в файле.

## Сторонние компоненты

| Проект | Лицензия |
|---|---|
| KISS-ICP | MIT |
| Patchwork++ | BSD-2-Clause |
| Bonxai | MPL-2.0 |
| DUFOMap (опционально) | BSD-3-Clause |
| rerun, Lichtblick | MIT/MPL-2.0 |
| OpenVDB/PCL/Eigen | MPL-2.0/BSD |

Запрещены к линковке: GPL-компоненты (ERASOR, TRAVEL) и код без лицензии
(Removert) — см. `PLAN.md` §18.6.
