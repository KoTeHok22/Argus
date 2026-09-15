#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
#
# Точка входа образа Argus (PLAN.md 12.2).
# Требование ТЗ: docker build -> docker run -> ros2 bag play -> увидеть результат.

set -euo pipefail

source /opt/ros/humble/setup.bash
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi

MODE="${1:-help}"
shift || true

case "$MODE" in
  detect)
    BAG="${1:?usage: detect <bag_dir> [rate]}"
    RATE="${2:-1.0}"
    shift 2 || true
    exec ros2 launch argus_launch argus.launch.py \
        bag:="$BAG" rate:="$RATE" "$@"
    ;;

  demo)
    BAG="${1:?usage: demo <bag_dir>}"
    exec ros2 launch argus_launch argus_demo.launch.py bag:="$BAG"
    ;;

  eval)
    BAG="${1:?usage: eval <bag_dir>}"
    exec ros2 run argus_eval run_eval --bag "$BAG" --report
    ;;

  shell)
    exec bash
    ;;

  help|*)
    cat <<'EOF'
Argus - обнаружение препятствий в тоннеле метро по данным 3D-лидара

Использование:
  docker run --rm -v /path/to/bags:/data argus detect /data/<bag_dir> [rate]
      Детекция на bag-файле. rate=1.0 - реальное время, rate=0 - без задержек.

  docker run --rm -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /path/to/bags:/data argus demo /data/<bag_dir>
      Детекция с визуализацией в RViz2.

  docker run --rm -v /path/to/bags:/data argus eval /data/<bag_dir>
      Оценка и метрики.

  docker run --rm -it argus shell
      Интерактивная оболочка.

Пример:
  docker build -f docker/Dockerfile -t argus .
  docker run --rm -v "$PWD/data/recordings":/data argus \
      detect /data/roundT_doubleT
EOF
    ;;
esac
