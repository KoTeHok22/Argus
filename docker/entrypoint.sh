#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi

MODE="${1:-help}"
shift || true

bag_topic() {
  python3 /opt/argus/bag_topic.py "$1"
}

case "$MODE" in
  detect)
    BAG="${1:?usage: detect <bag_dir> [rate]}"
    RATE="${2:-1.0}"
    TOPIC="$(bag_topic "$BAG")"
    if [ "$RATE" = "0" ]; then
      RATE="1"
    fi
    echo "bag=$BAG topic=$TOPIC rate=$RATE"
    python3 /opt/argus/detect_watch.py &
    WATCH_PID=$!
    setsid ros2 launch argus_launch argus.launch.py lidar_topic:="$TOPIC" &
    LAUNCH_PID=$!
    sleep 5
    ros2 bag play "$BAG" \
        --storage sqlite3 \
        --rate "$RATE" \
        --disable-keyboard-controls \
        --disable-loan-message \
        --read-ahead-queue-size 8
    sleep 2
    kill -INT -"$LAUNCH_PID" 2>/dev/null || true
    kill -INT "$WATCH_PID" 2>/dev/null || true
    sleep 1
    kill -TERM -"$LAUNCH_PID" 2>/dev/null || true
    kill -TERM "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    ;;

  demo)
    BAG="${1:?usage: demo <bag_dir>}"
    TOPIC="$(bag_topic "$BAG")"
    exec ros2 launch argus_launch argus.launch.py \
        bag:="$BAG" lidar_topic:="$TOPIC" rviz:=true
    ;;

  eval)
    BAG="${1:?usage: eval <bag_dir>}"
    exec ros2 run argus_eval run_eval.py --bag "$BAG" --report
    ;;

  shell)
    exec bash
    ;;

  help|*)
    cat <<'EOF'
Argus — препятствие в габарите поезда по 3D-лидару.

  docker build -f docker/Dockerfile -t argus .
  docker run --rm --shm-size=256m -v /path/to/bags:/data argus detect /data/<bag_dir> [rate]

  detect  прогон bag, печать /argus/obstacles, выход после конца записи
  demo    то же плюс RViz2 (нужен DISPLAY)
  eval    каркас отчёта
  shell   оболочка внутри образа

rate=1 — реальное время, rate=0 — то же (кадры ~24 МБ, быстрее не ускоряет).
--shm-size=256m обязателен: облако не проходит через стандартный 64 МБ SHM.
Топик лидара читается из metadata.yaml.

Пример:
  docker run --rm --shm-size=256m \
      -v "$PWD/data/recordings":/data argus \
      detect /data/doubleT_obstacle 0
EOF
    ;;
esac
