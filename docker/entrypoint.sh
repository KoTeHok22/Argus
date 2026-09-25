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
    BAG="${1:?usage: detect <bag_dir> [rate] [params_file]}"
    RATE="${2:-1.0}"
    PARAMS_FILE="${3:-${ARGUS_PARAMS_FILE:-}}"
    TOPIC="$(bag_topic "$BAG")"
    if [ "$RATE" = "0" ]; then
      RATE="1"
    fi
    export ARGUS_LATENCY_CSV="${ARGUS_LATENCY_CSV:-/tmp/argus_latency.csv}"
    PLAYER="${ARGUS_PLAYER:-py}"
    echo "bag=$BAG topic=$TOPIC rate=$RATE player=$PLAYER csv=$ARGUS_LATENCY_CSV"
    python3 /opt/argus/detect_watch.py &
    WATCH_PID=$!
    if [ -n "$PARAMS_FILE" ]; then
      setsid ros2 launch argus_launch argus.launch.py lidar_topic:="$TOPIC" params_file:="$PARAMS_FILE" &
    else
      setsid ros2 launch argus_launch argus.launch.py lidar_topic:="$TOPIC" &
    fi
    LAUNCH_PID=$!
    sleep 5
    if [ "$PLAYER" = "cli" ]; then
      ros2 bag play "$BAG" \
          --storage sqlite3 \
          --rate "$RATE" \
          --disable-keyboard-controls \
          --disable-loan-message \
          --read-ahead-queue-size 8
    else
      python3 /opt/argus/bag_play.py "$BAG" --topic "$TOPIC" --rate "$RATE"
    fi
    sleep 5
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

  ui)
    export ARGUS_ROOT="${ARGUS_ROOT:-/ws}"
    export ARGUS_DATA="${ARGUS_DATA:-/data}"
    export ARGUS_RESULTS="${ARGUS_RESULTS:-/ws/results}"
    export ARGUS_UI_ROOT="${ARGUS_UI_ROOT:-/ws/data/ui}"
    mkdir -p "$ARGUS_RESULTS" "$ARGUS_UI_ROOT"
    export PYTHONPATH="/ws/argus_web${PYTHONPATH:+:$PYTHONPATH}"
    exec python3 -m argus_web --host 0.0.0.0 --port "${ARGUS_WEB_PORT:-8080}"
    ;;

  shell)
    exec bash
    ;;

  help|*)
    cat <<'EOF'
Argus — препятствие в габарите поезда по 3D-лидару.

  docker build -f docker/Dockerfile -t argus .
  docker run --rm --shm-size=256m -v /path/to/bags:/data argus detect /data/<bag_dir>
  docker run --rm --shm-size=256m -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix argus demo /data/<bag_dir>

  detect  прогон bag, печать /argus/obstacles, выход после конца записи
  demo    то же плюс RViz2: /argus/cloud, габарит и бокс на /argus/markers, карта на /argus/model
  eval    каркас отчёта
  ui      панель: загрузка записи, эфир, отчёт (порт 8080)
  shell   оболочка внутри образа

rate по умолчанию 1 (реальное время). --shm-size=256m обязателен: кадр ~24 МБ.
Топик лидара читается из metadata.yaml.

Пример:
  docker run --rm --shm-size=256m \
      -v "$PWD/data/recordings":/data argus \
      detect /data/doubleT_obstacle 0

  docker run --rm --shm-size=256m -p 8080:8080 \
      -v "$PWD/data/recordings":/data argus ui
EOF
    ;;
esac
