#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
source install/setup.bash

BAG="${1:-Datas/dataset/for_hackathon/doubleT_obstacle_smoke}"

cleanup() {
    [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null || true
    pkill -f argus_ 2>/dev/null || true
}
trap cleanup EXIT

echo "[fault-injection] launch: bag=$BAG"
ros2 launch argus_launch argus.launch.py bag:="$BAG" rate:=2.0 > /tmp/argus_fi_launch.log 2>&1 &
LAUNCH_PID=$!

wait_health() {
    local want="$1" tries="${2:-30}"
    for _ in $(seq 1 "$tries"); do
        local got
        got=$(timeout 2 ros2 topic echo /argus/system_health --once 2>/dev/null | head -1 || true)
        if [[ "$got" == *"$want"* ]]; then
            echo "$got"
            return 0
        fi
        sleep 1
    done
    echo "TIMEOUT waiting for '$want'" >&2
    return 1
}

echo "[fault-injection] шаг 1: все узлы живы"
wait_health "OK" 30
echo "[fault-injection] OK: система здорова"

echo "[fault-injection] шаг 2: убиваю argus_detector"
pkill -f argus_detector_node
HEALTH=$(wait_health "DEAD:" 20)
echo "[fault-injection] OK: watchdog увидел отказ: $HEALTH"
[[ "$HEALTH" == *"argus_detector"* ]] || { echo "FAIL: в списке мёртвых нет argus_detector" >&2; exit 1; }

echo "[fault-injection] шаг 3: fusion публикует DEGRADED"
DEGRADED_SEEN=0
for _ in $(seq 1 20); do
    STATUS=$(timeout 2 ros2 topic echo /argus/obstacles --once 2>/dev/null | grep -m1 'status:' | awk '{print $2}' || true)
    if [ "$STATUS" = "3" ]; then
        DEGRADED_SEEN=1
        break
    fi
    sleep 1
done
[ "$DEGRADED_SEEN" = "1" ] || { echo "FAIL: fusion не выдал STATUS_DEGRADED=3" >&2; exit 1; }
echo "[fault-injection] OK: STATUS_DEGRADED подтверждён"

echo "[fault-injection] PASS"
