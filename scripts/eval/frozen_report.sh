#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
#
# Единый воспроизводимый отчёт по замороженным наборам кадров.
# Запуск: scripts/eval/frozen_report.sh [out_dir]
# Требует собранный argus_core (colcon build) или Docker-образ argus.
set -euo pipefail
cd "$(dirname "$0")/../.."

OUT_DIR="${1:-results/frozen_report}"
mkdir -p "$OUT_DIR"

DETECTOR=""
if [ -x "build/argus_core/offline_detector" ]; then
  export LD_LIBRARY_PATH="build/argus_core:build/argus_msgs:${LD_LIBRARY_PATH:-}"
  DETECTOR="build/argus_core/offline_detector"
elif command -v docker >/dev/null 2>&1; then
  DETECTOR="docker"
else
  echo "нет build/argus_core/offline_detector и нет docker" >&2
  exit 1
fi

run() {
  local frames="$1"; shift
  if [ "$DETECTOR" = "docker" ]; then
    docker run --rm --ipc=host -v "$PWD":/ws -w /ws argus:dev \
      bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && \
                ./build/argus_core/offline_detector $frames $*" 2>/dev/null
  else
    $DETECTOR "$frames" "$@" 2>/dev/null
  fi
}

count_alerts() { awk -F, 'NR>1 && $2==1 {c++} END {print c+0}'; }
first_alert() {
  awk -F, 'NR>1 && $2==1 {printf "%.2f", $8; exit}' | grep . || echo "-"
}

REPORT="$OUT_DIR/report.md"
{
  echo "# Замороженный отчёт (offline, production-конфигурация)"
  echo
  echo "Сгенерировано: \`scripts/eval/frozen_report.sh\`"
  echo
  echo "| Набор | Кадров | Тревог | Первая дистанция, м |"
  echo "|---|---:|---:|---:|"
} > "$REPORT"

run_set() {
  local name="$1" path="$2"; shift 2
  local csv="$OUT_DIR/${name}.csv"
  run "$path" "$@" > "$csv"
  local alerts total first
  alerts="$(count_alerts < "$csv")"
  total="$(awk 'END{print NR-1}' "$csv")"
  first="$(first_alert < "$csv")"
  printf '| `%s` | %s | %s | %s |\n' "$name" "$total" "$alerts" "$first" >> "$REPORT"
  printf '  %-28s alerts=%-4s first=%s\n' "$name" "$alerts" "$first"
}

echo "[frozen-report] obstacle / platform (регрессия)"
run_set doubleT_obstacle      data/frames/doubleT_obstacle.frames --frames 200 --fusion
run_set doubleT_platform      data/frames/doubleT_platform.frames --frames 300 --fusion

echo "[frozen-report] пустые фоны"
run_set roundT_doubleT        data/frames/roundT_doubleT.frames --frames 100 --fusion
run_set new_data_head         data/frames/new_data_head.frames --frames 100 --fusion
run_set new_data_23           data/frames/new_data_23.frames --frames 100 --fusion
run_set new_data_2931         data/frames/new_data_2931.frames --frames 100 --fusion

if [ "${ARGUS_FROZEN_INCLUDE_CLOUD:-0}" = "1" ]; then
  echo "[frozen-report] независимый набор с объектами (долго)"
  run_set cloud_with_fake_obj data/frames/cloud_with_fake_obj.frames --frames 1510 --fusion
else
  echo "[frozen-report] cloud_with_fake_obj пропущен (включить: ARGUS_FROZEN_INCLUDE_CLOUD=1)"
fi

echo "[frozen-report] отчёт: $REPORT"
