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

# Список наборов задаётся данными: каждая пара "кадры:кадров".
# По умолчанию берутся локальные файлы из data/frames, метки генерируются нейтрально.
FRAME_DIR="${ARGUS_FRAMES_DIR:-data/frames}"
SETS="${ARGUS_FROZEN_SETS:-}"
if [ -z "$SETS" ]; then
  n=0
  for f in "$FRAME_DIR"/*.frames; do
    [ -f "$f" ] || continue
    n=$((n + 1))
    SETS="${SETS}${SETS:+ }$(basename "$f" .frames):200"
  done
fi

idx=0
for item in $SETS; do
  idx=$((idx + 1))
  stem="${item%%:*}"
  frames="${item##*:}"
  run_set "set-${idx}" "$FRAME_DIR/${stem}.frames" --frames "$frames" --fusion
done

echo "[frozen-report] отчёт: $REPORT"
