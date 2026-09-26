#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
#
set -euo pipefail
cd "$(dirname "$0")/../.."

OUT_DIR="${1:-results/kilometric}"
mkdir -p "$OUT_DIR"

DETECTOR=""
if [ -x "build/argus_core/offline_detector" ]; then
  export LD_LIBRARY_PATH="build/argus_core:build/argus_msgs:${LD_LIBRARY_PATH:-}"
  DETECTOR="local"
elif command -v docker >/dev/null 2>&1; then
  DETECTOR="docker"
else
  echo "нет build/argus_core/offline_detector и нет docker" >&2
  exit 1
fi

run() {
  if [ "$DETECTOR" = "docker" ]; then
    docker run --rm --ipc=host -v "$PWD":/ws -w /ws argus:dev \
      bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && \
                ./build/argus_core/offline_detector $*" 2>/dev/null
  else
    build/argus_core/offline_detector "$@" 2>/dev/null
  fi
}

FRAME_DIR="${ARGUS_FRAMES_DIR:-data/frames}"
SETS="${ARGUS_KILOMETRIC_SETS:-}"

if [ -z "$SETS" ]; then
  for f in "$FRAME_DIR"/*.frames; do
    [ -f "$f" ] || continue
    stem="$(basename "$f" .frames)"
    SETS="${SETS}${SETS:+ }${stem}:0"
  done
fi

for item in $SETS; do
  stem="${item%%:*}"
  frames="${item##*:}"
  limit=()
  if [ "$frames" != "0" ]; then limit=(--frames "$frames"); fi
  printf '  %-32s ' "$stem"
  run "$FRAME_DIR/${stem}.frames" "${limit[@]}" --fusion > "$OUT_DIR/${stem}.detect.csv"
  run "$FRAME_DIR/${stem}.frames" "${limit[@]}" --odometry > "$OUT_DIR/${stem}.odom.csv"
  python3 scripts/eval/metrics_report.py "$OUT_DIR/${stem}.detect.csv" \
    --odom "$OUT_DIR/${stem}.odom.csv" --label "$stem" --json > "$OUT_DIR/${stem}.json"
  python3 - "$OUT_DIR/${stem}.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1], encoding="utf-8"))
print(f"events={d['events']} frames={d['total_frames']} alerts={d['alert_frames']} "
      f"dist={d['distance_m']}m fp/km={d['fp_per_km']} ev/km={d['events_per_km']}")
PY
done

python3 - "$OUT_DIR" <<'PY'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
runs = []
for path in sorted(out.glob("*.json")):
    runs.append(json.loads(path.read_text(encoding="utf-8")))
lines = ["# Километровая сводка (offline, production)", "",
         "Сгенерировано: `scripts/eval/kilometric_report.sh`", "",
         "Дистанция — интеграл скорости ICP-одометрии на том же наборе. На наборах без цели",
         "(пустые контроли) тревоги/км — это FP/км; на наборах с целью те же величины",
         "описывают обнаружение, а не ложные срабатывания. Наборы короче 50 м в нормировке",
         "не участвуют — шум малой дистанции (прочерк).", "",
         "| Набор | Кадров | Дистанция, м | Тревог (кадров) | Событий | Тревог/км | Событий/км |",
         "|---|---:|---:|---:|---:|---:|---:|"]
for r in runs:
    name = r["csv"].split(":")[0]
    dist = r["distance_m"] or 0.0
    if dist >= 50.0 and r["fp_per_km"] is not None:
        fp = f"{r['fp_per_km']:.2f}"
        ev = f"{r['events_per_km']:.3f}"
    else:
        fp = "-"
        ev = "-"
    lines.append(f"| `{name}` | {r['total_frames']} | {dist:.1f} | {r['alert_frames']} | "
                 f"{r['events']} | {fp} | {ev} |")
(out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[kilometric] отчёт: {out / 'report.md'}")
PY
