#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ALERT_STATUSES = {"BLOCKED", "WARNING"}
LIDAR_HZ = 10.0


@dataclass
class Event:
    start_s: float
    end_s: float
    frames: int
    nearest_m: float = -1.0

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_s - self.start_s)

    def to_json(self) -> dict:
        return {
            "start_s": round(self.start_s, 3),
            "end_s": round(self.end_s, 3),
            "frames": self.frames,
            "nearest_m": round(self.nearest_m, 2),
        }


@dataclass
class RunMetrics:
    csv_path: str
    total_frames: int
    status_counts: dict = field(default_factory=dict)
    alerts: int = 0
    alert_frames: int = 0
    duration_s: float = 0.0
    distance_m: float = 0.0
    events: list = field(default_factory=list)

    @property
    def fp_per_km(self) -> float:
        if self.distance_m <= 0:
            return float("nan")
        return self.alert_frames / (self.distance_m / 1000.0)

    @property
    def events_per_km(self) -> float:
        if self.distance_m <= 0:
            return float("nan")
        return len(self.events) / (self.distance_m / 1000.0)

    @property
    def alert_frame_ratio(self) -> float:
        return self.alert_frames / max(1, self.total_frames)

    def to_json(self) -> dict:
        return {
            "csv": self.csv_path,
            "total_frames": self.total_frames,
            "status_counts": self.status_counts,
            "alert_frames": self.alert_frames,
            "events": self.alert_frames and len(self.events) or 0,
            "duration_s": round(self.duration_s, 2),
            "distance_m": round(self.distance_m, 1),
            "alert_frame_ratio": round(self.alert_frame_ratio, 5),
            "fp_per_km": None if self.distance_m <= 0 else round(self.fp_per_km, 3),
            "events_per_km": None if self.distance_m <= 0 else round(self.events_per_km, 3),
            "alert_events": [e.to_json() for e in self.events],
        }


def read_detect_csv(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open(encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)
    return rows


def events_from_rows(rows: list[dict], hz: float) -> tuple[list[Event], int, Counter]:
    counts: Counter = Counter()
    events: list[Event] = []
    current: Event | None = None
    alert_frames = 0
    for index, row in enumerate(rows):
        status = (row.get("status") or "").strip().upper()
        counts[status] += 1
        t = index / hz
        alert = status in ALERT_STATUSES
        nearest = parse_float(row.get("nearest_m"))
        if alert:
            alert_frames += 1
            if current is None:
                current = Event(start_s=t, end_s=t, frames=1, nearest_m=nearest)
            else:
                current.end_s = t
                current.frames += 1
                if nearest >= 0 and (current.nearest_m < 0 or nearest < current.nearest_m):
                    current.nearest_m = nearest
        elif current is not None:
            events.append(current)
            current = None
    if current is not None:
        events.append(current)
    return events, alert_frames, counts


def parse_float(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return -1.0


def distance_from_odom(path: Path) -> tuple[float, float]:
    total_m = 0.0
    duration_s = 0.0
    last_stamp = None
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(",")
            if len(parts) < 6:
                continue
            try:
                frame = int(parts[0])
                stamp = parse_float(parts[1])
                speed = parse_float(parts[5])
            except ValueError:
                continue
            if last_stamp is not None and stamp > 0:
                dt = stamp - last_stamp
                if 0 < dt < 1.0:
                    total_m += max(0.0, speed) * dt
                    duration_s += dt
            last_stamp = stamp
    return total_m, duration_s


def distance_from_speed(duration_s: float, speed_mps: float) -> float:
    return max(0.0, duration_s) * max(0.0, speed_mps)


def analyze(detect_csv: Path, odom_csv: Path | None, avg_speed_mps: float | None,
            hz: float, label: str) -> RunMetrics:
    rows = read_detect_csv(detect_csv)
    events, alert_frames, counts = events_from_rows(rows, hz)
    duration_s = len(rows) / hz
    if odom_csv is not None:
        distance_m, odom_duration_s = distance_from_odom(odom_csv)
        if odom_duration_s > 0:
            duration_s = odom_duration_s
    elif avg_speed_mps is not None:
        distance_m = distance_from_speed(duration_s, avg_speed_mps)
    else:
        distance_m = 0.0
    result = RunMetrics(
        csv_path=str(detect_csv),
        total_frames=len(rows),
        status_counts=dict(counts),
        alert_frames=alert_frames,
        duration_s=duration_s,
        distance_m=distance_m,
        events=events,
    )
    if label:
        result.csv_path = f"{label}: {detect_csv}"
    return result


def print_human(metrics: RunMetrics) -> None:
    print(f"# {metrics.csv_path}")
    print(f"кадров      : {metrics.total_frames}")
    print(f"статусы     : {metrics.status_counts}")
    print(f"кадров с тревогой: {metrics.alert_frames} ({metrics.alert_frame_ratio * 100:.2f}%)")
    print(f"событий     : {len(metrics.events)}")
    print(f"длительность: {metrics.duration_s:.1f} с")
    if metrics.distance_m > 0:
        print(f"дистанция   : {metrics.distance_m:.1f} м ({metrics.distance_m / 1000.0:.3f} км)")
        print(f"FP/км       : {metrics.fp_per_km:.2f} кадров/км")
        print(f"событий/км  : {metrics.events_per_km:.3f}")
    else:
        print("дистанция   : неизвестна (нужен --odom или --speed)")
    for e in metrics.events:
        print(f"  событие {e.start_s:7.2f}–{e.end_s:7.2f} с, {e.frames:4d} кадров, ближайшая {e.nearest_m:6.2f} м")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="События/км и FP/км по detect-CSV")
    p.add_argument("csv", type=Path, help="CSV прогона (stamp/status/nearest)")
    p.add_argument("--odom", type=Path, help="CSV одометрии (frame,valid,x,y,z,speed,...)")
    p.add_argument("--speed", type=float, help="средняя скорость, м/с (если нет одометрии)")
    p.add_argument("--hz", type=float, default=LIDAR_HZ, help="частота кадров (по умолчанию 10)")
    p.add_argument("--label", default="", help="метка для отчёта")
    p.add_argument("--json", action="store_true", help="вывести JSON вместо текста")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    metrics = analyze(args.csv, args.odom, args.speed, args.hz, args.label)
    if args.json:
        print(json.dumps(metrics.to_json(), ensure_ascii=False, indent=2))
    else:
        print_human(metrics)
    return 0


if __name__ == "__main__":
    sys.exit(main())
