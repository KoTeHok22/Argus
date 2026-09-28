#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

RING_BINS = [(0, 8), (8, 16), (16, 32), (32, 64), (64, 1 << 30)]


def parse_case(text):
    parts = text.split(":")
    if len(parts) != 4:
        raise ValueError("case must be L:W:H:D")
    length, width, height, distance = (float(part) for part in parts)
    return (length, width, height), distance


def bin_label(low, high):
    return f"{low}+" if high > 1_000_000 else f"{low}-{high}"


def generate(generator, python, source, out_frames, out_truth, poses, dimensions,
             distance, start, frames, seed):
    command = [
        python, generator, str(source),
        "--out", str(out_frames), "--truth", str(out_truth),
        "--distance", str(distance),
        "--length", str(dimensions[0]), "--width", str(dimensions[1]),
        "--height", str(dimensions[2]),
        "--start-frame", str(start), "--frames", str(frames), "--seed", str(seed),
    ]
    if poses:
        command += ["--poses", str(poses)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(Path(out_truth).read_text(encoding="utf-8"))


def detect(detector, frames_path, csv_path):
    process = subprocess.run(
        [detector, str(frames_path), "--fusion"],
        check=True, capture_output=True, text=True,
    )
    lines = [line for line in process.stdout.splitlines()
             if line and line.split(",", 1)[0].isdigit()]
    with open(csv_path, "w", encoding="utf-8", newline="") as output:
        output.write("frame,alert,clusters,raw,filtered,tracks,forward_m,range_m,"
                     "anom_points,anom_cells\n")
        output.write("\n".join(lines) + "\n")
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append(row)
    return rows


def analyze(rows, truth, tolerance_m):
    by_frame = {int(row["frame"]): row for row in rows}
    frames = []
    for frame in truth.get("frames", []):
        index = frame["frame"]
        distance = frame.get("distance_m")
        active = distance is not None and distance > 0 and \
            (frame.get("rays_replaced", 0) + frame.get("rays_added", 0)) > 0
        detected = by_frame.get(index)
        if detected is None:
            continue
        alert = detected["alert"] == "1"
        forward = float(detected["forward_m"]) if alert and detected["forward_m"] else None
        matched = bool(active and alert and forward is not None and
                       abs(forward - distance) <= tolerance_m)
        frames.append({
            "frame": index,
            "distance_m": distance if active else None,
            "rays": int(frame.get("rays_replaced", 0)) + int(frame.get("rays_added", 0)),
            "active": active,
            "alert": alert,
            "forward_m": forward,
            "matched": matched,
        })
    return frames


def summarize(frames):
    summary = []
    for low, high in RING_BINS:
        pool = [f for f in frames if f["active"] and low <= f["rays"] < high]
        matched = sum(f["matched"] for f in pool)
        summary.append({
            "bin": bin_label(low, high),
            "frames": len(pool),
            "matched": matched,
            "missed": len(pool) - matched,
            "recall": matched / len(pool) if pool else None,
        })
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", required=True)
    parser.add_argument("--poses", required=True)
    parser.add_argument("--detector", default="build/argus_core/offline_detector")
    parser.add_argument("--generator", default="scripts/synth_obstacle.py")
    parser.add_argument("--python", default="python3")
    parser.add_argument("--cases", required=True)
    parser.add_argument("--start-frame", type=int, default=10)
    parser.add_argument("--frames-n", type=int, default=40)
    parser.add_argument("--tolerance-m", type=float, default=3.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    results = []
    with tempfile.TemporaryDirectory(prefix="argus_rays_") as directory:
        root = Path(directory)
        for case in args.cases.split(","):
            if not case.strip():
                continue
            dimensions, distance = parse_case(case.strip())
            out_frames = root / "scene.frames"
            out_truth = root / "truth.json"
            out_csv = root / "detector.csv"
            truth = generate(args.generator, args.python, args.frames, out_frames,
                             out_truth, args.poses, dimensions, distance,
                             args.start_frame, args.frames_n, args.seed)
            rows = detect(args.detector, out_frames, out_csv)
            frames = analyze(rows, truth, args.tolerance_m)
            summary = summarize(frames)
            entry = {
                "dimensions_m": list(dimensions),
                "distance_m": distance,
                "active_frames": sum(f["active"] for f in frames),
                "matched_frames": sum(f["matched"] for f in frames),
                "mean_rays": (sum(f["rays"] for f in frames if f["active"]) /
                              max(1, sum(f["active"] for f in frames))),
                "bins": summary,
            }
            results.append(entry)
            print(f"L{dimensions[0]:g} W{dimensions[1]:g} H{dimensions[2]:g} "
                  f"D{distance:g}: active={entry['active_frames']} "
                  f"matched={entry['matched_frames']} mean_rays={entry['mean_rays']:.1f}",
                  flush=True)
    payload = {"cases": results}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[ray-coverage] {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
