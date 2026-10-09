#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
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
    if any(not math.isfinite(value) or value <= 0
           for value in (length, width, height, distance)):
        raise ValueError("case dimensions and distance must be positive and finite")
    return (length, width, height), distance


def file_receipt(path):
    path = Path(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def detector_command(detector, frames_path, geom_max_range):
    if not math.isfinite(geom_max_range) or geom_max_range <= 0:
        raise ValueError("geometry maximum range must be positive and finite")
    return [str(detector), str(frames_path), "--fusion", "--geom-max-range",
            str(geom_max_range)]


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


def detect(detector, frames_path, csv_path, geom_max_range=150.0):
    process = subprocess.run(
        detector_command(detector, frames_path, geom_max_range),
        check=True, capture_output=True, text=True,
    )
    with open(csv_path, "w", encoding="utf-8", newline="") as output:
        output.write(process.stdout)
    Path(csv_path).with_suffix(".stderr.txt").write_text(process.stderr, encoding="utf-8")
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        expected = ["frame", "alert", "clusters", "raw", "filtered", "tracks",
                    "forward_m", "range_m", "anom_points", "anom_cells"]
        if reader.fieldnames != expected:
            raise ValueError("unexpected detector CSV header")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError("malformed detector CSV row")
            rows.append(row)
    return rows


def analyze(rows, truth, tolerance_m):
    if not math.isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError("tolerance must be nonnegative and finite")
    expected = [frame["frame"] for frame in truth.get("frames", [])]
    if expected != list(range(len(expected))) or not expected:
        raise ValueError("truth must contain consecutive frames starting at zero")
    expected_indices = set(expected)
    by_frame = {}
    for row in rows:
        index = int(row["frame"])
        if index not in expected_indices or index in by_frame:
            raise ValueError("duplicate or unexpected detector frame")
        if row["alert"] not in ("0", "1"):
            raise ValueError("invalid detector alert")
        if row["alert"] == "1" and not math.isfinite(float(row["forward_m"])):
            raise ValueError("alert distance must be finite")
        by_frame[index] = row
    frames = []
    for frame in truth.get("frames", []):
        index = frame["frame"]
        distance = frame.get("distance_m")
        if distance is not None and not math.isfinite(distance):
            raise ValueError("truth distance must be finite or null")
        rays = int(frame.get("rays_replaced", 0)) + int(frame.get("rays_added", 0))
        if rays < 0:
            raise ValueError("ray count cannot be negative")
        active = distance is not None and math.isfinite(distance) and distance > 0 and \
            bool(frame.get("box_min", rays > 0))
        detected = by_frame.get(index)
        alert = detected is not None and detected["alert"] == "1"
        forward = float(detected["forward_m"]) if alert and detected["forward_m"] else None
        matched = bool(active and rays > 0 and alert and forward is not None and
                       abs(forward - distance) <= tolerance_m)
        frames.append({
            "frame": index,
            "distance_m": distance if active else None,
            "rays": rays,
            "prediction_received": detected is not None,
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
    parser.add_argument("--geom-max-ranges", default="150")
    parser.add_argument("--artifacts-dir")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    ranges = [float(value) for value in args.geom_max_ranges.split(",")]
    for maximum in ranges:
        detector_command(args.detector, args.frames, maximum)
    if len(set(ranges)) != len(ranges):
        raise ValueError("geometry ranges must be unique")
    if args.frames_n <= 0 or not 0 <= args.start_frame < args.frames_n:
        raise ValueError("start-frame must select a requested source frame")
    if not math.isfinite(args.tolerance_m) or args.tolerance_m < 0:
        raise ValueError("tolerance must be nonnegative and finite")
    cases = [parse_case(case.strip()) for case in args.cases.split(",") if case.strip()]
    if not cases:
        raise ValueError("at least one case is required")
    inputs = [Path(path).resolve() for path in
              (args.frames, args.poses, args.detector, args.generator, __file__)]
    if Path(args.out).resolve() in inputs:
        raise ValueError("report path must not overwrite an input")
    artifacts = Path(args.artifacts_dir) if args.artifacts_dir else None
    if artifacts:
        artifacts.mkdir(parents=True, exist_ok=False)
    receipts = {name: file_receipt(path) for name, path in (
        ("source", args.frames), ("poses", args.poses), ("detector", args.detector),
        ("generator", args.generator), ("harness", __file__))}

    results = []
    with tempfile.TemporaryDirectory(prefix="argus_rays_") as directory:
        root = Path(directory)
        for case_index, (dimensions, distance) in enumerate(cases):
            out_frames = root / "scene.frames"
            case_root = artifacts / f"case_{case_index:02d}" if artifacts else root
            case_root.mkdir(parents=True, exist_ok=True)
            out_truth = case_root / "truth.json"
            truth = generate(args.generator, args.python, args.frames, out_frames,
                             out_truth, args.poses, dimensions, distance,
                             args.start_frame, args.frames_n, args.seed)
            if len(truth["frames"]) != args.frames_n:
                raise ValueError("generator returned fewer frames than requested")
            scene_receipt = file_receipt(out_frames)
            truth_receipt = file_receipt(out_truth)
            for range_index, maximum in enumerate(ranges):
                out_csv = case_root / f"detector_{range_index:02d}_range_{maximum:g}.csv"
                rows = detect(args.detector, out_frames, out_csv, maximum)
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
                    "geom_max_range_m": maximum,
                    "predictions_received": sum(f["prediction_received"] for f in frames),
                    "expected_predictions": len(frames),
                    "scene": scene_receipt,
                    "truth": truth_receipt,
                    "predictions": file_receipt(out_csv),
                    "stderr": file_receipt(out_csv.with_suffix(".stderr.txt")),
                    "invocation": detector_command(args.detector, out_frames, maximum),
                    "frames": frames,
                }
                results.append(entry)
                print(f"range={maximum:g} L{dimensions[0]:g} W{dimensions[1]:g} H{dimensions[2]:g} "
                      f"D{distance:g}: active={entry['active_frames']} "
                      f"matched={entry['matched_frames']} mean_rays={entry['mean_rays']:.1f}",
                      flush=True)
    for receipt in receipts.values():
        if file_receipt(receipt["path"])["sha256"] != receipt["sha256"]:
            raise ValueError("an input changed during evaluation")
    payload = {"schema_version": 2, "evaluation_kind": "synthetic_nearest_distance_diagnostic",
               "receipts": receipts, "parameters": vars(args), "cases": results}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[ray-coverage] {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
