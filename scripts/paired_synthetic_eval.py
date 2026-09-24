import argparse
import csv
import json
import math
import sys


def load_alerts(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or not {"frame", "alert", "forward_m"}.issubset(rows[0]):
        raise ValueError(f"{path}: expected frame,alert,forward_m")
    return rows


def evaluate(empty, target, truth, tolerance_m):
    frames = truth.get("frames")
    if (not isinstance(frames, list) or len(empty) != len(target) or
            len(target) != len(frames)):
        raise ValueError("empty, target and truth lengths must match")
    if not math.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ValueError("tolerance must be positive and finite")
    results = []
    for index, (base, injected, expected) in enumerate(
            zip(empty, target, frames)):
        if (int(base["frame"]) != index or
                int(injected["frame"]) != index or
                expected.get("frame") != index):
            raise ValueError(f"frame mismatch at {index}")
        distance = expected.get("distance_m")
        active = (distance is not None and math.isfinite(float(distance)) and
                  float(distance) > 0)
        alert = int(injected["alert"]) == 1
        baseline_alert = int(base["alert"]) == 1
        detected_distance = (float(injected["forward_m"]) if alert else None)
        baseline_distance = (float(base["forward_m"])
                             if baseline_alert else None)
        matched = (active and alert and math.isfinite(detected_distance) and
                   abs(detected_distance - float(distance)) <= tolerance_m)
        uncontested = matched and not baseline_alert
        results.append({
            "frame": index, "truth_m": distance if active else None,
            "alert_m": detected_distance, "baseline_m": baseline_distance,
            "matched": matched, "uncontested": uncontested,
        })
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--empty", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--tolerance-m", type=float, default=3.0)
    args = parser.parse_args()
    with open(args.truth, encoding="utf-8") as fh:
        truth = json.load(fh)
    rows = evaluate(
        load_alerts(args.empty), load_alerts(args.target), truth,
        args.tolerance_m,
    )
    writer = csv.writer(sys.stdout)
    writer.writerow((
        "frame", "truth_m", "alert_m", "baseline_m", "matched", "uncontested",
    ))
    for row in rows:
        writer.writerow((
            row["frame"], row["truth_m"], row["alert_m"], row["baseline_m"],
            int(row["matched"]), int(row["uncontested"]),
        ))


if __name__ == "__main__":
    main()
