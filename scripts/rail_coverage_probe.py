import argparse
import csv
import sys

import numpy as np

if __package__:
    from scripts.synth_obstacle import read_frames
else:
    from synth_obstacle import read_frames


def paired_coverage(points, bin_m=5.0, min_points=3, max_range_m=105.0):
    if bin_m <= 0 or min_points < 1 or max_range_m <= 0:
        raise ValueError("invalid coverage parameters")
    x = points["x"]
    forward = -points["y"]
    z = points["z"]
    low = (z >= -0.9) & (z <= -0.6)
    bins = int(max_range_m // bin_m)
    paired = 0
    left_counts = []
    right_counts = []
    for index in range(bins):
        in_bin = (low & (forward >= index * bin_m) &
                  (forward < (index + 1) * bin_m))
        left = int(np.count_nonzero(in_bin & (x >= 1.0) & (x <= 2.0)))
        right = int(np.count_nonzero(in_bin & (x <= -1.0) & (x >= -2.0)))
        left_counts.append(left)
        right_counts.append(right)
        if paired == index and left >= min_points and right >= min_points:
            paired += 1
    return paired * bin_m, left_counts, right_counts


def observed_centerline(points, bin_m=5.0, min_points=3, max_range_m=105.0,
                        min_width_m=2.4, max_width_m=4.0):
    if (bin_m <= 0 or min_points < 1 or max_range_m <= 0 or
            min_width_m <= 0 or max_width_m < min_width_m):
        raise ValueError("invalid centerline parameters")
    x = points["x"]
    forward = -points["y"]
    z = points["z"]
    low = (z >= -0.9) & (z <= -0.6)
    line = []
    for index in range(int(max_range_m // bin_m)):
        in_bin = (low & (forward >= index * bin_m) &
                  (forward < (index + 1) * bin_m))
        left = x[in_bin & (x >= 1.0) & (x <= 2.0)]
        right = x[in_bin & (x <= -1.0) & (x >= -2.0)]
        if len(left) < min_points or len(right) < min_points:
            break
        width = float(np.median(left) - np.median(right))
        if not min_width_m <= width <= max_width_m:
            break
        line.append(((index + 0.5) * bin_m,
                     float((np.median(left) + np.median(right)) / 2.0),
                     len(left), len(right)))
    return line


def read_alerts(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or not {"frame", "alert", "forward_m"}.issubset(rows[0]):
        raise ValueError("alerts CSV requires frame,alert,forward_m")
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("frames")
    parser.add_argument("--alerts")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--bin-m", type=float, default=5.0)
    parser.add_argument("--min-points", type=int, default=3)
    parser.add_argument("--max-range-m", type=float, default=105.0)
    parser.add_argument("--centerline", action="store_true")
    args = parser.parse_args()
    frames = read_frames(args.frames)
    if args.limit > 0:
        frames = frames[:args.limit]
    alerts = read_alerts(args.alerts) if args.alerts else None
    if alerts is not None and len(alerts) != len(frames):
        parser.error("alerts CSV must contain exactly one row per frame")
    writer = csv.writer(sys.stdout)
    if args.centerline:
        writer.writerow(("frame", "distance_m", "center_x", "left_count",
                         "right_count"))
    else:
        writer.writerow(("frame", "paired_coverage_m", "alert_m",
                         "within_coverage", "left_counts", "right_counts"))
    for index, (_, _, points, _) in enumerate(frames):
        if args.centerline:
            for distance, center, left, right in observed_centerline(
                    points, args.bin_m, args.min_points, args.max_range_m):
                writer.writerow((index, distance, center, left, right))
            continue
        coverage, left, right = paired_coverage(
            points, args.bin_m, args.min_points, args.max_range_m)
        alert_distance = ""
        within = ""
        if alerts is not None:
            row = alerts[index]
            if int(row["frame"]) != index:
                parser.error(f"alerts CSV frame mismatch at {index}")
            if int(row["alert"]):
                alert_distance = float(row["forward_m"])
                within = int(0 < alert_distance <= coverage)
        writer.writerow((index, coverage, alert_distance, within,
                         ";".join(map(str, left)), ";".join(map(str, right))))


if __name__ == "__main__":
    main()
