import argparse
import csv
import sys

import numpy as np

if __package__:
    from scripts.synth_obstacle import read_frames, read_poses
else:
    from synth_obstacle import read_frames, read_poses


def accumulated_centerline(frames, poses, current, window=20, bin_m=5.0,
                           max_range_m=105.0, min_points=3, min_frames=2,
                           return_support=False):
    if (window < 1 or bin_m <= 0 or max_range_m <= 0 or min_points < 1 or
            min_frames < 1 or current < 0 or current >= len(frames) or
            len(poses) != len(frames)):
        raise ValueError("invalid temporal coverage parameters")
    origin, rotation = poses[current]
    observations = [([], []) for _ in range(int(max_range_m // bin_m))]
    for index in range(max(0, current - window + 1), current + 1):
        points = frames[index][2]
        low = points[(points["z"] >= -0.9) & (points["z"] <= -0.6)]
        if not len(low):
            continue
        xyz = np.stack((low["x"], low["y"], low["z"]), axis=1)
        position, heading = poses[index]
        local = ((xyz @ heading.T + position - origin) @ rotation)
        forward = -local[:, 1]
        x = local[:, 0]
        for segment, (left, right) in enumerate(observations):
            in_bin = ((forward >= segment * bin_m) &
                      (forward < (segment + 1) * bin_m))
            left_points = x[in_bin & (x >= 1.0) & (x <= 2.0)]
            right_points = x[in_bin & (x <= -1.0) & (x >= -2.0)]
            if len(left_points) >= min_points:
                left.append(float(np.median(left_points)))
            if len(right_points) >= min_points:
                right.append(float(np.median(right_points)))
    line = []
    support = []
    continuous = True
    for segment, (left, right) in enumerate(observations):
        paired = len(left) >= min_frames and len(right) >= min_frames
        width = float(np.median(left) - np.median(right)) if paired else None
        valid = paired and 2.4 <= width <= 4.0
        support.append(((segment + 0.5) * bin_m, len(left), len(right),
                        width, valid))
        if not valid:
            continuous = False
        if continuous:
            line.append(((segment + 0.5) * bin_m,
                         float((np.median(left) + np.median(right)) / 2),
                         len(left), len(right)))
    return (line, support) if return_support else line


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("frames")
    parser.add_argument("--poses", required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--window", type=int, default=20)
    parser.add_argument("--min-frames", type=int, default=2)
    parser.add_argument("--support", action="store_true")
    args = parser.parse_args()
    frames = read_frames(args.frames)
    if args.limit > 0:
        frames = frames[:args.limit]
    poses = read_poses(args.poses, frames)
    writer = csv.writer(sys.stdout)
    if args.support:
        writer.writerow(("frame", "distance_m", "left_frames", "right_frames",
                         "width_m", "valid"))
    else:
        writer.writerow(("frame", "supported_m", "end_center_x",
                         "end_left_frames", "end_right_frames"))
    for index in range(len(frames)):
        if args.support:
            _, support = accumulated_centerline(
                frames, poses, index, window=args.window,
                min_frames=args.min_frames, return_support=True)
            for distance, left, right, width, valid in support:
                writer.writerow((
                    index, distance, left, right,
                    width if width is not None else "", int(valid)))
        else:
            line = accumulated_centerline(
                frames, poses, index, window=args.window,
                min_frames=args.min_frames)
            writer.writerow((index, len(line) * 5.0,
                             line[-1][1] if line else "",
                             line[-1][2] if line else 0,
                             line[-1][3] if line else 0))


if __name__ == "__main__":
    main()
