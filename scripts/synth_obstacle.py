#!/usr/bin/env python3
import argparse
import json
import os
import struct

import numpy as np

MAGIC = b"ARGFRM1\x00"
PRESETS = {
    "person": (0.45, 0.30, 1.75),
    "case": (0.45, 0.35, 0.70),
    "cart": (1.20, 0.80, 1.10),
}


def ray_box_intersection(origin, directions, box_min, box_max):
    origin = np.asarray(origin, dtype=np.float64)
    directions = np.atleast_2d(np.asarray(directions, dtype=np.float64))
    box_min = np.asarray(box_min, dtype=np.float64)
    box_max = np.asarray(box_max, dtype=np.float64)
    hit = np.zeros(len(directions), dtype=bool)
    distance = np.full(len(directions), np.inf)
    for i, direction in enumerate(directions):
        near = -np.inf
        far = np.inf
        valid = True
        for axis in range(3):
            delta = direction[axis]
            start = origin[axis]
            low = box_min[axis]
            high = box_max[axis]
            if abs(delta) < 1e-12:
                continue
            t1 = (low - start) / delta
            t2 = (high - start) / delta
            near = max(near, min(t1, t2))
            far = min(far, max(t1, t2))
        if valid and far >= max(near, 0.0):
            hit[i] = True
            distance[i] = max(near, 0.0)
    return hit, distance


def read_frames(path):
    with open(path, "rb") as fh:
        magic = fh.read(8)
        if magic != MAGIC:
            raise ValueError("bad ARGFRM1")
        n = struct.unpack("<I", fh.read(4))[0]
        frames = []
        for _ in range(n):
            stamp, n_raw, n_valid, n_nr = struct.unpack("<dIII", fh.read(20))
            raw = fh.read(n_valid * 20)
            nr = fh.read(n_nr * 4)
            pts = np.frombuffer(raw, dtype=np.dtype(
                [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")]
            )).copy()
            frames.append((stamp, n_raw, pts, np.frombuffer(nr, dtype="<u4").copy()))
    return frames


def write_frames(path, frames):
    with open(path, "wb") as fh:
        fh.write(MAGIC)
        fh.write(struct.pack("<I", len(frames)))
        for stamp, n_raw, pts, nr in frames:
            fh.write(struct.pack("<dIII", stamp, n_raw, len(pts), len(nr)))
            fh.write(pts.tobytes())
            fh.write(nr.astype("<u4").tobytes())


def insert_box(pts, distance_m, preset, lateral_m=0.0):
    length, width, height = PRESETS[preset]
    y0 = -(distance_m + length)
    y1 = -distance_m
    box_min = np.array([-width / 2.0 + lateral_m, y0, -1.2], dtype=np.float64)
    box_max = np.array([width / 2.0 + lateral_m, y1, -1.2 + height], dtype=np.float64)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    ranges = np.linalg.norm(xyz, axis=1)
    directions = xyz / np.maximum(ranges[:, None], 1e-9)
    hit, tmin = ray_box_intersection(np.zeros(3), directions, box_min, box_max)
    replace = hit & (tmin < ranges)
    out = pts.copy()
    entry = directions[replace] * tmin[replace, None]
    out["x"][replace] = entry[:, 0]
    out["y"][replace] = entry[:, 1]
    out["z"][replace] = entry[:, 2]
    return out, {
        "preset": preset,
        "distance_m": distance_m,
        "box_min": box_min.tolist(),
        "box_max": box_max.tolist(),
        "rays_replaced": int(replace.sum()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--out", required=True)
    ap.add_argument("--distance", type=float, required=True)
    ap.add_argument("--preset", choices=sorted(PRESETS), default="cart")
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--truth", required=True)
    args = ap.parse_args()
    frames = read_frames(args.src)[: args.frames]
    changed = []
    truth = None
    for stamp, n_raw, pts, nr in frames:
        pts2, info = insert_box(pts, args.distance, args.preset)
        changed.append((stamp, n_raw, pts2, nr))
        truth = info
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_frames(args.out, changed)
    with open(args.truth, "w", encoding="utf-8") as fh:
        json.dump(truth, fh, indent=2)
        fh.write("\n")
    print(f"frames={len(changed)} rays={truth['rays_replaced']} -> {args.out}")


if __name__ == "__main__":
    main()
