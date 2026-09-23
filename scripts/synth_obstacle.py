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
RINGS = 128


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


def insert_box(pts, distance_m, preset, lateral_m=0.0, vertical_m=0.0,
               n_raw=None, no_return=None, pod=0.0, seed=0):
    length, width, height = PRESETS[preset]
    y0 = -(distance_m + length)
    y1 = -distance_m
    z0 = -1.2 + vertical_m
    box_min = np.array([-width / 2.0 + lateral_m, y0, z0], dtype=np.float64)
    box_max = np.array([width / 2.0 + lateral_m, y1, z0 + height], dtype=np.float64)
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
    nr_left = no_return
    added = 0
    if pod > 0.0 and n_raw and no_return is not None and len(no_return):
        raw = pts["raw"].astype(np.int64)
        occupied = np.zeros(n_raw, dtype=bool)
        occupied[raw] = True
        empty = no_return.astype(np.int64)
        empty = empty[(empty >= 0) & (empty < n_raw) & ~occupied[empty]]
        if len(empty):
            az = empty // RINGS
            ring = empty % RINGS
            az_ref = raw // RINGS
            ring_ref = raw % RINGS
            dirs = np.zeros((len(empty), 3), dtype=np.float64)
            for r in np.unique(ring):
                src = np.where(ring_ref == r)[0]
                dst = np.where(ring == r)[0]
                if len(src) < 8:
                    continue
                order = np.argsort(az_ref[src])
                src_az = az_ref[src][order].astype(np.float64)
                src_dir = directions[src][order]
                pos = np.searchsorted(src_az, az[dst].astype(np.float64))
                pos = np.clip(pos, 1, len(src_az) - 1)
                left = pos - 1
                span = np.maximum(src_az[pos] - src_az[left], 1.0)
                w = (az[dst].astype(np.float64) - src_az[left]) / span
                dirs[dst] = src_dir[left] * (1.0 - w)[:, None] + src_dir[pos] * w[:, None]
            norms = np.linalg.norm(dirs, axis=1)
            usable = norms > 1e-6
            dirs[usable] /= norms[usable, None]
            hit_e, tmin_e = ray_box_intersection(np.zeros(3), dirs, box_min, box_max)
            take = usable & hit_e & (tmin_e > 0.5) & (tmin_e < 400.0)
            idx = np.nonzero(take)[0]
            if len(idx):
                rng = np.random.default_rng(seed)
                keep = rng.random(len(idx)) < pod
                idx = idx[keep]
            if len(idx):
                entry_e = dirs[idx] * tmin_e[idx, None]
                extra = np.empty(len(idx), dtype=pts.dtype)
                extra["x"] = entry_e[:, 0]
                extra["y"] = entry_e[:, 1]
                extra["z"] = entry_e[:, 2]
                extra["i"] = 1.0
                extra["raw"] = empty[idx].astype(np.uint32)
                out = np.concatenate([out, extra])
                drop = set(empty[idx].tolist())
                nr_left = np.array([v for v in no_return if int(v) not in drop], dtype=np.uint32)
                added = len(idx)
    return out, nr_left, {
        "preset": preset,
        "distance_m": distance_m,
        "lateral_m": lateral_m,
        "vertical_m": vertical_m,
        "pod": pod,
        "box_min": box_min.tolist(),
        "box_max": box_max.tolist(),
        "rays_replaced": int(replace.sum()),
        "rays_added": added,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--out", required=True)
    ap.add_argument("--distance", type=float, required=True)
    ap.add_argument("--preset", choices=sorted(PRESETS), default="cart")
    ap.add_argument("--lateral", type=float, default=0.0)
    ap.add_argument("--vertical", type=float, default=0.0)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--start-frame", type=int, default=0)
    ap.add_argument("--pod", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--truth", required=True)
    args = ap.parse_args()
    frames = read_frames(args.src)[: args.frames]
    changed = []
    truth = None
    replaced = 0
    added = 0
    for k, (stamp, n_raw, pts, nr) in enumerate(frames):
        if k < args.start_frame:
            pts2, nr2, info = pts, nr, {
                "preset": args.preset,
                "distance_m": args.distance,
                "lateral_m": args.lateral,
                "vertical_m": args.vertical,
                "pod": args.pod,
                "box_min": [],
                "box_max": [],
                "rays_replaced": 0,
                "rays_added": 0,
            }
        else:
            pts2, nr2, info = insert_box(
                pts, args.distance, args.preset, args.lateral, args.vertical,
                n_raw, nr, args.pod, args.seed + k,
            )
        changed.append((stamp, n_raw, pts2, nr2))
        replaced += info["rays_replaced"]
        added += info["rays_added"]
        truth = info
    truth["rays_replaced"] = replaced
    truth["rays_added"] = added
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_frames(args.out, changed)
    with open(args.truth, "w", encoding="utf-8") as fh:
        json.dump(truth, fh, indent=2)
        fh.write("\n")
    print(
        f"frames={len(changed)} replaced={truth['rays_replaced']} "
        f"added={truth['rays_added']} -> {args.out}"
    )


if __name__ == "__main__":
    main()
