#!/usr/bin/env python3
import argparse
import csv
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
                if start < low or start > high:
                    valid = False
                    break
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


def read_poses(path, frames):
    poses = []
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        required = {"frame", "valid", "x", "y", "z", "heading_deg"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("pose CSV requires frame,valid,x,y,z,heading_deg")
        for index, row in enumerate(reader):
            if index == len(frames):
                break
            if int(row["frame"]) != index or int(row["valid"]) != 1:
                raise ValueError(f"missing or invalid pose at frame {index}")
            values = np.array([float(row[key]) for key in ("x", "y", "z")])
            heading = np.deg2rad(float(row["heading_deg"]))
            if not np.isfinite(values).all() or not np.isfinite(heading):
                raise ValueError(f"non-finite pose at frame {index}")
            c, s = np.cos(heading), np.sin(heading)
            rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            poses.append((values, rotation))
    if len(poses) != len(frames):
        raise ValueError(f"pose CSV has {len(poses)} poses for {len(frames)} frames")
    return poses


def insert_box(pts, distance_m, preset, lateral_m=0.0, vertical_m=0.0,
               n_raw=None, no_return=None, pod=0.0, seed=0,
               ray_origin=None, ray_rotation=None):
    length, width, height = PRESETS[preset]
    y0 = -(distance_m + length)
    y1 = -distance_m
    z0 = -1.2 + vertical_m
    box_min = np.array([-width / 2.0 + lateral_m, y0, z0], dtype=np.float64)
    box_max = np.array([width / 2.0 + lateral_m, y1, z0 + height], dtype=np.float64)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    ranges = np.linalg.norm(xyz, axis=1)
    directions = xyz / np.maximum(ranges[:, None], 1e-9)
    ray_origin = np.zeros(3) if ray_origin is None else np.asarray(ray_origin, dtype=np.float64)
    ray_rotation = np.eye(3) if ray_rotation is None else np.asarray(ray_rotation, dtype=np.float64)
    hit, tmin = ray_box_intersection(ray_origin, directions @ ray_rotation.T, box_min, box_max)
    replace = hit & (tmin > 0.5) & (tmin < 400.0) & (tmin < ranges)
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
            hit_e, tmin_e = ray_box_intersection(ray_origin, dirs @ ray_rotation.T, box_min, box_max)
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
    ap.add_argument("--poses")
    args = ap.parse_args()
    frames = read_frames(args.src)[: args.frames]
    if not frames:
        raise ValueError("source has no frames")
    if args.start_frame < 0 or args.start_frame >= len(frames):
        raise ValueError("start-frame must select a source frame")
    poses = read_poses(args.poses, frames) if args.poses else None
    changed = []
    truth_frames = []
    replaced = 0
    added = 0
    if poses:
        anchor_position, anchor_rotation = poses[args.start_frame]
    for k, (stamp, n_raw, pts, nr) in enumerate(frames):
        ray_origin = None
        ray_rotation = None
        if poses:
            position, rotation = poses[k]
            ray_origin = anchor_rotation.T @ (position - anchor_position)
            ray_rotation = anchor_rotation.T @ rotation
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
                n_raw, nr, args.pod, args.seed + k, ray_origin, ray_rotation,
            )
        if poses:
            box_min = np.asarray(info["box_min"])
            box_max = np.asarray(info["box_max"])
            corners = np.array([
                [x, y, z]
                for x in (box_min[0], box_max[0])
                for y in (box_min[1], box_max[1])
                for z in (box_min[2], box_max[2])
            ]) if k >= args.start_frame else np.empty((0, 3))
            sensor_corners = (corners - ray_origin) @ ray_rotation
            if k >= args.start_frame:
                near_corner = np.array([args.lateral, -args.distance, -1.2 + args.vertical])
                current_near = ray_rotation.T @ (near_corner - ray_origin)
                info["forward_m"] = float(-current_near[1])
            else:
                info["forward_m"] = None
            info["initial_distance_m"] = args.distance
            info["distance_m"] = info["forward_m"]
            info["box_min_world"] = info["box_min"]
            info["box_max_world"] = info["box_max"]
            info["box_min"] = sensor_corners.min(axis=0).tolist() if len(corners) else []
            info["box_max"] = sensor_corners.max(axis=0).tolist() if len(corners) else []
            info["box_corners_sensor"] = sensor_corners.tolist()
            info["sensor_position_m"] = position.tolist()
            info["heading_deg"] = float(np.rad2deg(np.arctan2(rotation[1, 0], rotation[0, 0])))
        if poses:
            info["frame"] = k
        changed.append((stamp, n_raw, pts2, nr2))
        replaced += info["rays_replaced"]
        added += info["rays_added"]
        truth_frames.append(info)
    truth = dict(truth_frames[-1])
    truth["rays_replaced"] = replaced
    truth["rays_added"] = added
    if poses:
        truth["mode"] = "world_fixed"
        truth["poses"] = args.poses
        truth["anchor_frame"] = args.start_frame
        truth["frames"] = truth_frames
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
