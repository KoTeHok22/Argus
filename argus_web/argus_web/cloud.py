from __future__ import annotations

import sqlite3
import struct
from pathlib import Path

import numpy as np

from argus_web.bags import bag_db3_files, parse_metadata

MAX_ABS_COORD = 1.0e4
RANGE_MIN, RANGE_MAX = 0.5, 400.0
TARGET_POINTS = 18000


def parse_pointcloud2_cdr(blob: bytes) -> dict | None:
    if blob[:4] != b"\x00\x01\x00\x00":
        return None
    pos = 4
    sec, nsec = struct.unpack_from("<iI", blob, pos)
    pos += 8
    slen = struct.unpack_from("<I", blob, pos)[0]
    pos += 4
    frame = blob[pos:pos + slen - 1].decode("utf-8", "replace")
    pos += slen
    pos += (4 - ((pos - 4) % 4)) % 4
    _height, width = struct.unpack_from("<II", blob, pos)
    pos += 8
    fcount = struct.unpack_from("<I", blob, pos)[0]
    pos += 4
    fields = {}
    for _ in range(fcount):
        nlen = struct.unpack_from("<I", blob, pos)[0]
        pos += 4
        name = blob[pos:pos + nlen - 1].decode("utf-8", "replace")
        pos += nlen
        pos += (4 - ((pos - 4) % 4)) % 4
        off = struct.unpack_from("<I", blob, pos)[0]
        pos += 4
        dt = blob[pos]
        pos += 1
        pos += (4 - ((pos - 4) % 4)) % 4
        pos += 4
        if name in ("x", "y", "z", "intensity"):
            fields[name] = (off, dt)
    pos += 1
    pos += (4 - ((pos - 4) % 4)) % 4
    point_step, _row_step = struct.unpack_from("<II", blob, pos)
    pos += 8
    dlen = struct.unpack_from("<I", blob, pos)[0]
    pos += 4
    return {
        "sec": sec,
        "nsec": nsec,
        "frame": frame,
        "width": width,
        "point_step": point_step,
        "fields": fields,
        "data": np.frombuffer(blob[pos:pos + dlen], dtype=np.uint8),
    }


def decode_xyz(msg: dict) -> np.ndarray:
    ps = msg["point_step"]
    names = [n for n in ("x", "y", "z") if n in msg["fields"]]
    if len(names) < 3:
        return np.zeros((0, 3), dtype=np.float32)
    offsets = [msg["fields"][n][0] for n in names]
    dt = np.dtype(
        {
            "names": names,
            "formats": ["<f4"] * len(names),
            "offsets": offsets,
            "itemsize": ps,
        }
    )
    pts = msg["data"].view(dt)
    x = pts["x"].astype(np.float64)
    y = pts["y"].astype(np.float64)
    z = pts["z"].astype(np.float64)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    ok &= (np.abs(x) < MAX_ABS_COORD) & (np.abs(y) < MAX_ABS_COORD) & (
        np.abs(z) < MAX_ABS_COORD
    )
    rng = np.sqrt(x * x + y * y + z * z)
    is_zero = (x == 0) & (y == 0) & (z == 0)
    ok &= ~is_zero
    ok &= (rng >= RANGE_MIN) & (rng <= RANGE_MAX)
    return np.stack([x[ok], y[ok], z[ok]], axis=1).astype(np.float32)


def downsample(points: np.ndarray, limit: int = TARGET_POINTS) -> np.ndarray:
    if points.shape[0] <= limit:
        return points
    step = max(1, points.shape[0] // limit)
    return points[::step][:limit]


def _messages(db3: Path, topic_id: int | None, offset: int, limit: int):
    con = sqlite3.connect(f"file:{db3}?mode=ro&immutable=1", uri=True)
    try:
        if topic_id is None:
            topics = con.execute("SELECT id, name, type FROM topics").fetchall()
            pc = [
                t
                for t in topics
                if "PointCloud2" in (t[2] or "") or "pointcloud" in (t[1] or "")
            ]
            if not pc:
                return []
            topic_id = max(
                pc,
                key=lambda t: con.execute(
                    "SELECT COUNT(*) FROM messages WHERE topic_id=?",
                    (t[0],),
                ).fetchone()[0],
            )[0]
        rows = con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT ? OFFSET ?",
            (topic_id, limit, offset),
        ).fetchall()
    finally:
        con.close()
    return rows


_SPLIT_CACHE: dict[str, list[tuple[Path, int]]] = {}


def _file_counts(bag_dir: Path) -> list[tuple[Path, int]]:
    key = str(bag_dir)
    cached = _SPLIT_CACHE.get(key)
    if cached is not None:
        return cached
    files = bag_db3_files(bag_dir)
    counts: list[tuple[Path, int]] = []
    for path in files:
        con = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
        try:
            count = con.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        finally:
            con.close()
        counts.append((path, int(count)))
    _SPLIT_CACHE[key] = counts
    return counts


def _split_index(bag_dir: Path, frame: int) -> tuple[Path, int] | None:
    counts = _file_counts(bag_dir)
    if not counts:
        return None
    remaining = frame
    for path, count in counts:
        if remaining < count:
            return path, remaining
        remaining -= count
    path, count = counts[-1]
    return path, max(0, min(remaining, max(count - 1, 0)))


def load_frame(bag_dir: Path, frame: int) -> dict:
    meta = parse_metadata(bag_dir)
    total = meta["frames"] or 0
    located = _split_index(bag_dir, frame)
    empty = {
        "frame": frame,
        "total": total,
        "points": 0,
        "raw_points": 0,
        "xyz": np.zeros((0, 3), dtype=np.float32),
        "stamp_ns": 0,
    }
    if located is None:
        return empty
    path, local = located
    rows = _messages(path, None, local, 1)
    if not rows:
        return empty
    stamp, blob = rows[0]
    msg = parse_pointcloud2_cdr(bytes(blob))
    if msg is None:
        return empty
    xyz = decode_xyz(msg)
    sampled = downsample(xyz)
    return {
        "frame": frame,
        "total": total or frame + 1,
        "points": int(sampled.shape[0]),
        "raw_points": int(xyz.shape[0]),
        "xyz": sampled,
        "stamp_ns": int(stamp),
        "width": int(msg.get("width") or 0),
    }


def pack_xyz(points: np.ndarray) -> bytes:
    return np.ascontiguousarray(points, dtype=np.float32).tobytes()
