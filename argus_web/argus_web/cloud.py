from __future__ import annotations

import sqlite3
import struct
from pathlib import Path

import numpy as np

from argus_web.bags import bag_db3_files, parse_metadata

MAX_ABS_COORD = 1.0e4
RANGE_MIN, RANGE_MAX = 0.5, 400.0
TARGET_POINTS = 2500


def header_stamp_ns(blob: bytes) -> int | None:
    if len(blob) < 12 or blob[:4] != b"\x00\x01\x00\x00":
        return None
    sec, nsec = struct.unpack_from("<iI", blob, 4)
    if sec < 0 or nsec >= 1_000_000_000:
        return None
    return int(sec) * 1_000_000_000 + int(nsec)


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


def _lidar_topic_id(con: sqlite3.Connection) -> int | None:
    topics = con.execute("SELECT id, name, type FROM topics").fetchall()
    pc = [
        t
        for t in topics
        if "PointCloud2" in (t[2] or "") or "pointcloud" in (t[1] or "")
    ]
    if not pc:
        return None
    return max(
        pc,
        key=lambda t: con.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?",
            (t[0],),
        ).fetchone()[0],
    )[0]


_STAMP_CACHE: dict[str, list[str]] = {}


def frame_stamps(bag_dir: Path) -> list[str]:
    key = str(bag_dir)
    cached = _STAMP_CACHE.get(key)
    if cached is not None:
        return cached
    synthetic_file = bag_dir / "cloud.argfrm"
    if synthetic_file.is_file():
        stamps = []
        with synthetic_file.open("rb") as fh:
            if fh.read(8) != b"ARGFRM1\0":
                return stamps
            count = struct.unpack("<I", fh.read(4))[0]
            for _ in range(count):
                header = fh.read(20)
                if len(header) != 20:
                    break
                stamp_s, n_raw, n_valid, n_nr = struct.unpack("<dIII", header)
                stamps.append(str(int(stamp_s * 1e9)))
                fh.seek(n_valid * 20 + n_nr * 4, 1)
        _STAMP_CACHE[key] = stamps
        return stamps
    stamps: list[str] = []
    for path, _count in _file_counts(bag_dir):
        con = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
        try:
            topic_id = _lidar_topic_id(con)
            if topic_id is None:
                continue
            rows = con.execute(
                "SELECT substr(data, 5, 8) FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic_id,),
            ).fetchall()
        finally:
            con.close()
        for row in rows:
            blob = b"\x00\x01\x00\x00" + bytes(row[0])
            stamp = header_stamp_ns(blob)
            if stamp is not None:
                stamps.append(str(stamp))
    _STAMP_CACHE[key] = stamps
    return stamps


def _messages(db3: Path, topic_id: int | None, offset: int, limit: int):
    con = sqlite3.connect(f"file:{db3}?mode=ro&immutable=1", uri=True)
    try:
        if topic_id is None:
            topic_id = _lidar_topic_id(con)
            if topic_id is None:
                return []
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


def _preview_cache(bag_dir: Path, frame: int) -> Path | None:
    try:
        from argus_web.paths import ui_root
    except Exception:
        return None
    key = str(bag_dir.resolve()).encode()
    import hashlib
    digest = hashlib.sha1(key).hexdigest()[:16]
    path = ui_root() / "preview" / digest
    path.mkdir(parents=True, exist_ok=True)
    return path / f"{frame}.npz"


def _store_preview(bag_dir: Path, payload: dict) -> None:
    path = _preview_cache(bag_dir, int(payload["frame"]))
    if path is None:
        return
    np.savez(
        path,
        xyz=payload["xyz"],
        total=np.int32(payload["total"]),
        raw_points=np.int32(payload["raw_points"]),
        stamp_ns=np.int64(payload["stamp_ns"]),
    )


def _load_preview(bag_dir: Path, frame: int) -> dict | None:
    path = _preview_cache(bag_dir, frame)
    if path is None or not path.is_file():
        return None
    data = np.load(path)
    xyz = data["xyz"]
    return {
        "frame": frame,
        "total": int(data["total"]),
        "points": int(xyz.shape[0]),
        "raw_points": int(data["raw_points"]),
        "xyz": xyz,
        "stamp_ns": int(data["stamp_ns"]),
    }


def warm_preview(bag_dir: Path, frames: int | None = None) -> int:
    meta = parse_metadata(bag_dir)
    total = frames if frames is not None else int(meta.get("frames") or 0)
    built = 0
    for frame in range(max(0, total)):
        if _preview_cache(bag_dir, frame) is not None and _preview_cache(bag_dir, frame).is_file():
            continue
        load_frame(bag_dir, frame)
        built += 1
    return built


def load_frame(bag_dir: Path, frame: int) -> dict:
    cached = _load_preview(bag_dir, frame)
    if cached is not None:
        return cached
    synthetic_file = bag_dir / "cloud.argfrm"
    if synthetic_file.is_file():
        with synthetic_file.open("rb") as fh:
            magic = fh.read(8)
            total = struct.unpack("<I", fh.read(4))[0]
            frame = max(0, min(frame, total - 1))
            points = np.zeros(0, dtype=np.dtype([
                ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")
            ]))
            stamp_s = 0.0
            n_valid = 0
            for _ in range(frame + 1):
                header = fh.read(20)
                if len(header) != 20:
                    break
                stamp_s, n_raw, n_valid, n_nr = struct.unpack("<dIII", header)
                points = np.frombuffer(fh.read(n_valid * 20), dtype=np.dtype([
                    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")
                ]))
                fh.seek(n_nr * 4, 1)
        if magic != b"ARGFRM1\0" or total == 0:
            return {
                "frame": frame,
                "total": 0,
                "points": 0,
                "raw_points": 0,
                "xyz": np.zeros((0, 3), dtype=np.float32),
                "stamp_ns": 0,
            }
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32)
        xyz = downsample(xyz)
        payload = {
            "frame": frame,
            "total": total,
            "points": int(xyz.shape[0]),
            "raw_points": int(n_valid),
            "xyz": xyz,
            "stamp_ns": int(stamp_s * 1e9),
        }
        _store_preview(bag_dir, payload)
        return payload
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
    header = header_stamp_ns(bytes(blob))
    xyz = decode_xyz(msg)
    sampled = downsample(xyz)
    payload = {
        "frame": frame,
        "total": total or frame + 1,
        "points": int(sampled.shape[0]),
        "raw_points": int(xyz.shape[0]),
        "xyz": sampled,
        "stamp_ns": int(header if header is not None else stamp),
        "width": int(msg.get("width") or 0),
    }
    _store_preview(bag_dir, payload)
    return payload


def pack_xyz(points: np.ndarray) -> bytes:
    return np.ascontiguousarray(points, dtype=np.float32).tobytes()
