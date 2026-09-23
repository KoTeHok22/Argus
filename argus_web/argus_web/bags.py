from __future__ import annotations

import os
import re
import struct
import time
from pathlib import Path

from argus_web.paths import bags_yaml, data_root, uploads_root

LIDAR_TOPICS = ("/lidar_points", "/sensing/lidar/hesai128/pointcloud")

SPLIT_LABEL = {
    "train": "рабочая",
    "holdout": "отложенная",
}

STATUS_READY = "ready"
STATUS_INCOMPLETE = "incomplete"
STATUS_HOLDOUT = "holdout"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def parse_metadata(bag_dir: Path) -> dict:
    meta = bag_dir / "metadata.yaml"
    out = {
        "topic": "/lidar_points",
        "frames": 0,
        "duration_ns": 0,
        "files": [],
        "storage": "sqlite3",
    }
    if not meta.is_file():
        return out
    text = _read(meta)
    names = re.findall(r"name:\s*(/\S+)", text)
    for want in LIDAR_TOPICS:
        if want in names:
            out["topic"] = want
            break
    else:
        if names:
            out["topic"] = names[0]
    count = re.search(r"message_count:\s*(\d+)", text)
    if count:
        out["frames"] = int(count.group(1))
    duration = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if duration:
        out["duration_ns"] = int(duration.group(1))
    files = re.findall(r"relative_file_paths:\s*\n((?:\s+-\s+\S+\n)+)", text)
    listed = []
    if files:
        listed = re.findall(r"-\s+(\S+)", files[0])
    else:
        listed = [p.name for p in sorted(bag_dir.glob("*.db3"))]
    out["files"] = listed
    storage = re.search(r"storage_identifier:\s*(\S+)", text)
    if storage:
        out["storage"] = storage.group(1).strip()
    return out


def _load_registry() -> dict:
    path = bags_yaml()
    if not path.is_file():
        return {}
    text = _read(path)
    registry: dict[str, dict] = {}
    current = None
    for raw in text.splitlines():
        line = raw.rstrip()
        m = re.match(r"^  ([A-Za-z0-9_]+):\s*$", line)
        if m:
            current = m.group(1)
            registry[current] = {"name": current}
            continue
        if current is None:
            continue
        kv = re.match(r"^    ([a-z_]+):\s*(.+)$", line)
        if not kv:
            continue
        key, val = kv.group(1), kv.group(2).strip()
        if val.startswith("[") and val.endswith("]"):
            inner = val[1:-1].strip()
            registry[current][key] = [
                x.strip().strip(",") for x in inner.split(",") if x.strip()
            ]
        elif (val.startswith('"') and val.endswith('"')) or (
            val.startswith("'") and val.endswith("'")
        ):
            registry[current][key] = val[1:-1]
        else:
            if re.fullmatch(r"\d+", val):
                registry[current][key] = int(val)
            elif re.fullmatch(r"\d+\.\d+", val):
                registry[current][key] = float(val)
            else:
                registry[current][key] = val
    return registry


def _dir_size(bag_dir: Path, names: list[str]) -> int:
    total = 0
    listed = names or [p.name for p in bag_dir.glob("*.db3")]
    for name in listed:
        path = bag_dir / Path(name).name
        try:
            if path.is_file():
                total += path.stat().st_size
        except OSError:
            continue
    meta = bag_dir / "metadata.yaml"
    try:
        if meta.is_file():
            total += meta.stat().st_size
    except OSError:
        pass
    return total


def _present_files(bag_dir: Path, names: list[str]) -> tuple[int, int]:
    if not names:
        found = list(bag_dir.glob("*.db3"))
        return len(found), len(found)
    have = 0
    for name in names:
        if (bag_dir / name).is_file() or (bag_dir / Path(name).name).is_file():
            have += 1
    return have, len(names)


def _candidate_dirs() -> list[Path]:
    roots = [data_root(), uploads_root()]
    root = os.environ.get("ARGUS_SYNTHETIC_DATA")
    if root:
        roots.append(Path(root).resolve())
    found: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        if not root.is_dir():
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in {".git", "__pycache__"}]
            if "metadata.yaml" in filenames or "cloud.argfrm" in filenames:
                path = Path(dirpath)
                if path not in seen:
                    seen.add(path)
                    found.append(path)
                dirnames.clear()
    return found


def _status(split: str, have: int, total: int) -> str:
    if split == "holdout":
        return STATUS_HOLDOUT
    if total == 0:
        return STATUS_INCOMPLETE
    if have < total:
        return STATUS_INCOMPLETE
    return STATUS_READY


def describe(bag_dir: Path, registry: dict | None = None) -> dict:
    registry = registry if registry is not None else _load_registry()
    name = bag_dir.name
    meta = parse_metadata(bag_dir)
    info = dict(registry.get(name, {}))
    synthetic_file = bag_dir / "cloud.argfrm"
    if synthetic_file.is_file():
        with synthetic_file.open("rb") as fh:
            header = fh.read(12)
        frames = struct.unpack_from("<I", header, 8)[0] if header[:8] == b"ARGFRM1\0" and len(header) == 12 else 0
        duration_s = max(0, frames - 1) / 10.0
        return {
            "id": name,
            "name": name,
            "path": str(bag_dir),
            "topic": "synthetic/ARGFRM1",
            "frames": frames,
            "points_per_frame": None,
            "rate_hz": 10.0,
            "duration_s": duration_s,
            "size_bytes": synthetic_file.stat().st_size,
            "files_have": 1 if frames else 0,
            "files_total": 1,
            "split": "synthetic",
            "split_label": "синтетика",
            "labels": ["synthetic"],
            "notes": (bag_dir / "notes.txt").read_text(encoding="utf-8") if (bag_dir / "notes.txt").is_file() else "",
            "status": STATUS_READY if frames else STATUS_INCOMPLETE,
            "holdout": False,
            "synthetic": True,
        }
    have, total = _present_files(bag_dir, meta["files"])
    split = str(info.get("split") or "train")
    duration_s = meta["duration_ns"] / 1_000_000_000.0 if meta["duration_ns"] else 0.0
    return {
        "id": name,
        "name": name,
        "path": str(bag_dir),
        "topic": info.get("topic") or meta["topic"],
        "frames": int(meta["frames"] or 0),
        "points_per_frame": info.get("points_per_frame"),
        "rate_hz": info.get("rate_hz") or 10.0,
        "duration_s": duration_s,
        "size_bytes": _dir_size(bag_dir, meta["files"]),
        "files_have": have,
        "files_total": total,
        "split": split,
        "split_label": SPLIT_LABEL.get(split, split),
        "labels": info.get("labels") or [],
        "notes": info.get("notes") or "",
        "status": _status(split, have, total),
        "holdout": split == "holdout",
        "synthetic": False,
    }


_BAGS_CACHE: tuple[float, list[dict]] | None = None


def invalidate_bags() -> None:
    global _BAGS_CACHE
    _BAGS_CACHE = None


def list_bags() -> list[dict]:
    global _BAGS_CACHE
    now = time.time()
    if _BAGS_CACHE and now - _BAGS_CACHE[0] < 2.0:
        return _BAGS_CACHE[1]
    registry = _load_registry()
    bags = [describe(path, registry) for path in _candidate_dirs()]
    best: dict[str, dict] = {}
    for item in bags:
        current = best.get(item["id"])
        if current is None or (item["frames"], item["size_bytes"]) > (current["frames"], current["size_bytes"]):
            best[item["id"]] = item
    bags = [item for item in best.values() if item["frames"] or item["size_bytes"]]
    bags.sort(key=lambda item: (item["holdout"], item["synthetic"], item["name"]))
    _BAGS_CACHE = (now, bags)
    return bags


def get_bag(bag_id: str) -> dict | None:
    for item in list_bags():
        if item["id"] == bag_id:
            return item
    return None


def bag_db3_files(bag_dir: Path) -> list[Path]:
    meta = parse_metadata(bag_dir)
    files = []
    for name in meta["files"]:
        path = bag_dir / name
        if not path.is_file():
            path = bag_dir / Path(name).name
        if path.is_file():
            files.append(path)
    if not files:
        files = sorted(bag_dir.glob("*.db3"))
    return files
