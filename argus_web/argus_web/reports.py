from __future__ import annotations

import csv
import json
import math
from pathlib import Path

from argus_web.paths import results_root, runs_root

STATUS_LABEL = {
    "CLEAR": "свободно",
    "WARNING": "внимание",
    "BLOCKED": "препятствие",
    "DEGRADED": "нет данных",
}


def _num(value) -> float | None:
    if value in (None, "", "nan", "-"):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number):
        return None
    return number


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    k = (len(ordered) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(ordered) - 1)
    w = k - lo
    return ordered[lo] * (1.0 - w) + ordered[hi] * w


def _read_rows(path: Path) -> tuple[list[str], list[dict]]:
    text = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    header_i = next(
        (
            i
            for i, line in enumerate(text)
            if line.startswith("stamp_ns,") or line.startswith("frame,")
        ),
        None,
    )
    if header_i is None:
        return [], []
    reader = csv.DictReader(text[header_i:])
    fields = list(reader.fieldnames or [])
    rows = []
    for row in reader:
        if not any((v or "").strip() for v in row.values()):
            continue
        rows.append(row)
    return fields, rows


def _stamp_of(row: dict) -> int | None:
    raw = row.get("stamp_ns")
    if raw in (None, ""):
        return None
    text = str(raw).strip()
    if text.endswith(".0"):
        text = text[:-2]
    if not text.lstrip("-").isdigit():
        return None
    try:
        return int(text)
    except ValueError:
        return None


def _obstacles(row: dict) -> list[dict]:
    raw = row.get("obstacles_json") or ""
    if not raw:
        return []
    try:
        value = json.loads(raw)
    except (TypeError, ValueError):
        return []
    return value if isinstance(value, list) else []


def _timed_row(row: dict) -> bool:
    for key in ("t_alert_path_ms", "t_detect_ms", "t_filter_ms"):
        value = _num(row.get(key))
        if value is not None and value > 0:
            return True
    return False


def _is_latency(fields: list[str]) -> bool:
    return "status" in fields and (
        "stamp_ns" in fields or "t_alert_path_ms" in fields or "t_detect_ms" in fields
    )


def _is_fusion(fields: list[str]) -> bool:
    return "frame" in fields and "alert" in fields


def summarize_latency(rows: list[dict]) -> dict:
    statuses = {}
    for row in rows:
        name = row.get("status") or "UNKNOWN"
        statuses[name] = statuses.get(name, 0) + 1
    usable = [row for row in rows if (row.get("status") or "") not in ("", "DEGRADED")]
    if not usable:
        usable = rows
    blocked = [r for r in usable if r.get("status") == "BLOCKED"]
    clear = [r for r in usable if r.get("status") == "CLEAR"]
    nearest = [
        v
        for v in (_num(r.get("nearest_m")) for r in blocked)
        if v is not None and v > 0
    ]
    paths = [v for v in (_num(r.get("t_alert_path_ms")) for r in usable) if v is not None]
    fps = [v for v in (_num(r.get("fps")) for r in usable) if v is not None and v > 0]
    first_blocked = next(
        (i for i, r in enumerate(usable) if r.get("status") == "BLOCKED"),
        None,
    )
    frames_out = []
    for i, row in enumerate(usable):
        nearest_m = _num(row.get("nearest_m"))
        frames_out.append(
            {
                "index": i,
                "stamp_ns": (str(stamp) if (stamp := _stamp_of(row)) is not None else None),
                "status": row.get("status"),
                "status_label": STATUS_LABEL.get(row.get("status") or "", row.get("status")),
                "nearest_m": nearest_m if nearest_m and nearest_m > 0 else None,
                "forward_m": _num(row.get("forward_m")),
                "objects": int(_num(row.get("objects")) or 0),
                "obstacles": _obstacles(row),
                "point_count": int(_num(row.get("points")) or 0),
                "fps": _num(row.get("fps")),
                "latency_ms": _num(row.get("t_alert_path_ms")),
                "explain": (row.get("explain") or "").strip() or None,
            }
        )
    return {
        "kind": "live",
        "frames": len(usable),
        "blocked": len(blocked),
        "clear": len(clear),
        "degraded": statuses.get("DEGRADED", 0),
        "statuses": statuses,
        "nearest_m": min(nearest) if nearest else None,
        "first_blocked_frame": first_blocked,
        "fps": fps[-1] if fps else None,
        "latency_ms": {
            "median": percentile(paths, 50),
            "p95": percentile(paths, 95),
            "max": max(paths) if paths else None,
        },
        "rows": frames_out,
    }


def summarize_fusion(rows: list[dict]) -> dict:
    usable = []
    for row in rows:
        frame = (row.get("frame") or "").strip()
        if not frame.isdigit():
            continue
        usable.append(row)
    alerts = [r for r in usable if str(r.get("alert") or "0").strip() in ("1", "true", "True")]
    nearest = [
        v
        for v in (_num(r.get("range_m")) for r in alerts)
        if v is not None and v > 0
    ]
    forward = [
        v
        for v in (_num(r.get("forward_m")) for r in alerts)
        if v is not None and v > 0
    ]
    first = next((int(r["frame"]) for r in usable if str(r.get("alert") or "0").strip() == "1"), None)
    frames_out = []
    for row in usable:
        alert = str(row.get("alert") or "0").strip() == "1"
        frames_out.append(
            {
                "index": int(row["frame"]),
                "status": "BLOCKED" if alert else "CLEAR",
                "status_label": "препятствие" if alert else "свободно",
                "nearest_m": _num(row.get("range_m")) if alert else None,
                "forward_m": _num(row.get("forward_m")) if alert else None,
                "objects": int(_num(row.get("tracks")) or 0),
                "obstacles": [],
                "latency_ms": None,
            }
        )
    return {
        "kind": "offline",
        "frames": len(usable),
        "blocked": len(alerts),
        "clear": len(usable) - len(alerts),
        "degraded": 0,
        "nearest_m": min(nearest) if nearest else (min(forward) if forward else None),
        "first_blocked_frame": first,
        "fps": None,
        "latency_ms": {"median": None, "p95": None, "max": None},
        "rows": frames_out,
    }


def _guess_bag(stem: str) -> str | None:
    mapping = (
        ("obstacle", "doubleT_obstacle"),
        ("platform", "doubleT_platform"),
        ("new_data", "new_data"),
        ("head", "new_data"),
        ("t28", "doubleT_obstacle"),
    )
    low = stem.lower()
    for key, name in mapping:
        if key in low:
            return name
    return None


def _title(path: Path) -> str:
    stem = path.stem
    bag = _guess_bag(stem)
    if bag:
        return bag
    return stem


def load_report(path: Path) -> dict:
    fields, rows = _read_rows(path)
    if _is_latency(fields):
        summary = summarize_latency(rows)
    elif _is_fusion(fields):
        summary = summarize_fusion(rows)
    else:
        summary = {
            "kind": "unknown",
            "frames": len(rows),
            "blocked": 0,
            "clear": 0,
            "degraded": 0,
            "nearest_m": None,
            "first_blocked_frame": None,
            "fps": None,
            "latency_ms": {"median": None, "p95": None, "max": None},
            "rows": [],
            "obstacles": [],
        }
    summary.update(
        {
            "id": path.stem,
            "name": _title(path),
            "path": str(path),
            "file_name": path.name,
            "size_bytes": path.stat().st_size if path.is_file() else 0,
            "mtime": path.stat().st_mtime if path.is_file() else 0,
            "bag_id": _report_bag_id(path.stem, path),
            "screenshots": [f"/api/reports/{path.stem}/frame.png?frame={row['index']}" for row in summary.get("rows", []) if row.get("status") == "BLOCKED"],
        }
    )
    return summary


def _report_bag_id(stem: str, path: Path) -> str | None:
    for index in range(len(stem)):
        if stem.startswith(("_202", "_203"), index) and stem[index + 4:index + 5].isdigit():
            return stem[:index]
    from argus_web.bags import list_bags

    matches = [bag["id"] for bag in list_bags() if stem.startswith(f"{bag['id']}_")]
    if matches:
        return max(matches, key=len)
    if path.parent.name == "runs":
        return stem.rsplit("_", 1)[0] if "_" in stem else None
    return _guess_bag(stem)


KEEP_RESULTS = {
    "t28_detect_latency.csv",
    "fusion_obstacle.csv",
    "fusion_platform.csv",
    "val_obstacle.csv",
    "val_platform.csv",
}


def list_reports() -> list[dict]:
    seen: set[Path] = set()
    reports = []
    roots = ((runs_root(), True), (results_root(), False))
    for root, take_all in roots:
        if not root.is_dir():
            continue
        for path in sorted(root.glob("*.csv")):
            if path.name.startswith("_"):
                continue
            if not take_all and path.name not in KEEP_RESULTS:
                continue
            resolved = path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            try:
                reports.append(load_report(path))
            except OSError:
                continue
    reports.sort(key=lambda item: item.get("mtime") or 0, reverse=True)
    return reports


def get_report(report_id: str) -> dict | None:
    for item in list_reports():
        if item["id"] == report_id:
            return item
    return None


def csv_to_text(report: dict) -> str:
    lines = [
        f"Запись: {report['name']}",
        f"Кадров: {report['frames']}",
        f"Препятствие: {report['blocked']}",
        f"Свободно: {report['clear']}",
    ]
    if report.get("nearest_m") is not None:
        lines.append(f"Ближайшее: {report['nearest_m']:.1f} м")
    lat = report.get("latency_ms") or {}
    if lat.get("median") is not None:
        lines.append(
            f"Задержка: {lat['median']:.1f} мс (95-й {lat.get('p95') or 0:.1f})"
        )
    lines.append("")
    lines.append("кадр;состояние;дальность_м;задержка_мс")
    for row in report.get("rows") or []:
        nearest = row.get("nearest_m")
        latency = row.get("latency_ms")
        lines.append(
            "{index};{status};{nearest};{latency}".format(
                index=row.get("index"),
                status=row.get("status_label"),
                nearest="" if nearest is None else f"{nearest:.2f}",
                latency="" if latency is None else f"{latency:.1f}",
            )
        )
    return "\n".join(lines) + "\n"
