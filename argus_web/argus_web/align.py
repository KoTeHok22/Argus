from __future__ import annotations

import math
import re
from pathlib import Path

from argus_web.paths import params_yaml, ui_params_yaml

GAUGE_KEYS = (
    "forward_axis",
    "half_width",
    "height",
    "base_offset",
    "chamfer",
    "nose_offset",
    "sensor_height",
    "max_range",
    "safety_margin",
)

DEFAULT_GAUGE = {
    "forward_axis": "-y",
    "half_width": 1.50,
    "height": 2.10,
    "base_offset": 0.20,
    "chamfer": 0.20,
    "nose_offset": 0.00,
    "sensor_height": 1.20,
    "max_range": 300.0,
    "safety_margin": 0.10,
}

AXES = {
    "-y": (0.0, -1.0, 1.0, 0.0),
    "+y": (0.0, 1.0, -1.0, 0.0),
    "y": (0.0, 1.0, -1.0, 0.0),
    "-x": (-1.0, 0.0, 0.0, -1.0),
    "+x": (1.0, 0.0, 0.0, 1.0),
    "x": (1.0, 0.0, 0.0, 1.0),
}


def _num(text: str) -> float:
    return float(text.strip().strip("'\""))


def active_gauge_path() -> Path:
    override = ui_params_yaml()
    if override.is_file():
        return override
    return params_yaml()


def load_gauge(path: Path | None = None) -> dict:
    source = path if path is not None else active_gauge_path()
    gauge = dict(DEFAULT_GAUGE)
    if not source.is_file():
        return gauge
    text = source.read_text(encoding="utf-8", errors="ignore")
    block = re.search(r"(?m)^[ ]{4}gauge:\s*\n((?:[ ]{6}\S.*\n)*)", text)
    if block is None:
        return gauge
    for line in block.group(1).splitlines():
        match = re.match(r"^[ ]{6}([A-Za-z0-9_]+):\s*(.+?)\s*$", line)
        if not match or match.group(1) not in GAUGE_KEYS:
            continue
        key, raw = match.group(1), match.group(2)
        if key == "forward_axis":
            gauge[key] = raw.strip().strip("'\"")
        else:
            gauge[key] = _num(raw)
    return gauge


def save_gauge(values: dict) -> dict:
    current = load_gauge()
    for key in GAUGE_KEYS:
        if key not in values:
            continue
        if key == "forward_axis":
            name = str(values[key]).strip().lower()
            if name not in AXES:
                raise ValueError("Неизвестная ось вперёд")
            current[key] = name
        else:
            number = float(values[key])
            if not math.isfinite(number):
                raise ValueError("Нужно число")
            current[key] = number
    if float(current["half_width"]) <= 0 or float(current["height"]) <= 0:
        raise ValueError("Ширина и высота должны быть больше нуля")
    if (float(current["chamfer"]) < 0 or
            float(current["chamfer"]) >= float(current["half_width"])):
        raise ValueError("Скос должен быть меньше ширины")
    if float(current["safety_margin"]) < 0:
        raise ValueError("Зазор не может быть отрицательным")
    lines = ["    gauge:"]
    for key in GAUGE_KEYS:
        value = current[key]
        if key == "forward_axis":
            lines.append(f"      {key}: \"{value}\"")
        else:
            lines.append(f"      {key}: {float(value):.4f}")
    source = params_yaml().read_text(encoding="utf-8")
    block = re.compile(r"(?m)^    gauge:\s*\n(?:      \S.*\n)*")
    if not block.search(source):
        raise ValueError("Габарит отсутствует в конфигурации детектора")
    ui_params_yaml().write_text(
        block.sub("\n".join(lines) + "\n", source, count=1), encoding="utf-8")
    return gauge_profile(current)


def reset_gauge() -> dict:
    path = ui_params_yaml()
    if path.is_file():
        path.unlink()
    return gauge_profile(load_gauge())


def _axes(name: str) -> tuple[float, float, float, float]:
    return AXES.get(str(name).strip().lower(), AXES["-y"])


def to_gauge(x: float, y: float, gauge: dict) -> tuple[float, float]:
    fx, fy, lx, ly = _axes(gauge["forward_axis"])
    return x * fx + y * fy, x * lx + y * ly


def from_gauge(gx: float, gy: float, gauge: dict) -> tuple[float, float]:
    fx, fy, lx, ly = _axes(gauge["forward_axis"])
    return gx * fx + gy * lx, gx * fy + gy * ly


def gauge_profile(gauge: dict | None = None, view_range: float = 40.0) -> dict:
    g = dict(DEFAULT_GAUGE)
    if gauge:
        g.update(gauge)
    hw = float(g["half_width"])
    z_shift = -float(g["sensor_height"])
    z0 = z_shift + float(g["base_offset"])
    z1 = z_shift + float(g["height"]) - float(g["chamfer"])
    z2 = z_shift + float(g["height"])
    hw_top = hw - float(g["chamfer"])
    nose = float(g["nose_offset"])
    far = min(float(g["max_range"]), view_range)
    gy = [-hw, hw, hw, hw_top, -hw_top, -hw]
    gz = [z0, z0, z1, z2, z2, z1]
    near = []
    end = []
    for k in range(6):
        x, y = from_gauge(nose, gy[k], g)
        near.append([x, y, gz[k]])
        x, y = from_gauge(far, gy[k], g)
        end.append([x, y, gz[k]])
    left = [near[0], end[0]]
    right = [near[1], end[1]]
    return {
        "forward_axis": g["forward_axis"],
        "half_width": hw,
        "height": float(g["height"]),
        "sensor_height": float(g["sensor_height"]),
        "base_offset": float(g["base_offset"]),
        "chamfer": float(g["chamfer"]),
        "nose_offset": nose,
        "view_range": far,
        "left": left,
        "right": right,
        "near": near,
        "far": end,
    }


def _stamp(row: dict) -> int | None:
    raw = row.get("stamp_ns")
    if raw in (None, ""):
        return None
    text = str(raw).strip()
    if text.endswith(".0"):
        text = text[:-2]
    if not text.isdigit() and not (text.startswith("-") and text[1:].isdigit()):
        return None
    try:
        return int(text)
    except ValueError:
        return None


def _timed(row: dict) -> bool:
    for key in ("t_alert_path_ms", "t_detect_ms", "t_filter_ms"):
        raw = row.get(key)
        if raw in (None, ""):
            continue
        try:
            value = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isnan(value) and value > 0:
            return True
    return False


def align_rows(frame_stamps: list[int], rows: list[dict]) -> dict:
    if not frame_stamps:
        return {"rows": [], "by_stamp": {}, "dropped": 0}
    lo = min(frame_stamps)
    hi = max(frame_stamps)
    kept = []
    dropped = 0
    for row in rows:
        stamp = _stamp(row)
        if stamp is None or stamp < lo or stamp > hi or not _timed(row):
            dropped += 1
            continue
        kept.append(row)
    by_stamp = {}
    for row in kept:
        stamp = _stamp(row)
        if stamp is not None:
            by_stamp[stamp] = row
    return {"rows": kept, "by_stamp": by_stamp, "dropped": dropped}


def verdict_for_stamp(stamp_ns: int | None, aligned: dict | None, has_report: bool) -> dict:
    if not has_report or aligned is None:
        return {
            "code": "not_run",
            "text": "Детектор ещё не прогнан",
            "nearest_m": None,
            "explain": None,
        }
    if stamp_ns is None:
        return {
            "code": "unsure",
            "text": "Детектор не уверен",
            "nearest_m": None,
            "explain": None,
        }
    row = (aligned.get("by_stamp") or {}).get(int(stamp_ns))
    if row is None:
        return {
            "code": "unsure",
            "text": "Детектор не уверен",
            "nearest_m": None,
            "explain": None,
        }
    status = (row.get("status") or "").strip()
    explain = (row.get("explain") or "").strip() or None
    if status == "BLOCKED":
        try:
            nearest = float(row.get("nearest_m"))
        except (TypeError, ValueError):
            nearest = None
        if nearest is None or nearest <= 0 or math.isnan(nearest):
            return {
                "code": "unsure",
                "text": "Детектор не уверен",
                "nearest_m": None,
                "explain": explain,
            }
        meters = f"{nearest:.1f}".replace(".", ",")
        return {
            "code": "blocked",
            "text": f"Препятствие, {meters} м",
            "nearest_m": nearest,
            "explain": explain,
        }
    if status == "CLEAR":
        return {
            "code": "clear",
            "text": "В габарите пусто",
            "nearest_m": None,
            "explain": explain,
        }
    return {
        "code": "unsure",
        "text": "Детектор не уверен",
        "nearest_m": None,
        "explain": explain,
    }
