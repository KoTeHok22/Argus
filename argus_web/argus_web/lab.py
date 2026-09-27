from __future__ import annotations

import csv
import json
import os
import subprocess
import threading
import uuid
from pathlib import Path

from argus_web.paths import repo_root
from argus_web.cloud import load_frame

_JOBS: dict[str, dict] = {}
_LOCK = threading.Lock()


def _frames_sources() -> list[dict]:
    roots = [repo_root() / "data" / "frames", repo_root() / "results"]
    items = []
    seen = set()
    for root in roots:
        if not root.is_dir():
            continue
        for path in sorted(root.glob("*.frames")):
            resolved = str(path.resolve())
            if resolved in seen:
                continue
            seen.add(resolved)
            items.append({"id": path.stem, "name": path.stem, "path": resolved})
    return items


def sources() -> list[dict]:
    return [{key: value for key, value in item.items() if key != "path"} for item in _frames_sources()]


def status(job_id: str) -> dict | None:
    with _LOCK:
        job = _JOBS.get(job_id)
        return dict(job) if job else None


def _update(job_id: str, **values) -> None:
    with _LOCK:
        if job_id in _JOBS:
            _JOBS[job_id].update(values)


def _worker(job_id: str, source: str, values: dict) -> None:
    root = repo_root()
    work = root / "data" / "ui" / "lab" / job_id
    work.mkdir(parents=True, exist_ok=True)
    frames = work / "cloud.argfrm"
    truth = work / "truth.json"
    csv_path = work / "detector.csv"
    generator = root / "scripts" / "synth_obstacle.py"
    detector = Path(os.environ.get("ARGUS_DETECTOR", str(root / "install/argus_core/lib/argus_core/offline_detector")))
    command = [
        "python3", str(generator), source, "--out", str(frames), "--truth", str(truth),
        "--distance", str(values["distance"]), "--length", str(values["length"]),
        "--width", str(values["width"]), "--height", str(values["height"]),
        "--lateral", str(values["lateral"]), "--vertical", str(values["vertical"]),
        "--frames", "40", "--start-frame", "8", "--pod", "1",
    ]
    try:
        _update(job_id, status="generating", progress=0.1)
        subprocess.run(command, check=True, capture_output=True, text=True)
        _update(job_id, status="detecting", progress=0.35)
        process = subprocess.run(
            [str(detector), str(frames), "--fusion", "--debug-clusters", "--temporal-residual", "--temporal-vote", "--temporal-window", "5", "--temporal-threshold", "0.5"],
            check=True,
            capture_output=True,
            text=True,
        )
        lines = [line for line in process.stdout.splitlines() if line and line.split(",", 1)[0].isdigit()]
        with csv_path.open("w", encoding="utf-8") as stream:
            stream.write("frame,alert,clusters,raw,filtered,tracks,forward_m,range_m,anom_points,anom_cells\n")
            stream.write("\n".join(lines) + "\n")
        candidates = sum(line.startswith("CANDIDATE,") for line in process.stderr.splitlines())
        confirmed = sum(line.startswith("CONFIRMED,") for line in process.stderr.splitlines())
        events = []
        for line in process.stderr.splitlines():
            fields = line.split(",")
            if fields[0] not in ("CANDIDATE", "CONFIRMED"):
                continue
            try:
                events.append({
                    "kind": fields[0].lower(),
                    "frame": int(fields[1]),
                    "position": [float(value) for value in fields[2:5]],
                    "extent": [float(value) for value in fields[5:8]],
                    "range_m": float(fields[8]),
                    "point_count": int(fields[9]),
                    "confidence": float(fields[10]),
                })
            except (ValueError, IndexError):
                continue
        _update(job_id, status="ready", progress=1.0, frames=40, candidates=candidates, confirmed=confirmed, frames_path=str(frames), csv_path=str(csv_path), truth_path=str(truth), events=events)
    except Exception as exc:
        _update(job_id, status="error", progress=1.0, error=str(exc))


def create(source_id: str, values: dict) -> dict:
    source = next((item for item in _frames_sources() if item["id"] == source_id), None)
    if source is None:
        raise ValueError("Источник кадров не найден")
    for key in ("length", "width", "height", "distance", "lateral", "vertical"):
        values[key] = float(values[key])
        if values[key] <= 0 and key in ("length", "width", "height", "distance"):
            raise ValueError(f"Параметр {key} должен быть больше нуля")
    job_id = f"lab_{uuid.uuid4().hex[:12]}"
    job = {"id": job_id, "status": "queued", "progress": 0.0, "source": source_id, **values}
    with _LOCK:
        _JOBS[job_id] = job
    threading.Thread(target=_worker, args=(job_id, source["path"], dict(values)), daemon=True).start()
    return job


def result(job_id: str) -> dict:
    job = status(job_id)
    if not job or job.get("status") != "ready":
        raise ValueError("Лабораторный прогон ещё не готов")
    rows = []
    with open(job["csv_path"], newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return {
        **job,
        "rows": rows,
        "candidate_rows": [row for row in rows if int(row["clusters"]) > 0 or row["alert"] == "1"],
        "truth": json.loads(Path(job["truth_path"]).read_text(encoding="utf-8")),
    }


def frame(job_id: str, index: int) -> dict:
    job = status(job_id)
    if not job or job.get("status") != "ready":
        raise ValueError("Лабораторный прогон ещё не готов")
    frames_path = Path(job["frames_path"])
    data = load_frame(frames_path.parent, index)
    row = next((item for item in job.get("rows", []) if int(item["frame"]) == data["frame"]), None)
    events = [item for item in job.get("events", []) if item["frame"] == data["frame"]]
    return {"frame": data["frame"], "total": data["total"], "points": data["xyz"].tolist(), "row": row, "events": events}
