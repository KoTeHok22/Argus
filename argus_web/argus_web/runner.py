from __future__ import annotations

import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

from argus_web.bags import get_bag
from argus_web.paths import params_yaml, results_root, runs_root, ui_params_yaml

_LOCK = threading.Lock()
_RUN: dict | None = None


def current_run() -> dict | None:
    with _LOCK:
        return dict(_RUN) if _RUN else None


def _update(**fields) -> None:
    global _RUN
    with _LOCK:
        if _RUN is None:
            _RUN = {}
        _RUN.update(fields)


def active_params_file() -> Path:
    override = ui_params_yaml()
    if override.is_file():
        return override
    return params_yaml()


def save_params(text: str) -> Path:
    path = ui_params_yaml()
    path.write_text(text, encoding="utf-8")
    return path


def load_params_text() -> str:
    path = active_params_file()
    return path.read_text(encoding="utf-8")


def _detect_cmd(bag: dict, rate: float, csv_path: Path) -> list[str]:
    os.environ["ARGUS_LATENCY_CSV"] = str(csv_path)
    if Path("/entrypoint.sh").is_file():
        return ["bash", "/entrypoint.sh", "detect", bag["path"], str(rate)]
    docker = shutil.which("docker")
    if docker:
        return [
            docker,
            "run",
            "--rm",
            "--shm-size=256m",
            "-v",
            f"{Path(bag['path']).resolve()}:/data/{bag['id']}:ro",
            "-v",
            f"{csv_path.parent.resolve()}:/tmp/argus_runs",
            "-e",
            f"ARGUS_LATENCY_CSV=/tmp/argus_runs/{csv_path.name}",
            "argus",
            "detect",
            f"/data/{bag['id']}",
            str(rate),
        ]
    raise RuntimeError("Нет образа argus и нет ROS в этом окружении")


def start_run(bag_id: str, rate: float = 1.0) -> dict:
    bag = get_bag(bag_id)
    if bag is None:
        raise ValueError("Запись не найдена")
    if bag["holdout"]:
        raise ValueError("Отложенная запись закрыта до фазы проверки")
    if bag["status"] != "ready":
        raise ValueError("Запись ещё не собрана полностью")
    existing = current_run()
    if existing and existing.get("status") in ("starting", "running"):
        raise ValueError("Уже идёт другой прогон")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = runs_root() / f"{bag_id}_{stamp}.csv"
    _update(
        id=csv_path.stem,
        bag_id=bag_id,
        status="starting",
        csv=str(csv_path),
        started=time.time(),
        error=None,
        log="",
    )
    thread = threading.Thread(
        target=_execute, args=(bag, rate, csv_path), daemon=True
    )
    thread.start()
    return current_run() or {}


def _execute(bag: dict, rate: float, csv_path: Path) -> None:
    env = os.environ.copy()
    env["ARGUS_LATENCY_CSV"] = str(csv_path)
    try:
        cmd = _detect_cmd(bag, rate, csv_path)
    except Exception as exc:
        _update(status="error", error=str(exc))
        return
    _update(status="running", command=" ".join(cmd))
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(Path(bag["path"]).resolve().parent),
            env=env,
            capture_output=True,
            text=True,
            timeout=60 * 60,
        )
        log = (proc.stdout or "") + (proc.stderr or "")
        status = "ready" if proc.returncode == 0 else "error"
        _update(status=status, log=log[-8000:], returncode=proc.returncode)
        if csv_path.is_file():
            try:
                shutil.copy2(csv_path, results_root() / csv_path.name)
            except OSError:
                pass
    except Exception as exc:
        _update(status="error", error=str(exc))
