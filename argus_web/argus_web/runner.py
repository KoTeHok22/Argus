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
    params_path = (csv_path.parent / f"{csv_path.stem}_params.yaml").resolve()
    if bag.get("synthetic"):
        raise ValueError("Для синтетической последовательности нужен ARGFRM1 detector runner")
    if Path("/entrypoint.sh").is_file():
        return ["bash", "/entrypoint.sh", "detect", bag["path"], str(rate),
                str(params_path)]
    docker = shutil.which("docker")
    if docker:
        workspace = Path("/ws").resolve()
        mounts = ["-v", f"{Path(bag['path']).resolve()}:/data/{bag['id']}:ro"]
        if csv_path.resolve().is_relative_to(workspace):
            mounts.extend(["-v", f"{workspace}:/ws"])
            report_path = f"/ws/{csv_path.resolve().relative_to(workspace)}"
            params_container = f"/ws/{params_path.resolve().relative_to(workspace)}"
        else:
            mounts.extend(["-v", f"{csv_path.parent.resolve()}:/tmp/argus_runs"])
            report_path = f"/tmp/argus_runs/{csv_path.name}"
            params_container = f"/tmp/argus_runs/{params_path.name}"
        return [
            docker,
            "run",
            "--rm",
            "--shm-size=256m",
            *mounts,
            "-e",
            f"ARGUS_LATENCY_CSV={report_path}",
            "argus",
            "detect",
            f"/data/{bag['id']}",
            str(rate),
            params_container,
        ]
    raise RuntimeError("Нет образа argus и нет ROS в этом окружении")


def start_run(bag_id: str, rate: float = 1.0) -> dict:
    global _RUN
    bag = get_bag(bag_id)
    if bag is None:
        raise ValueError("Запись не найдена")
    if bag["holdout"]:
        raise ValueError("Отложенная запись закрыта до фазы проверки")
    if bag["status"] != "ready":
        raise ValueError("Запись ещё не собрана полностью")
    with _LOCK:
        if _RUN and _RUN.get("status") in ("starting", "running"):
            raise ValueError("Уже идёт другой прогон")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = runs_root() / f"{bag_id}_{stamp}.csv"
        while csv_path.exists():
            stamp = f"{stamp}_{time.time_ns() % 1000000000:09d}"
            csv_path = runs_root() / f"{bag_id}_{stamp}.csv"
        _RUN = {
            "id": csv_path.stem,
            "bag_id": bag_id,
            "status": "starting",
            "csv": str(csv_path),
            "started": time.time(),
            "error": None,
            "log": "",
        }
    thread = threading.Thread(
        target=_execute, args=(bag, rate, csv_path), daemon=True
    )
    try:
        thread.start()
    except Exception as exc:
        _update(status="error", error=str(exc))
    return current_run() or {}


def _execute(bag: dict, rate: float, csv_path: Path) -> None:
    env = os.environ.copy()
    env["ARGUS_LATENCY_CSV"] = str(csv_path)
    try:
        params_snapshot = csv_path.parent / f"{csv_path.stem}_params.yaml"
        shutil.copy2(active_params_file(), params_snapshot)
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
        if csv_path.is_file():
            from argus_web.reports import load_report

            report = load_report(csv_path)
            _update(status=status, log=log[-8000:], returncode=proc.returncode, report_id=report["id"], report=report)
            try:
                shutil.copy2(csv_path, results_root() / csv_path.name)
            except OSError:
                pass
        else:
            _update(status=status, log=log[-8000:], returncode=proc.returncode)
    except Exception as exc:
        _update(status="error", error=str(exc))
