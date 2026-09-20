from __future__ import annotations

import os
from pathlib import Path


def repo_root() -> Path:
    env = os.environ.get("ARGUS_ROOT")
    if env:
        return Path(env).resolve()
    return Path(__file__).resolve().parents[2]


def data_root() -> Path:
    env = os.environ.get("ARGUS_DATA")
    if env:
        return Path(env).resolve()
    return repo_root() / "data" / "dataset"


def results_root() -> Path:
    env = os.environ.get("ARGUS_RESULTS")
    if env:
        return Path(env).resolve()
    return repo_root() / "results"


def ui_root() -> Path:
    env = os.environ.get("ARGUS_UI_ROOT")
    if env:
        return Path(env).resolve()
    return repo_root() / "data" / "ui"


def uploads_root() -> Path:
    path = ui_root() / "uploads"
    path.mkdir(parents=True, exist_ok=True)
    return path


def runs_root() -> Path:
    path = ui_root() / "runs"
    path.mkdir(parents=True, exist_ok=True)
    return path


def static_root() -> Path:
    env = os.environ.get("ARGUS_WEB_STATIC")
    if env:
        return Path(env).resolve()
    candidates = [
        Path(__file__).resolve().parents[1] / "static",
        repo_root() / "argus_web" / "static",
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, Path(get_package_share_directory("argus_web")) / "static")
    except Exception:
        pass
    for path in candidates:
        if path.is_dir():
            return path
    return candidates[-1]


def bags_yaml() -> Path:
    return repo_root() / "argus_launch" / "config" / "bags.yaml"


def params_yaml() -> Path:
    return repo_root() / "argus_launch" / "config" / "argus_params.yaml"


def ui_params_yaml() -> Path:
    path = ui_root()
    path.mkdir(parents=True, exist_ok=True)
    return path / "params.yaml"
