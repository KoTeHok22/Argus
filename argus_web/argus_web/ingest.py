from __future__ import annotations

import shutil
import subprocess
import tarfile
import threading
import uuid
from pathlib import Path
from shutil import rmtree

from argus_web.bags import describe, invalidate_bags
from argus_web.paths import uploads_root

_LOCK = threading.Lock()
_JOBS: dict[str, dict] = {}


def _safe_name(name: str) -> str:
    base = Path(name).name
    keep = "".join(ch if ch.isalnum() or ch in "._-+" else "_" for ch in base)
    return keep or "upload.bin"


def job_status(job_id: str) -> dict | None:
    with _LOCK:
        job = _JOBS.get(job_id)
        return dict(job) if job else None


def list_jobs() -> list[dict]:
    with _LOCK:
        return [dict(job) for job in _JOBS.values()]


def _set(job_id: str, **fields) -> None:
    with _LOCK:
        job = _JOBS.setdefault(job_id, {"id": job_id})
        job.update(fields)


def _extract_zst(src: Path, dest: Path, job_id: str) -> None:
    import zstandard

    with src.open("rb") as compressed:
        reader = zstandard.ZstdDecompressor().stream_reader(compressed)
        with tarfile.open(fileobj=reader, mode="r|") as archive:
            _extract_members(archive, dest, job_id)


def _extract_members(archive, dest: Path, job_id: str) -> None:
    members = 0
    for member in archive:
        if member.isdir():
            continue
        relative = Path(member.name)
        if relative.is_absolute() or ".." in relative.parts:
            continue
        target = (dest / relative).resolve()
        if not target.is_relative_to(dest.resolve()):
            continue
        extracted = archive.extractfile(member)
        if extracted is None:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("wb") as fh:
            shutil.copyfileobj(extracted, fh)
        members += 1
        _set(job_id, progress=min(0.95, 0.1 + members / 250.0))


def ingest_archive(src: Path, dest_name: str, notes: str = "") -> dict:
    dest = uploads_root() / dest_name
    if dest.exists() and any(dest.iterdir()):
        _set(dest_name, name=dest_name, status="error", error=f"Запись «{dest_name}» уже существует", progress=1.0)
        try:
            src.unlink()
        except OSError:
            pass
        return job_status(dest_name) or {}
    if dest.exists():
        dest.rmdir()
    dest.mkdir(parents=True, exist_ok=True)
    job_id = dest_name
    _set(job_id, name=dest_name, status="parsing", progress=0.05, path=str(dest), notes=(notes or "").strip())
    try:
        if src.suffix.lower() == ".zst" or src.name.endswith(".tar.zst"):
            _extract_zst(src, dest, job_id)
        elif src.is_dir():
            for item in src.iterdir():
                target = dest / item.name
                if item.is_dir():
                    shutil.copytree(item, target, dirs_exist_ok=True)
                else:
                    shutil.copy2(item, target)
        else:
            shutil.copy2(src, dest / src.name)
        meta = dest / "metadata.yaml"
        if not meta.is_file():
            nested = next(dest.rglob("metadata.yaml"), None)
            if nested and nested != meta:
                root = nested.parent
                for child in root.iterdir():
                    target = dest / child.name
                    if child.resolve() != target.resolve():
                        if target.exists():
                            if target.is_dir():
                                rmtree(target)
                            else:
                                target.unlink()
                        shutil.move(str(child), str(target))
                if root != dest and root.is_dir() and not any(root.iterdir()):
                    root.rmdir()
        bag = describe(dest)
        invalidate_bags()
        _set(job_id, bag=bag)
        _set(
            job_id,
            status="ready" if bag["status"] == "ready" else "incomplete",
            progress=1.0,
            bag=bag,
            source=str(src),
        )
        if bag["status"] != "ready":
            try:
                src.unlink()
            except OSError:
                pass
        return job_status(job_id) or {}
    except Exception as exc:
        _set(job_id, status="error", error=str(exc), progress=1.0)
        return job_status(job_id) or {}


def save_upload(filename: str, chunks) -> Path:
    dest = uploads_root() / "_incoming"
    dest.mkdir(parents=True, exist_ok=True)
    path = dest / f"{uuid.uuid4().hex}_{_safe_name(filename)}"
    with path.open("wb") as fh:
        for chunk in chunks:
            fh.write(chunk)
    return path
