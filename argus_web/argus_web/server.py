#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import mimetypes
import os
import re
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from argus_web.align import gauge_profile, load_gauge, reset_gauge, save_gauge
from argus_web.bags import get_bag, list_bags
from argus_web.cloud import frame_stamps, load_frame, pack_xyz
from argus_web.ingest import ingest_archive, job_status, list_jobs, save_upload
from argus_web.paths import static_root, ui_params_yaml
from argus_web.reports import csv_to_text, get_report, list_reports
from argus_web.runner import active_params_file, current_run, start_run

HOST = os.environ.get("ARGUS_WEB_HOST", "0.0.0.0")
PORT = int(os.environ.get("ARGUS_WEB_PORT", "8080"))


def _json(handler: BaseHTTPRequestHandler, payload, status: int = 200) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def _text(handler: BaseHTTPRequestHandler, payload: str, status: int = 200, mime: str = "text/plain; charset=utf-8") -> None:
    body = payload.encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", mime)
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def _bytes(handler: BaseHTTPRequestHandler, payload: bytes, mime: str, extra: dict | None = None) -> None:
    handler.send_response(200)
    handler.send_header("Content-Type", mime)
    handler.send_header("Content-Length", str(len(payload)))
    handler.send_header("Cache-Control", "no-store")
    if extra:
        for key, value in extra.items():
            handler.send_header(key, value)
    handler.end_headers()
    handler.wfile.write(payload)


def _file(handler: BaseHTTPRequestHandler, path: Path) -> None:
    if not path.is_file():
        handler.send_error(HTTPStatus.NOT_FOUND)
        return
    mime = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
    data = path.read_bytes()
    handler.send_response(200)
    handler.send_header("Content-Type", mime)
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def overview() -> dict:
    bags = list_bags()
    reports = list_reports()
    run = current_run()
    ready = [b for b in bags if b["status"] == "ready"]
    latest = reports[0] if reports else None
    return {
        "system": "live" if run and run.get("status") == "running" else "idle",
        "bags": len(bags),
        "ready": len(ready),
        "reports": len(reports),
        "run": run,
        "latest": latest,
        "now": time.time(),
    }


def _query(path: str) -> dict:
    parsed = urlparse(path)
    return {k: v[-1] for k, v in parse_qs(parsed.query).items()}


def _report_frame(handler: BaseHTTPRequestHandler, route: str, query: dict) -> None:
    report_id = unquote(route.split("/")[-2])
    report = get_report(report_id)
    if report is None or not report.get("bag_id"):
        handler.send_error(HTTPStatus.NOT_FOUND)
        return
    bag = get_bag(report["bag_id"])
    if bag is None:
        handler.send_error(HTTPStatus.NOT_FOUND)
        return
    bag_path = Path(bag["path"])
    frame = int(query.get("frame") or "0")
    stamp = query.get("stamp")
    stamps = frame_stamps(bag_path) if stamp else []
    resolved_stamp = stamp if stamp in stamps else None
    if resolved_stamp:
        frame = stamps.index(resolved_stamp)
    cloud = load_frame(bag_path, frame)
    frame = cloud["frame"]
    from io import BytesIO
    from PIL import Image, ImageDraw
    import numpy as np

    points = cloud["xyz"]
    image = Image.new("RGB", (1200, 700), (16, 21, 26))
    draw = ImageDraw.Draw(image)
    half_width = load_gauge().get("half_width", 1.5)
    forward = -points[:, 1] if len(points) else np.zeros(0)
    lateral = points[:, 0] if len(points) else []
    scale = 5.0
    center_x, base_y = 600, 640
    draw.line((center_x - half_width * scale, base_y, center_x - half_width * scale, 60), fill=(240, 188, 70), width=2)
    draw.line((center_x + half_width * scale, base_y, center_x + half_width * scale, 60), fill=(240, 188, 70), width=2)
    for distance, side in zip(forward, lateral):
        if 0 <= distance <= 110:
            draw.point((center_x + float(side) * scale, base_y - float(distance) * scale), fill=(94, 174, 203))
    row = next(
        (item for item in report.get("rows", []) if resolved_stamp and str(item.get("stamp_ns")) == resolved_stamp),
        report.get("rows", [])[frame] if frame < len(report.get("rows", [])) else None,
    )
    for obstacle in (row or {}).get("obstacles", []):
        position = obstacle.get("position") or []
        extent = obstacle.get("extent") or []
        if len(position) != 3 or len(extent) != 3:
            continue
        object_forward = -float(position[1])
        object_side = float(position[0])
        half_forward = float(extent[1]) / 2.0
        half_side = float(extent[0]) / 2.0
        left = center_x + (object_side - half_side) * scale
        right = center_x + (object_side + half_side) * scale
        top = base_y - (object_forward + half_forward) * scale
        bottom = base_y - (object_forward - half_forward) * scale
        draw.rectangle((left, top, right, bottom), outline=(255, 95, 82), width=3)
        draw.text((left, max(8, top - 18)), f"{obstacle.get('track_id', '')} {extent[0]:.2f} x {extent[1]:.2f} x {extent[2]:.2f} m", fill=(255, 220, 210))
    buffer = BytesIO()
    image.save(buffer, format="PNG")
    _bytes(handler, buffer.getvalue(), "image/png")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        route = parsed.path
        query = _query(self.path)
        if route in ("/", "/index.html"):
            _file(self, static_root() / "index.html")
            return
        if route.startswith("/static/"):
            rel = unquote(route[len("/static/"):])
            if rel not in {"app.css", "app.js", "index.html"}:
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            _file(self, static_root() / rel)
            return
        if route == "/api/overview":
            _json(self, overview())
            return
        if route == "/api/bags":
            _json(self, {"items": list_bags(), "jobs": list_jobs()})
            return
        if route.startswith("/api/bags/"):
            bag_id = unquote(route.split("/", 3)[-1])
            bag = get_bag(bag_id)
            if bag is None:
                _json(self, {"error": "Запись не найдена"}, 404)
                return
            _json(self, bag)
            return
        if route == "/api/reports":
            _json(self, {"items": list_reports()})
            return
        if route.startswith("/api/reports/") and route.endswith(".csv"):
            report_id = unquote(route.split("/")[-1][:-4])
            report = get_report(report_id)
            if report is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            _text(self, csv_to_text(report), mime="text/csv; charset=utf-8")
            return
        if route.startswith("/api/reports/") and route.endswith("/frame.png"):
            _report_frame(self, route, query)
            return
        if route.startswith("/api/reports/") and not route.endswith(".csv"):
            report_id = unquote(route.split("/")[-1])
            report = get_report(report_id)
            if report is None:
                _json(self, {"error": "Отчёт не найден"}, 404)
                return
            _json(self, report)
            return
        if route.startswith("/api/reports/") and not route.endswith("/frame.png"):
            report_id = unquote(route.split("/")[-1])
            report = get_report(report_id)
            if report is None:
                _json(self, {"error": "Отчёт не найден"}, 404)
                return
            _json(self, report)
            return
        if route == "/api/run":
            _json(self, current_run() or {"status": "idle"})
            return
        if route == "/api/gauge":
            _json(self, gauge_profile(load_gauge()))
            return
        if route == "/api/params":
            _text(self, active_params_file().read_text(encoding="utf-8"), mime="text/yaml; charset=utf-8")
            return
        if route == "/api/stamps":
            bag_id = query.get("bag")
            bag = get_bag(bag_id or "")
            if bag is None:
                _json(self, {"error": "Запись не найдена"}, 404)
                return
            _json(self, {"stamps": frame_stamps(Path(bag["path"]))})
            return
        if route == "/api/cloud":
            bag_id = query.get("bag")
            frame = int(query.get("frame") or "0")
            bag = get_bag(bag_id or "")
            if bag is None:
                _json(self, {"error": "Запись не найдена"}, 404)
                return
            data = load_frame(Path(bag["path"]), frame)
            extra = {
                "X-Argus-Frame": str(data["frame"]),
                "X-Argus-Total": str(data["total"]),
                "X-Argus-Points": str(data["points"]),
                "X-Argus-Raw": str(data["raw_points"]),
                "X-Argus-Stamp": str(data.get("stamp_ns") or 0),
            }
            _bytes(self, pack_xyz(data["xyz"]), "application/octet-stream", extra)
            return
        if route.startswith("/api/synthetic/"):
            bag_id = unquote(route.rsplit("/", 1)[-1])
            bag = get_bag(bag_id)
            if bag is None or not bag.get("synthetic"):
                _json(self, {"error": "Синтетическая запись не найдена"}, 404)
                return
            frame = int(query.get("frame") or "0")
            data = load_frame(Path(bag["path"]), frame)
            extra = {
                "X-Argus-Frame": str(data["frame"]),
                "X-Argus-Total": str(data["total"]),
                "X-Argus-Points": str(data["points"]),
                "X-Argus-Raw": str(data["raw_points"]),
                "X-Argus-Stamp": str(data.get("stamp_ns") or 0),
            }
            _bytes(self, pack_xyz(data["xyz"]), "application/octet-stream", extra)
            return
        if route.startswith("/api/jobs/"):
            job = job_status(unquote(route.split("/")[-1]))
            if job is None:
                _json(self, {"error": "Загрузка не найдена"}, 404)
                return
            _json(self, job)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        route = parsed.path
        length = int(self.headers.get("Content-Length") or "0")
        if route == "/api/run":
            body = json.loads(self.rfile.read(length) or b"{}")
            try:
                run = start_run(body.get("bag") or "", float(body.get("rate") or 1.0))
            except ValueError as exc:
                _json(self, {"error": str(exc)}, 400)
                return
            _json(self, run, 202)
            return
        if route == "/api/gauge":
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
                if body.get("reset"):
                    _json(self, reset_gauge())
                    return
                _json(self, save_gauge(body))
            except (TypeError, ValueError) as exc:
                _json(self, {"error": str(exc)}, 400)
            return
        if route == "/api/upload":
            filename = unquote(self.headers.get("X-Filename") or "upload.bin")
            if not filename.lower().endswith((".zst", ".db3")):
                _json(self, {"error": "Загрузите архив .zst или файл .db3"}, 415)
                return
            dest_name = unquote(self.headers.get("X-Name") or Path(filename).stem)
            dest_name = re.sub(r"[^A-Za-z0-9._-]+", "_", dest_name) or "upload"
            notes = unquote(self.headers.get("X-Notes") or "")

            def upload_chunks():
                remaining = length
                while remaining > 0:
                    chunk = self.rfile.read(min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    remaining -= len(chunk)
                    yield chunk

            saved = save_upload(filename, upload_chunks())
            from argus_web.ingest import _set

            _set(dest_name, id=dest_name, name=dest_name, status="queued", progress=0.0)
            thread = threading.Thread(
                target=_ingest_and_run, args=(saved, dest_name, notes), daemon=True
            )
            thread.start()
            _json(self, {"id": dest_name, "status": "parsing"}, 202)
            return
        self.send_error(HTTPStatus.NOT_FOUND)


def _ingest_and_run(saved: Path, dest_name: str, notes: str) -> None:
    job = ingest_archive(saved, dest_name, notes)
    if job.get("status") != "ready":
        return
    try:
        run = start_run(dest_name)
        from argus_web.ingest import _set

        _set(dest_name, status="analyzing", run_id=run.get("id"), report_id=None)
        if run.get("status") == "error":
            _set(dest_name, status="error", error=run.get("error"))
            return
        threading.Thread(
            target=_follow_run,
            args=(dest_name, run.get("id")),
            daemon=True,
        ).start()
    except Exception as exc:
        from argus_web.ingest import _set

        _set(dest_name, status="error", error=str(exc))
    finally:
        try:
            saved.unlink()
        except OSError:
            pass


def _follow_run(job_id: str, run_id: str | None) -> None:
    from argus_web.ingest import _set

    while True:
        run = current_run()
        if not run or run.get("id") != run_id:
            return
        if run.get("status") not in ("starting", "running"):
            if run.get("status") == "ready":
                _set(job_id, status="ready", report_id=run.get("report_id"))
            else:
                _set(job_id, status="error", error=run.get("error") or "Прогон детектора завершился с ошибкой")
            return
        time.sleep(2)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Панель Argus")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args(argv)
    ui_params_yaml().parent.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Argus UI http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
