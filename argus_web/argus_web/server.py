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
from argus_web.paths import params_yaml, static_root, ui_params_yaml
from argus_web.reports import csv_to_text, get_report, list_reports
from argus_web.runner import current_run, start_run

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
            path = (static_root() / rel).resolve()
            if not str(path).startswith(str(static_root().resolve())):
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            _file(self, path)
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
        if route.startswith("/api/reports/"):
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
            _text(self, params_yaml().read_text(encoding="utf-8"), mime="text/yaml; charset=utf-8")
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
            dest_name = unquote(self.headers.get("X-Name") or Path(filename).stem)
            dest_name = re.sub(r"[^A-Za-z0-9._-]+", "_", dest_name) or "upload"
            notes = unquote(self.headers.get("X-Notes") or "")
            remaining = length
            chunks = []
            while remaining > 0:
                chunk = self.rfile.read(min(1024 * 1024, remaining))
                if not chunk:
                    break
                chunks.append(chunk)
                remaining -= len(chunk)
            saved = save_upload(filename, chunks)
            thread = threading.Thread(
                target=ingest_archive, args=(saved, dest_name, notes), daemon=True
            )
            thread.start()
            _json(self, {"id": dest_name, "status": "parsing"}, 202)
            return
        self.send_error(HTTPStatus.NOT_FOUND)


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
