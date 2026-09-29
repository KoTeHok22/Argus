#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import os
from collections import defaultdict

import rclpy
from rclpy.node import Node

from argus_msgs.msg import Diagnostics, ObstacleArray

STATUS = {
    ObstacleArray.STATUS_CLEAR: "CLEAR",
    ObstacleArray.STATUS_WARNING: "WARNING",
    ObstacleArray.STATUS_BLOCKED: "BLOCKED",
    ObstacleArray.STATUS_DEGRADED: "DEGRADED",
}

CSV_FIELDS = (
    "source_frame",
    "stamp_ns",
    "status",
    "nearest_m",
    "objects",
    "obstacles_json",
    "candidates_json",
    "fps",
    "t_filter_ms",
    "t_range_image_ms",
    "t_ground_ms",
    "t_odometry_ms",
    "t_detect_ms",
    "t_model_ms",
    "t_fusion_ms",
    "t_alert_path_ms",
)


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    w = k - lo
    return s[lo] * (1.0 - w) + s[hi] * w


class Watch(Node):
    def __init__(self, csv_path: str) -> None:
        super().__init__("argus_detect_watch")
        self.csv_path = csv_path
        self.rows: dict[int, dict] = defaultdict(dict)
        self.written: set[int] = set()
        self.alerts = 0
        self.frames = 0
        self.last_status = None
        os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
        self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.csv_file.flush()
        qos = 50
        self.create_subscription(ObstacleArray, "/argus/obstacles", self.on_obstacles, qos)
        self.create_subscription(Diagnostics, "/argus/diagnostics", self.on_pre, qos)
        self.create_subscription(Diagnostics, "/argus/diagnostics_odometry", self.on_odom, qos)
        self.create_subscription(Diagnostics, "/argus/diagnostics_detector", self.on_det, qos)
        self.create_subscription(Diagnostics, "/argus/diagnostics_model", self.on_model, qos)

    def on_pre(self, msg: Diagnostics) -> None:
        row = self.rows[stamp_ns(msg.header)]
        row["t_filter_ms"] = msg.t_filter_ms
        row["t_range_image_ms"] = msg.t_range_image_ms
        row["t_ground_ms"] = msg.t_ground_ms

    def on_odom(self, msg: Diagnostics) -> None:
        self.rows[stamp_ns(msg.header)]["t_odometry_ms"] = msg.t_odometry_ms

    def on_det(self, msg: Diagnostics) -> None:
        self.rows[stamp_ns(msg.header)]["t_detect_ms"] = msg.t_detect_ms

    def on_model(self, msg: Diagnostics) -> None:
        self.rows[stamp_ns(msg.header)]["t_model_ms"] = msg.t_model_ms

    def on_obstacles(self, msg: ObstacleArray) -> None:
        key = stamp_ns(msg.header)
        self.frames += 1
        name = STATUS.get(msg.status, str(msg.status))
        row = self.rows[key]
        row["source_frame"] = self.frames - 1
        row["stamp_ns"] = key
        row["status"] = name
        row["nearest_m"] = msg.nearest_range_m
        row["objects"] = msg.obstacles_detected
        row["obstacles_json"] = json.dumps(
            [
                {
                    "track_id": obstacle.track_id,
                    "position": [obstacle.position.x, obstacle.position.y, obstacle.position.z],
                    "extent": [obstacle.extent.x, obstacle.extent.y, obstacle.extent.z],
                    "range_m": obstacle.range_m,
                    "point_count": obstacle.point_count,
                    "reason": obstacle.reason,
                }
                for obstacle in msg.obstacles
            ],
            separators=(",", ":"),
        )
        row["candidates_json"] = json.dumps(
            [
                {
                    "candidate_id": candidate.candidate_id,
                    "position": [candidate.position.x, candidate.position.y, candidate.position.z],
                    "extent": [candidate.extent.x, candidate.extent.y, candidate.extent.z],
                    "range_m": candidate.range_m,
                    "confidence": candidate.confidence,
                    "votes_free_space": candidate.votes_free_space,
                    "votes_no_return": candidate.votes_no_return,
                    "votes_geometry": candidate.votes_geometry,
                    "votes_temporal": candidate.votes_temporal,
                    "point_count": candidate.point_count,
                    "reason": candidate.reason,
                }
                for candidate in msg.candidates
            ],
            separators=(",", ":"),
        )
        row["fps"] = msg.fps
        row["t_fusion_ms"] = msg.processing_ms
        pre = (
            float(row.get("t_filter_ms") or 0)
            + float(row.get("t_range_image_ms") or 0)
            + float(row.get("t_ground_ms") or 0)
        )
        det = float(row.get("t_detect_ms") or 0)
        fus = float(row.get("t_fusion_ms") or 0)
        row["t_alert_path_ms"] = pre + det + fus
        if key not in self.written:
            out = {f: row.get(f, "") for f in CSV_FIELDS}
            self.writer.writerow(out)
            self.csv_file.flush()
            self.written.add(key)
        line = (
            f"{name} objects={msg.obstacles_detected} "
            f"nearest={msg.nearest_range_m:.1f}m fps={msg.fps:.1f} "
            f"fusion={msg.processing_ms:.1f}ms path={row['t_alert_path_ms']:.1f}ms"
        )
        if msg.status == ObstacleArray.STATUS_BLOCKED:
            self.alerts += 1
            print(f"ALERT {line}", flush=True)
        elif name != self.last_status:
            print(line, flush=True)
        self.last_status = name

    def close(self) -> None:
        self.csv_file.close()
        blocked = [r for r in self.rows.values() if r.get("status") == "BLOCKED"]
        paths = [float(r["t_alert_path_ms"]) for r in self.rows.values() if "t_alert_path_ms" in r]
        odoms = [float(r["t_odometry_ms"]) for r in self.rows.values() if "t_odometry_ms" in r]
        dets = [float(r["t_detect_ms"]) for r in self.rows.values() if "t_detect_ms" in r]
        models = [float(r["t_model_ms"]) for r in self.rows.values() if "t_model_ms" in r]
        fusions = [float(r["t_fusion_ms"]) for r in self.rows.values() if "t_fusion_ms" in r]
        print(f"frames={self.frames} blocked_alerts={self.alerts} csv={self.csv_path}", flush=True)
        print(
            f"alert_path_ms n={len(paths)} med={percentile(paths, 50):.1f} "
            f"p95={percentile(paths, 95):.1f} max={max(paths) if paths else 0:.1f}",
            flush=True,
        )
        print(
            f"odometry_ms n={len(odoms)} med={percentile(odoms, 50):.1f} "
            f"p95={percentile(odoms, 95):.1f} max={max(odoms) if odoms else 0:.1f}",
            flush=True,
        )
        print(
            f"detect_ms n={len(dets)} med={percentile(dets, 50):.1f} "
            f"p95={percentile(dets, 95):.1f} max={max(dets) if dets else 0:.1f}",
            flush=True,
        )
        print(
            f"model_ms n={len(models)} med={percentile(models, 50):.1f} "
            f"p95={percentile(models, 95):.1f} max={max(models) if models else 0:.1f}",
            flush=True,
        )
        print(
            f"fusion_ms n={len(fusions)} med={percentile(fusions, 50):.1f} "
            f"p95={percentile(fusions, 95):.1f} max={max(fusions) if fusions else 0:.1f}",
            flush=True,
        )
        if blocked:
            print(f"blocked_frames={len(blocked)} first_nearest={blocked[0].get('nearest_m')}", flush=True)


def main() -> int:
    csv_path = os.environ.get("ARGUS_LATENCY_CSV", "/tmp/argus_latency.csv")
    rclpy.init()
    node = Watch(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    node.close()
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
