#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from argus_msgs.msg import ObstacleArray

STATUS = {
    ObstacleArray.STATUS_CLEAR: "CLEAR",
    ObstacleArray.STATUS_WARNING: "WARNING",
    ObstacleArray.STATUS_BLOCKED: "BLOCKED",
    ObstacleArray.STATUS_DEGRADED: "DEGRADED",
}


class Watch(Node):
    def __init__(self) -> None:
        super().__init__("argus_detect_watch")
        self.create_subscription(ObstacleArray, "/argus/obstacles", self.on_msg, 10)
        self.last_status = None
        self.alerts = 0
        self.frames = 0

    def on_msg(self, msg: ObstacleArray) -> None:
        self.frames += 1
        name = STATUS.get(msg.status, str(msg.status))
        nearest = msg.nearest_range_m
        line = (
            f"{name} objects={msg.obstacles_detected} "
            f"nearest={nearest:.1f}m fps={msg.fps:.1f} "
            f"proc={msg.processing_ms:.1f}ms"
        )
        if msg.status == ObstacleArray.STATUS_BLOCKED:
            self.alerts += 1
            print(f"ALERT {line}", flush=True)
        elif name != self.last_status:
            print(line, flush=True)
        self.last_status = name


def main() -> int:
    rclpy.init()
    node = Watch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    print(f"frames={node.frames} blocked_alerts={node.alerts}", flush=True)
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
