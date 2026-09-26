#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from sensor_msgs.msg import PointCloud2


def read_stamp_ns(msg) -> int:
    header = getattr(msg, "header", None)
    if header is None:
        return 0
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Плеер bag на rosbag2_py + rclpy")
    parser.add_argument("bag", help="каталог rosbag2")
    parser.add_argument("--topic", default="", help="переопределить топик облака")
    parser.add_argument("--rate", type=float, default=1.0, help="скорость воспроизведения")
    parser.add_argument("--loop", action="store_true", help="повторять запись")
    parser.add_argument(
        "--max-messages", type=int, default=0, help="остановиться после N сообщений"
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("argus_bag_play")

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=args.bag, storage_id="sqlite3"),
            ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"не удалось открыть bag {args.bag}: {exc}")
        rclpy.shutdown()
        return 2

    topics = reader.get_all_topics_and_types()
    cloud_topics = [t.name for t in topics if t.type == "sensor_msgs/msg/PointCloud2"]
    if not cloud_topics:
        node.get_logger().error("в bag нет sensor_msgs/msg/PointCloud2")
        rclpy.shutdown()
        return 2
    topic = args.topic or cloud_topics[0]

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    publisher = node.create_publisher(PointCloud2, topic, qos)

    rate = args.rate if args.rate > 0 else 1.0
    sent = 0
    first_stamp = None
    wall_start = time.monotonic()
    node.get_logger().info(f"плеер: bag={args.bag} topic={topic} rate={rate}")

    while rclpy.ok():
        if not reader.has_next():
            if not args.loop:
                break
            reader.seek(0)
            first_stamp = None
            continue
        topic_name, data, stamp_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        if first_stamp is None:
            first_stamp = stamp_ns
            wall_start = time.monotonic()
        elif rate > 0:
            target = (stamp_ns - first_stamp) / 1e9 / rate
            delay = target - (time.monotonic() - wall_start)
            if delay > 0:
                time.sleep(delay)
        publisher.publish(msg)
        sent += 1
        if args.max_messages and sent >= args.max_messages:
            break

    node.get_logger().info(f"плеер: опубликовано {sent} сообщений")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
