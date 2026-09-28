#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

MAGIC = b"ARGFRM1\x00"
MIN_RANGE_M = 0.5
MAX_RANGE_M = 400.0
MAX_ABS_COORD = 1.0e4
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")])


def find_field(msg, name, datatypes):
    for field in msg.fields:
        if field.name == name and field.datatype in datatypes and field.count >= 1:
            return field
    return None


def filter_frame(msg, min_range=MIN_RANGE_M, max_range=MAX_RANGE_M,
                 max_abs_coord=MAX_ABS_COORD):
    xf = find_field(msg, "x", (7, 5, 6))
    yf = find_field(msg, "y", (7, 5, 6))
    zf = find_field(msg, "z", (7, 5, 6))
    if xf is None or yf is None or zf is None:
        raise ValueError("канал без float32 x/y/z")
    if msg.point_step == 0:
        raise ValueError("point_step == 0")
    n_points = int(msg.width) * int(msg.height)
    if n_points * int(msg.point_step) > len(msg.data):
        raise ValueError("data короче width*height*point_step")
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw[: n_points * int(msg.point_step)].reshape(n_points, int(msg.point_step))

    def column(offset):
        return raw[:, offset:offset + 4].copy().view("<f4").reshape(-1)

    x = column(xf.offset)
    y = column(yf.offset)
    z = column(zf.offset)
    inten = find_field(msg, "intensity", (7, 5, 6))
    intensity = column(inten.offset) if inten is not None else np.zeros(n_points, dtype="<f4")

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    zero = (x == 0.0) & (y == 0.0) & (z == 0.0)
    abs_ok = (np.abs(x) <= max_abs_coord) & (np.abs(y) <= max_abs_coord) & \
             (np.abs(z) <= max_abs_coord)
    d2 = x * x + y * y + z * z
    range_ok = (d2 >= min_range * min_range) & (d2 <= max_range * max_range)
    keep = finite & ~zero & abs_ok & range_ok
    no_return = finite & zero

    idx = np.nonzero(keep)[0]
    points = np.empty(idx.size, dtype=POINT_DTYPE)
    points["x"] = x[idx]
    points["y"] = y[idx]
    points["z"] = z[idx]
    points["i"] = intensity[idx]
    points["raw"] = idx.astype("<u4")
    nr = np.nonzero(no_return)[0].astype("<u4")
    return points, nr


def stamp_seconds(header):
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def resolve_topic(reader, requested):
    topics = [t.name for t in reader.get_all_topics_and_types()
              if t.type == "sensor_msgs/msg/PointCloud2"]
    if not topics:
        raise ValueError("в bag нет sensor_msgs/msg/PointCloud2")
    if requested:
        if requested not in topics:
            raise ValueError(f"в bag нет топика {requested}")
        return requested
    for preferred in ("/lidar_points", "/sensing/lidar/hesai128/pointcloud"):
        if preferred in topics:
            return preferred
    return topics[0]


def convert(bag_dir, out_path, topic="", max_frames=0):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import PointCloud2

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic = resolve_topic(reader, topic)
    out_path = Path(out_path)
    written = 0
    with out_path.open("wb") as fh:
        fh.write(MAGIC)
        fh.write(struct.pack("<I", 0))
        while reader.has_next():
            name, data, _ = reader.read_next()
            if name != topic:
                continue
            if max_frames and written >= max_frames:
                break
            msg = deserialize_message(data, PointCloud2)
            points, nr = filter_frame(msg)
            n_raw = int(msg.width) * int(msg.height)
            fh.write(struct.pack("<dIII", stamp_seconds(msg.header), n_raw,
                                 points.size, nr.size))
            fh.write(points.tobytes())
            fh.write(nr.tobytes())
            written += 1
            if written % 100 == 0:
                print(f"[bag-to-frames] {written} кадров", file=sys.stderr, flush=True)
        fh.seek(8)
        fh.write(struct.pack("<I", written))
    print(f"[bag-to-frames] {bag_dir} -> {out_path}: {written} кадров", file=sys.stderr)
    return written


def build_parser():
    parser = argparse.ArgumentParser(description="Дамп rosbag2 PointCloud2 в ARGFRM1")
    parser.add_argument("bag", type=Path, help="каталог rosbag2")
    parser.add_argument("--out", type=Path, required=True, help="выходной ARGFRM1-файл")
    parser.add_argument("--topic", default="", help="топик облака (по умолчанию первый)")
    parser.add_argument("--max-frames", type=int, default=0, help="0 — все кадры")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    convert(args.bag, args.out, args.topic, args.max_frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
