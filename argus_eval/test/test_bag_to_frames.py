import struct
from types import SimpleNamespace

import pytest

from scripts.eval.bag_to_frames import filter_frame

FLOAT32 = 7


def field(name, offset, datatype=FLOAT32, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def make_cloud(points, point_step=None, fields=None):
    layout = point_step if point_step is not None else 16
    buffer = bytearray(len(points) * layout)
    for index, point in enumerate(points):
        x, y, z, intensity = point
        struct.pack_into("<ffff", buffer, index * layout, x, y, z, intensity)
    if fields is None:
        fields = [field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)]
    return SimpleNamespace(fields=fields, width=len(points), height=1,
                           point_step=layout, data=bytes(buffer))


def test_keeps_valid_points_and_raw_indices():
    msg = make_cloud([(1.0, -2.0, 0.5, 7.0), (0.1, 0.1, 0.1, 3.0), (5.0, 5.0, 1.0, 9.0)])
    points, nr = filter_frame(msg)
    assert points["raw"].tolist() == [0, 2]
    assert points["y"].tolist() == [-2.0, 5.0]
    assert points["i"].tolist() == [7.0, 9.0]
    assert nr.tolist() == []


def test_zero_xyz_becomes_no_return():
    msg = make_cloud([(0.0, 0.0, 0.0, 0.0), (4.0, 0.0, 0.0, 1.0)])
    points, nr = filter_frame(msg)
    assert points["raw"].tolist() == [1]
    assert nr.tolist() == [0]


def test_nonfinite_is_dropped_without_no_return():
    msg = make_cloud([(float("nan"), 1.0, 1.0, 0.0), (2.0, 0.0, 0.0, 1.0)])
    points, nr = filter_frame(msg)
    assert points["raw"].tolist() == [1]
    assert nr.tolist() == []


def test_range_outside_filter_is_dropped():
    msg = make_cloud([(0.4, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 1.0), (500.0, 0.0, 0.0, 2.0)])
    points, nr = filter_frame(msg)
    assert points["raw"].tolist() == [1]
    assert nr.tolist() == []


def test_far_abs_coordinate_is_dropped():
    msg = make_cloud([(20000.0, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 1.0)])
    points, nr = filter_frame(msg)
    assert points["raw"].tolist() == [1]


def test_missing_intensity_fills_zero():
    fields = [field("x", 0), field("y", 4), field("z", 8)]
    buffer = bytearray(2 * 16)
    struct.pack_into("<fff", buffer, 0, 1.0, 0.0, 0.0)
    struct.pack_into("<fff", buffer, 16, 2.0, 0.0, 0.0)
    msg = SimpleNamespace(fields=fields, width=2, height=1, point_step=16, data=bytes(buffer))
    points, _ = filter_frame(msg)
    assert points["i"].tolist() == [0.0, 0.0]


def test_respects_padded_point_step():
    fields = [field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)]
    buffer = bytearray(2 * 32)
    struct.pack_into("<ffff", buffer, 0, 1.0, 0.0, 0.0, 5.0)
    struct.pack_into("<ffff", buffer, 32, 3.0, 0.0, 0.0, 6.0)
    msg = SimpleNamespace(fields=fields, width=2, height=1, point_step=32, data=bytes(buffer))
    points, _ = filter_frame(msg)
    assert points["i"].tolist() == [5.0, 6.0]
    assert points["raw"].tolist() == [0, 1]


def test_rejects_cloud_without_xyz():
    fields = [field("intensity", 0)]
    msg = SimpleNamespace(fields=fields, width=1, height=1, point_step=4, data=b"\x00\x00\x00\x00")
    with pytest.raises(ValueError):
        filter_frame(msg)


def test_rejects_truncated_buffer():
    msg = make_cloud([(1.0, 0.0, 0.0, 0.0)])
    msg.data = msg.data[:-4]
    with pytest.raises(ValueError):
        filter_frame(msg)
