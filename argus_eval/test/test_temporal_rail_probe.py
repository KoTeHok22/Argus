import numpy as np
import pytest

from scripts.temporal_rail_probe import accumulated_centerline


POINTS = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
])


def frame(offset=0):
    points = np.array([
        (1.5, -2 - offset, -0.8), (-1.5, -2 - offset, -0.8),
        (1.5, -7 - offset, -0.8), (-1.5, -7 - offset, -0.8),
        (1.5, -17 - offset, -0.8), (-1.5, -17 - offset, -0.8),
    ], dtype=POINTS)
    return (0, 0, points, np.array([]))


def pose(offset=0):
    return (np.array([0.0, offset, 0.0]), np.eye(3))


def test_requires_independent_frames_and_stops_at_gap():
    frames = [frame(), frame()]
    poses = [pose(), pose()]
    assert accumulated_centerline(frames, poses, 0, min_points=1) == []
    line = accumulated_centerline(frames, poses, 1, min_points=1)
    assert [point[0] for point in line] == [2.5, 7.5]
    assert line[0][2:] == (2, 2)


def test_motion_compensation_keeps_shared_world_features():
    frames = [frame(), frame(2)]
    poses = [pose(), pose(2)]
    line = accumulated_centerline(frames, poses, 1, min_points=1)
    assert [point[0] for point in line] == [2.5, 7.5]


def test_window_excludes_old_observations():
    frames = [frame(), frame(), frame()]
    poses = [pose(), pose(), pose()]
    assert accumulated_centerline(
        frames, poses, 2, window=1, min_points=1) == []
    assert len(accumulated_centerline(
        frames, poses, 2, window=2, min_points=1)) == 2


def test_invalid_parameters_fail_closed():
    with pytest.raises(ValueError):
        accumulated_centerline([frame()], [pose()], 0, window=0)


def test_support_reports_valid_bins_past_first_gap_without_extending_path():
    frames = [frame(), frame()]
    poses = [pose(), pose()]
    line, support = accumulated_centerline(
        frames, poses, 1, min_points=1, max_range_m=20,
        return_support=True)
    assert len(line) == 2
    assert [entry[-1] for entry in support] == [True, True, False, True]
    assert support[2][1:3] == (0, 0)
