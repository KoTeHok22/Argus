import numpy as np
import pytest

from scripts.rail_coverage_probe import paired_coverage


POINTS = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])


def test_paired_coverage_stops_at_first_missing_side():
    points = np.array([
        (1.5, -2.0, -0.8), (-1.5, -2.0, -0.8),
        (1.5, -7.0, -0.8), (-1.5, -7.0, -0.8),
        (1.5, -12.0, -0.8),
        (1.5, -17.0, -0.8), (-1.5, -17.0, -0.8),
    ], dtype=POINTS)
    coverage, left, right = paired_coverage(points, min_points=1, max_range_m=20)
    assert coverage == 10
    assert left == [1, 1, 1, 1]
    assert right == [1, 1, 0, 1]


def test_ground_height_and_width_are_only_proxies():
    points = np.array([
        (1.5, -2.0, -0.8), (-1.5, -2.0, -0.8),
        (1.5, -7.0, 0.2), (-1.5, -7.0, -0.8),
        (0.0, -12.0, -0.8),
    ], dtype=POINTS)
    coverage, _, _ = paired_coverage(points, min_points=1, max_range_m=15)
    assert coverage == 5


def test_invalid_coverage_parameters_fail():
    with pytest.raises(ValueError):
        paired_coverage(np.array([], dtype=POINTS), bin_m=0)
