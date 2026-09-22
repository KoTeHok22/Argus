import numpy as np

from scripts.synth_obstacle import ray_box_intersection


def test_ray_hits_box_in_front():
    origin = np.zeros(3)
    directions = np.array([[0.0, -1.0, 0.0], [0.2, -1.0, 0.0]])
    box_min = np.array([-0.4, -11.0, -1.2])
    box_max = np.array([0.4, -10.0, -0.1])
    hit, tmin = ray_box_intersection(origin, directions, box_min, box_max)
    assert hit.tolist() == [True, False]
    assert abs(tmin[0] - 10.0) < 1e-6


def test_ray_misses_beside_box():
    origin = np.zeros(3)
    directions = np.array([[0.0, -1.0, 0.2]])
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    box_min = np.array([-0.2, -20.0, -0.2])
    box_max = np.array([0.2, -19.0, 0.2])
    hit, _tmin = ray_box_intersection(origin, directions, box_min, box_max)
    assert hit.tolist() == [False]
