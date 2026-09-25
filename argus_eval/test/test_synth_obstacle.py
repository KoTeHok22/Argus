import json
import sys

import numpy as np
import pytest

from scripts.synth_obstacle import (
    insert_box, main, ray_box_intersection, read_frames, read_poses, write_frames,
)


def test_ray_hits_box_in_front():
    origin = np.array([0.0, 0.0, -0.5])
    directions = np.array([[0.0, -1.0, 0.0], [0.2, -1.0, 0.0]])
    box_min = np.array([-0.4, -11.0, -1.2])
    box_max = np.array([0.4, -10.0, -0.1])
    hit, tmin = ray_box_intersection(origin, directions, box_min, box_max)
    assert hit.tolist() == [True, False]
    assert abs(tmin[0] - 10.0) < 1e-6


def test_custom_dimensions_override_preset_and_are_recorded():
    points = np.zeros(1, dtype=np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4"),
    ]))
    points["y"] = -150.0
    points["raw"] = 0
    output, _, truth = insert_box(
        points, 60.0, "person", dimensions=(1.0, 3.0, 2.1),
    )
    assert truth["dimensions_m"] == [1.0, 3.0, 2.1]
    assert truth["rays_replaced"] == 1
    assert abs(float(output["y"][0]) + 60.0) < 1e-6


def test_ray_misses_beside_box():
    origin = np.zeros(3)
    directions = np.array([[0.0, -1.0, 0.2]])
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    box_min = np.array([-0.2, -20.0, -0.2])
    box_max = np.array([0.2, -19.0, 0.2])
    hit, _tmin = ray_box_intersection(origin, directions, box_min, box_max)
    assert hit.tolist() == [False]


def test_world_fixed_box_approaches_with_sensor_translation():
    points = np.array([(0.0, -40.0, -0.5, 1.0, 0)], dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")
    ])
    first, _, info = insert_box(points, 20.0, "person", ray_origin=np.zeros(3))
    second, _, _ = insert_box(points, 20.0, "person", ray_origin=np.array([0.0, -5.0, 0.0]))
    assert info["rays_replaced"] == 1
    assert first["y"][0] == pytest.approx(-20.0, abs=0.1)
    assert second["y"][0] == pytest.approx(-15.0, abs=0.1)


def test_world_fixed_box_respects_sensor_heading():
    points = np.array([(0.0, -40.0, -0.5, 1.0, 0)], dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")
    ])
    theta = np.deg2rad(15.0)
    rotation = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                         [np.sin(theta), np.cos(theta), 0.0],
                         [0.0, 0.0, 1.0]])
    _, _, info = insert_box(points, 20.0, "person", ray_rotation=rotation)
    assert info["rays_replaced"] == 0


def test_read_poses_rejects_missing_frames(tmp_path):
    path = tmp_path / "poses.csv"
    path.write_text("frame,valid,x,y,z,heading_deg\n0,1,0,0,0,0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="1 poses for 2 frames"):
        read_poses(path, [None, None])


def test_parallel_ray_outside_box_does_not_hit():
    hit, _ = ray_box_intersection(
        np.array([1.0, 0.0, 0.0]), np.array([[0.0, -1.0, 0.0]]),
        np.array([-0.2, -20.0, -0.2]), np.array([0.2, -19.0, 0.2]),
    )
    assert hit.tolist() == [False]


def test_box_behind_sensor_does_not_replace_return():
    points = np.array([(0.0, -40.0, -0.5, 1.0, 0)], dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")
    ])
    out, _, info = insert_box(points, 20.0, "person", ray_origin=np.array([0.0, -30.0, 0.0]))
    assert info["rays_replaced"] == 0
    np.testing.assert_array_equal(out, points)


def test_read_poses_rejects_invalid_pose(tmp_path):
    path = tmp_path / "poses.csv"
    path.write_text("frame,valid,x,y,z,heading_deg\n0,1,0,0,0,0\n1,0,0,-1,0,0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid pose at frame 1"):
        read_poses(path, [None, None])


def test_pose_csv_translates_and_rotates_box_in_sensor_frame(tmp_path):
    path = tmp_path / "poses.csv"
    path.write_text(
        "frame,valid,x,y,z,heading_deg\n"
        "0,1,0,0,0,0\n"
        "1,1,0,-5,0,0\n"
        "2,1,0,-5,0,15\n",
        encoding="utf-8",
    )
    poses = read_poses(path, [None] * 3)
    anchor_position, anchor_rotation = poses[0]
    near_corner = np.array([0.0, -20.0, -1.2])
    forward = []
    for position, rotation in poses:
        origin = anchor_rotation.T @ (position - anchor_position)
        relative_rotation = anchor_rotation.T @ rotation
        current_near = relative_rotation.T @ (near_corner - origin)
        forward.append(-current_near[1])
    assert forward == pytest.approx([20.0, 15.0, 15.0 * np.cos(np.deg2rad(15.0))])


def test_world_fixed_cli_preserves_scene_and_records_per_frame_truth(tmp_path, monkeypatch):
    source = tmp_path / "source.frames"
    output = tmp_path / "moving.frames"
    truth = tmp_path / "truth.json"
    poses = tmp_path / "poses.csv"
    points = np.array([(0.0, -40.0, -0.5, 1.0, 0)], dtype=[
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("raw", "<u4")
    ])
    write_frames(source, [(float(k), 1, points, np.empty(0, dtype="<u4")) for k in range(3)])
    poses.write_text(
        "frame,valid,x,y,z,heading_deg\n"
        "0,1,0,0,0,0\n"
        "1,1,0,-5,0,0\n"
        "2,1,0,-10,0,0\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(sys, "argv", [
        "synth_obstacle.py", str(source), "--out", str(output), "--truth", str(truth),
        "--distance", "20", "--preset", "person", "--frames", "3",
        "--start-frame", "1", "--poses", str(poses),
    ])
    main()
    generated = read_frames(output)
    np.testing.assert_array_equal(generated[0][2], points)
    assert generated[1][2]["y"][0] == pytest.approx(-20.0, abs=0.1)
    assert generated[2][2]["y"][0] == pytest.approx(-15.0, abs=0.1)
    report = json.loads(truth.read_text(encoding="utf-8"))
    assert report["mode"] == "world_fixed"
    assert report["frames"][0]["distance_m"] is None
    assert [frame["distance_m"] for frame in report["frames"][1:]] == pytest.approx([20, 15])
    assert [frame["rays_replaced"] for frame in report["frames"]] == [0, 1, 1]
    assert len(report["frames"][2]["box_corners_sensor"]) == 8
