from __future__ import annotations

import os
import struct
import sys
from pathlib import Path

from argus_web.cloud import header_stamp_ns, load_frame

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ARGUS_ROOT", str(ROOT.parent))
os.environ["ARGUS_DATA"] = str(Path(__file__).resolve().parent / "_empty_data")
os.environ["ARGUS_RESULTS"] = str(Path(__file__).resolve().parent / "_empty_results")
os.environ["ARGUS_UI_ROOT"] = str(Path(__file__).resolve().parent / "_empty_ui")

from argus_web.align import align_rows, gauge_profile, load_gauge, reset_gauge, save_gauge, verdict_for_stamp
from argus_web.bags import describe, parse_metadata
from argus_web.reports import csv_to_text, load_report, percentile, summarize_latency


def test_percentile_empty():
    assert percentile([], 50) is None


def test_percentile_mid():
    assert abs(percentile([10.0, 20.0, 30.0], 50) - 20.0) < 1e-9


def test_summarize_latency_blocked():
    rows = [
        {"status": "CLEAR", "nearest_m": "-1", "objects": "0", "fps": "10", "t_alert_path_ms": "20"},
        {"status": "BLOCKED", "nearest_m": "16.9", "objects": "1", "fps": "10", "t_alert_path_ms": "28"},
        {"status": "BLOCKED", "nearest_m": "17.1", "objects": "1", "fps": "9.9", "t_alert_path_ms": "32"},
        {"status": "DEGRADED", "nearest_m": "-1", "objects": "0", "fps": "0", "t_alert_path_ms": "0"},
    ]
    out = summarize_latency(rows)
    assert out["frames"] == 3
    assert out["blocked"] == 2
    assert out["clear"] == 1
    assert out["degraded"] == 1
    assert abs(out["nearest_m"] - 16.9) < 1e-6
    assert out["first_blocked_frame"] == 1
    assert out["rows"][1]["status_label"] == "препятствие"


def test_summarize_latency_includes_unprofiled_blocked_rows():
    rows = [
        {"status": "BLOCKED", "nearest_m": "12.5", "objects": "1", "fps": "10", "t_alert_path_ms": ""},
        {"status": "CLEAR", "nearest_m": "-1", "objects": "0", "fps": "10", "t_alert_path_ms": ""},
    ]
    out = summarize_latency(rows)
    assert out["frames"] == 2
    assert out["blocked"] == 1
    assert out["clear"] == 1


def test_summarize_latency_reads_obstacle_geometry():
    rows = [
        {
            "stamp_ns": "123",
            "status": "BLOCKED",
            "nearest_m": "16.9",
            "objects": "1",
            "obstacles_json": '[{"track_id":7,"position":[1,-16,0.5],"extent":[0.3,0.4,0.8],"point_count":12,"reason":"geometry"}]',
        }
    ]
    row = summarize_latency(rows)["rows"][0]
    assert row["obstacles"][0]["track_id"] == 7
    assert row["obstacles"][0]["extent"] == [0.3, 0.4, 0.8]


def test_load_report_fusion(tmp_path: Path):
    path = tmp_path / "fusion_obstacle.csv"
    path.write_text(
        "frame,alert,clusters,raw,filtered,tracks,forward_m,range_m,anom_points,anom_cells\n"
        "0,0,0,0,0,0,-1.00,-1.00,0,0\n"
        "3,1,2,2,0,1,16.83,16.90,101,320\n",
        encoding="utf-8",
    )
    report = load_report(path)
    assert report["name"] == "doubleT_obstacle"
    assert report["blocked"] == 1
    assert report["clear"] == 1
    assert abs(report["nearest_m"] - 16.9) < 1e-6
    text = csv_to_text(report)
    assert "Препятствие: 1" in text
    assert "16.90" in text


def test_parse_metadata(tmp_path: Path):
    bag = tmp_path / "doubleT_obstacle"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  duration:\n"
        "    nanoseconds: 20000000000\n"
        "  message_count: 201\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata:\n"
        "        name: /sensing/lidar/hesai128/pointcloud\n"
        "  relative_file_paths:\n"
        "    - doubleT_obstacle_0.db3\n",
        encoding="utf-8",
    )
    meta = parse_metadata(bag)
    assert meta["topic"] == "/sensing/lidar/hesai128/pointcloud"
    assert meta["frames"] == 201
    assert abs(meta["duration_ns"] / 1e9 - 20.0) < 1e-9
    assert meta["files"] == ["doubleT_obstacle_0.db3"]


def test_header_stamp_matches_report_clock():
    blob = b"\x00\x01\x00\x00" + struct.pack("<iI", 946687297, 499959946)
    assert header_stamp_ns(blob) == 946687297499959946
    assert header_stamp_ns(b"\x00\x00\x00\x00" + blob[4:]) is None
    frames = [946687297199933052, 946687297499959946]
    rows = [
        {"stamp_ns": "1789912875198547104", "status": "DEGRADED", "nearest_m": "-1", "t_alert_path_ms": "0"},
        {"stamp_ns": "946687297199933052", "status": "CLEAR", "nearest_m": "-1", "t_alert_path_ms": "56.5"},
        {"stamp_ns": "946687297499959946", "status": "BLOCKED", "nearest_m": "16.896", "t_alert_path_ms": "31.2"},
    ]
    aligned = align_rows(frames, rows)
    assert aligned["dropped"] == 1
    hit = verdict_for_stamp(946687297499959946, aligned, True)
    assert hit["text"] == "Препятствие, 16,9 м"
    miss = verdict_for_stamp(1, aligned, True)
    assert miss["text"] == "Детектор не уверен"
    assert verdict_for_stamp(946687297499959946, None, False)["text"] == "Детектор ещё не прогнан"


def test_foreign_report_does_not_say_clear():
    aligned = align_rows(
        [100],
        [{"stamp_ns": "900", "status": "CLEAR", "nearest_m": "-1", "t_alert_path_ms": "20"}],
    )
    assert aligned["rows"] == []
    assert verdict_for_stamp(100, aligned, True)["text"] == "Детектор не уверен"


def test_gauge_lines_from_yaml(tmp_path: Path):
    path = tmp_path / "argus_params.yaml"
    path.write_text(
        "argus_node_fusion:\n"
        "  ros__parameters:\n"
        "    gauge:\n"
        "      forward_axis: \"-y\"\n"
        "      half_width: 1.50\n"
        "      height: 2.10\n"
        "      base_offset: 0.20\n"
        "      chamfer: 0.20\n"
        "      nose_offset: 0.00\n"
        "      sensor_height: 1.20\n"
        "      max_range: 300.0\n"
        "      safety_margin: 0.10\n",
        encoding="utf-8",
    )
    gauge = load_gauge(path)
    profile = gauge_profile(gauge, view_range=10.0)
    assert abs(profile["left"][0][0] - -1.50) < 1e-6
    assert abs(profile["right"][0][0] - 1.50) < 1e-6
    assert abs(profile["left"][1][1] - -10.0) < 1e-6


def test_save_gauge_then_reset(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("ARGUS_UI_ROOT", str(tmp_path / "ui"))
    saved = save_gauge({"half_width": 2.0, "forward_axis": "+x"})
    assert saved["half_width"] == 2.0
    assert saved["forward_axis"] == "+x"
    again = load_gauge()
    assert again["half_width"] == 2.0
    restored = reset_gauge()
    assert restored["half_width"] == 1.5
    assert not (tmp_path / "ui" / "params.yaml").exists()


def test_active_params_file_prefers_ui_override(tmp_path: Path, monkeypatch):
    from argus_web import runner

    base = tmp_path / "argus_params.yaml"
    override = tmp_path / "params.yaml"
    base.write_text("base", encoding="utf-8")
    override.write_text("override", encoding="utf-8")
    monkeypatch.setattr(runner, "params_yaml", lambda: base)
    monkeypatch.setattr(runner, "ui_params_yaml", lambda: override)
    assert runner.active_params_file() == override


def test_saved_gauge_overrides_detector_configuration(tmp_path: Path, monkeypatch):
    from argus_web import align, runner

    base = tmp_path / "argus_params.yaml"
    base.write_text(
        "/**:\n  ros__parameters:\n    gauge:\n"
        "      half_width: 1.50\n      safety_margin: 0.10\n"
        "    fusion:\n      min_votes_for_alert: 1\n",
        encoding="utf-8",
    )
    override = tmp_path / "params.yaml"
    monkeypatch.setattr(align, "params_yaml", lambda: base)
    monkeypatch.setattr(align, "ui_params_yaml", lambda: override)
    monkeypatch.setattr(runner, "params_yaml", lambda: base)
    monkeypatch.setattr(runner, "ui_params_yaml", lambda: override)
    save_gauge({"half_width": 1.7, "safety_margin": 0.08})
    assert runner.active_params_file() == override
    text = override.read_text(encoding="utf-8")
    assert text.startswith("/**:\n  ros__parameters:\n")
    assert "half_width: 1.7000" in text
    assert "safety_margin: 0.0800" in text
    assert "min_votes_for_alert: 1" in text


def test_docker_run_mounts_active_gauge(tmp_path: Path, monkeypatch):
    from argus_web import runner

    override = tmp_path / "params.yaml"
    override.write_text("/**:\n", encoding="utf-8")
    monkeypatch.setattr(runner.shutil, "which", lambda _: "/usr/bin/docker")
    cmd = runner._detect_cmd(
        {"id": "sample", "path": str(tmp_path / "sample")},
        1.0, tmp_path / "result.csv",
    )
    assert f"{tmp_path}:/tmp/argus_runs" in cmd
    assert cmd[-1] == "/tmp/argus_runs/result_params.yaml"


def test_docker_runner_preserves_workspace_absolute_paths(monkeypatch):
    from argus_web import runner

    monkeypatch.setattr(runner.shutil, "which", lambda _: "/usr/bin/docker")
    cmd = runner._detect_cmd(
        {"id": "cloud_with_fake_obj", "path": "/data/cloud_with_fake_obj"},
        0.0,
        Path("/ws/data/ui/runs/cloud_with_fake_obj_run.csv"),
    )
    assert "/ws:/ws" in cmd
    assert "ARGUS_LATENCY_CSV=/ws/data/ui/runs/cloud_with_fake_obj_run.csv" in cmd
    assert "/ws/data/ui/runs/cloud_with_fake_obj_run_params.yaml" in cmd


def test_argfrm_synthetic_preview_is_not_ros_bag(tmp_path: Path):
    bag = tmp_path / "person20"
    bag.mkdir()
    point = struct.pack("<ffffI", 0.2, -20.0, 0.5, 1.0, 12)
    with (bag / "cloud.argfrm").open("wb") as fh:
        fh.write(b"ARGFRM1\0")
        fh.write(struct.pack("<I", 1))
        fh.write(struct.pack("<dIII", 0.0, 128, 1, 0))
        fh.write(point)
    info = describe(bag, {})
    cloud = load_frame(bag, 0)
    assert info["synthetic"] is True
    assert info["status"] == "ready"
    assert cloud["total"] == 1
    assert cloud["raw_points"] == 1
    assert abs(float(cloud["xyz"][0, 0]) - 0.2) < 1e-6
    assert float(cloud["xyz"][0, 1]) == -20.0
    assert float(cloud["xyz"][0, 2]) == 0.5
