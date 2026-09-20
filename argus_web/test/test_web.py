from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ARGUS_ROOT", str(ROOT.parent))
os.environ["ARGUS_DATA"] = str(Path(__file__).resolve().parent / "_empty_data")
os.environ["ARGUS_RESULTS"] = str(Path(__file__).resolve().parent / "_empty_results")
os.environ["ARGUS_UI_ROOT"] = str(Path(__file__).resolve().parent / "_empty_ui")

from argus_web.bags import parse_metadata
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
