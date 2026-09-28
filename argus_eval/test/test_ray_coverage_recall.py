import pytest

from scripts.eval.ray_coverage_recall import analyze, parse_case, summarize


def test_parse_case_builds_dimensions_and_distance():
    dimensions, distance = parse_case("3.0:2.1:2.1:60")
    assert dimensions == (3.0, 2.1, 2.1)
    assert distance == 60.0


def test_parse_case_rejects_wrong_field_count():
    with pytest.raises(ValueError):
        parse_case("3.0:2.1:60")


def test_analyze_matches_alert_at_target_distance():
    rows = [
        {"frame": "0", "alert": "0", "forward_m": "", "clusters": "0", "raw": "0",
         "filtered": "0", "tracks": "0", "range_m": "0", "anom_points": "0",
         "anom_cells": "0"},
        {"frame": "1", "alert": "1", "forward_m": "20.0", "clusters": "1", "raw": "1",
         "filtered": "0", "tracks": "1", "range_m": "20", "anom_points": "1",
         "anom_cells": "1"},
    ]
    truth = {"frames": [
        {"frame": 0, "distance_m": 20.0, "rays_replaced": 30, "rays_added": 0},
        {"frame": 1, "distance_m": 20.0, "rays_replaced": 30, "rays_added": 0},
    ]}
    frames = analyze(rows, truth, tolerance_m=3.0)
    assert frames[0]["active"] is True
    assert frames[0]["matched"] is False
    assert frames[1]["matched"] is True


def test_analyze_alert_far_from_target_is_not_matched():
    rows = [{"frame": "0", "alert": "1", "forward_m": "5.0", "clusters": "1", "raw": "1",
             "filtered": "0", "tracks": "1", "range_m": "5", "anom_points": "1",
             "anom_cells": "1"}]
    truth = {"frames": [{"frame": 0, "distance_m": 60.0, "rays_replaced": 40,
                         "rays_added": 0}]}
    frames = analyze(rows, truth, tolerance_m=3.0)
    assert frames[0]["matched"] is False


def test_analyze_inactive_without_rays():
    rows = [{"frame": "0", "alert": "0", "forward_m": "", "clusters": "0", "raw": "0",
             "filtered": "0", "tracks": "0", "range_m": "0", "anom_points": "0",
             "anom_cells": "0"}]
    truth = {"frames": [{"frame": 0, "distance_m": 60.0, "rays_replaced": 0,
                         "rays_added": 0}]}
    frames = analyze(rows, truth, tolerance_m=3.0)
    assert frames[0]["active"] is False


def test_summarize_bins_by_ray_count():
    frames = [
        {"active": True, "rays": 4, "matched": False},
        {"active": True, "rays": 12, "matched": True},
        {"active": True, "rays": 20, "matched": False},
        {"active": True, "rays": 300, "matched": True},
        {"active": False, "rays": 500, "matched": False},
    ]
    summary = {row["bin"]: row for row in summarize(frames)}
    assert summary["0-8"]["frames"] == 1
    assert summary["8-16"]["matched"] == 1
    assert summary["16-32"]["frames"] == 1
    assert summary["64+"]["frames"] == 1
    assert summary["64+"]["recall"] == 1.0
