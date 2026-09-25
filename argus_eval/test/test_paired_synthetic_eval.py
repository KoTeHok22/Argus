import pytest

from scripts.paired_synthetic_eval import coverage_report, evaluate, summarize_coverage


def row(frame, alert=0, distance=-1):
    return {
        "frame": str(frame), "alert": str(alert),
        "forward_m": str(distance),
    }


def test_baseline_alert_at_target_distance_is_contested():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    result = evaluate(
        [row(0, 1, 59)], [row(0, 1, 60)], truth, 3,
    )
    assert result[0]["matched"]
    assert not result[0]["uncontested"]


def test_other_background_alert_is_still_contested():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    result = evaluate(
        [row(0, 1, 30)], [row(0, 1, 60)], truth, 3,
    )
    assert result[0]["matched"]
    assert not result[0]["uncontested"]


def test_target_match_without_baseline_alert_is_uncontested():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    assert evaluate([row(0)], [row(0, 1, 60)], truth, 3)[0]["uncontested"]


def test_absent_target_and_misaligned_csv_are_rejected():
    truth = {"frames": [{"frame": 0, "distance_m": None}]}
    assert not evaluate([row(0)], [row(0, 1, 20)], truth, 3)[0]["matched"]
    with pytest.raises(ValueError, match="frame mismatch"):
        evaluate([row(1)], [row(0)], truth, 3)


def test_truth_without_per_frame_distance_is_rejected():
    with pytest.raises(ValueError, match="lengths"):
        evaluate([row(0)], [row(0)], {"distance_m": 60}, 3)


def test_coverage_summary_marks_undercovered_frames_unknown():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    result = evaluate([row(0)], [row(0, 1, 60)], truth, 3)
    summary = summarize_coverage([result[0]], [{"rays": 2, "rings": 4}], 8, 2)
    assert summary == {
        "eligible": 0, "unknown": 1, "inactive": 0, "missed": 0,
        "matched": 0, "uncontested": 0,
    }


def test_coverage_summary_counts_eligible_match():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    result = evaluate([row(0)], [row(0, 1, 60)], truth, 3)
    summary = summarize_coverage([result[0]], [{"rays": 8, "rings": 2}], 8, 2)
    assert summary == {
        "eligible": 1, "unknown": 0, "inactive": 0, "missed": 0,
        "matched": 1, "uncontested": 1,
    }


def test_coverage_summary_counts_eligible_miss_separately():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    result = evaluate([row(0)], [row(0)], truth, 3)
    summary = summarize_coverage([result[0]], [{"rays": 8, "rings": 2}], 8, 2)
    assert summary["eligible"] == 1
    assert summary["missed"] == 1
    assert summary["inactive"] == 0


def test_coverage_report_adds_rates():
    truth = {"frames": [{"frame": 0, "distance_m": 60.0}]}
    result = evaluate([row(0)], [row(0, 1, 60)], truth, 3)
    report = coverage_report([result[0]], [{"rays": 8, "rings": 2}], 8, 2)
    assert report["total"] == 1
    assert report["eligible_rate"] == 1.0
    assert report["matched_rate"] == 1.0
