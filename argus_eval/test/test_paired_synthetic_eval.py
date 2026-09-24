import pytest

from scripts.paired_synthetic_eval import evaluate


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
