# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from argus_eval.metrics import EmptyRunMetrics, SyntheticMetrics, recall_by_range


def test_fp_rate_and_minute():
    m = EmptyRunMetrics(
        bag="x", total_frames=1000, frames_with_alert=40,
        duration_s=100.0, max_alert_streak=3)
    assert abs(m.fp_rate - 0.04) < 1e-9
    assert abs(m.fp_per_minute - 24.0) < 1e-9
    assert m.ok()


def test_fp_rate_fails_threshold():
    m = EmptyRunMetrics(
        bag="x", total_frames=100, frames_with_alert=10,
        duration_s=10.0, max_alert_streak=50)
    assert not m.ok()  # streak 50 > 10


def test_recall_by_range():
    ms = [
        SyntheticMetrics("b", 30.0, "cart", True),
        SyntheticMetrics("b", 35.0, "cart", False),
        SyntheticMetrics("b", 250.0, "cart", False),
    ]
    curve = recall_by_range(ms)
    assert abs(curve[25.0] - 0.5) < 1e-9
    assert curve[250.0] == 0.0
    assert 275.0 not in curve  # пустой бин не попадает в кривую
