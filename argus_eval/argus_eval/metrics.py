# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
"""Метрики качества Argus (PLAN.md 11.1).

Принципиальное ограничение задачи: покадровой разметки препятствий НЕТ.
Метрики делятся на два класса:

A. Пустые участки (ground truth тривиален: препятствий нет)
   - fp_rate, fp_per_minute, длительность удержания ложной тревоги.

B. Синтетические препятствия (ground truth известен с точностью до см,
   потому что мы сами вставили объект)
   - recall, ошибка дистанции, кривая "дальность vs recall".
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class EmptyRunMetrics:
    """Метрики прогона на участке без препятствий."""

    bag: str
    total_frames: int
    frames_with_alert: int
    duration_s: float
    max_alert_streak: int  # сколько кадров подряд держалась ложная тревога

    @property
    def fp_rate(self) -> float:
        return self.frames_with_alert / max(1, self.total_frames)

    @property
    def fp_per_minute(self) -> float:
        return self.frames_with_alert / max(1e-6, self.duration_s / 60.0)

    def ok(self) -> bool:
        """Целевые значения PLAN.md 11.2.2: fp_rate < 0.05, streak < 10 кадров."""
        return self.fp_rate < 0.05 and self.max_alert_streak < 10


@dataclass
class SyntheticMetrics:
    """Метрики на синтетически вставленном препятствии."""

    bag: str
    true_range_m: float
    preset: str
    detected: bool
    detected_range_m: Optional[float] = None
    detected_frame: Optional[int] = None
    frames_to_detect: Optional[int] = None

    @property
    def range_error_m(self) -> Optional[float]:
        if self.detected_range_m is None:
            return None
        return abs(self.detected_range_m - self.true_range_m)


def recall_by_range(
    metrics: list[SyntheticMetrics],
    bin_size_m: float = 25.0,
    max_range_m: float = 300.0,
) -> dict[float, float]:
    """Кривая recall по бинам дистанции - прямой ответ на критерий G2."""
    out: dict[float, float] = {}
    b = 0.0
    while b < max_range_m:
        in_bin = [m for m in metrics if b <= m.true_range_m < b + bin_size_m]
        if in_bin:
            out[b] = sum(1 for m in in_bin if m.detected) / len(in_bin)
        b += bin_size_m
    return out
