# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
"""Runner: офлайн-прогон конвейера на bag быстрее real-time (PLAN.md 11.2.1).

Вариант A (рекомендован для старта): ros2-ноды с rate=0, сбор /argus/obstacles.
Вариант B: полностью офлайн на npz. Здесь - каркас.
"""

from __future__ import annotations

import argparse
import sys


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Оценка конвейера Argus")
    parser.add_argument("--bag", required=True, help="путь к bag-директории")
    parser.add_argument("--report", action="store_true", help="сгенерировать отчёт")
    args = parser.parse_args(argv)

    # Ф6.7.1: реализация прогона.
    print(f"[argus_eval] прогон {args.bag}: каркас, реализация Ф6.7.1")
    if args.report:
        print("[argus_eval] отчёт: каркас")
    return 0


if __name__ == "__main__":
    sys.exit(main())
