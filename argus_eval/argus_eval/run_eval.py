#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Оценка конвейера Argus")
    parser.add_argument("--bag", required=True, help="путь к bag-директории")
    parser.add_argument(
        "--report", action="store_true", help="сгенерировать отчёт")
    args = parser.parse_args(argv)
    print(f"[argus_eval] прогон {args.bag}: каркас, реализация Ф6.7.1")
    if args.report:
        print("[argus_eval] отчёт: каркас")
    return 0


if __name__ == "__main__":
    sys.exit(main())
