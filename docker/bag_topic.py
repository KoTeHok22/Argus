#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import sys


def main() -> int:
    bag = sys.argv[1] if len(sys.argv) > 1 else ""
    fallback = "/lidar_points"
    if not bag:
        print(fallback)
        return 0
    meta = os.path.join(bag, "metadata.yaml")
    if not os.path.isfile(meta):
        print(fallback)
        return 0
    names = re.findall(r"name:\s*(/\S+)", open(meta, encoding="utf-8", errors="ignore").read())
    prefer = ("/lidar_points", "/sensing/lidar/hesai128/pointcloud")
    for want in prefer:
        if want in names:
            print(want)
            return 0
    print(names[0] if names else fallback)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
