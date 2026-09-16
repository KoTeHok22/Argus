#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
# -u несовместим с source setup.bash (unbound AMENT_*), см. PLAN.md 16.3.2
set -eo pipefail
cd "$(dirname "$0")/../.."

source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "[build] OK"
