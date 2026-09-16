#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
# -u несовместим с source setup.bash (unbound AMENT_*), см. PLAN.md 16.3.2
set -eo pipefail
cd "$(dirname "$0")/../.."

source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
echo "[test] OK"
