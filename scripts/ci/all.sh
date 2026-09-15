#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
set -euo pipefail
cd "$(dirname "$0")/.."
./scripts/ci/lint.sh
./scripts/ci/build.sh
./scripts/ci/test.sh
