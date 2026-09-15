#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
#
# Импорт внешних зависимостей по манифесту (PLAN.md 5.1.7).
# После первого успешного билда заменить ветки в argus.repos на SHA (16.3.6).

set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p third_party/src
vcs import third_party/src < third_party/argus.repos
echo "OK: зависимости импортированы в third_party/src"
