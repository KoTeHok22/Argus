#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
#
# Линт: clang-format dry-run по C++ и базовые проверки репозитория.

set -euo pipefail
cd "$(dirname "$0")/../.."

echo "[lint] clang-format check"
find argus_core argus_msgs argus_node_* -name '*.cpp' -o -name '*.hpp' | while read -r f; do
  clang-format --dry-run --Werror "$f"
done

echo "[lint] конфиденциальное не в git"
if git ls-files 2>/dev/null | grep -E '^(data/|data/).*|\.(db3|zst|pcd|npz)$'; then
  echo "ОШИБКА: конфиденциальные/данные файлы в git" >&2
  exit 1
fi

echo "[lint] OK"
