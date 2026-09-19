#!/usr/bin/env bash
# Copyright 2026 Argus Team
# Licensed under the Apache License, Version 2.0
set -eo pipefail
cd "$(dirname "$0")/../.."

source install/setup.bash

total=0
for t in build/argus_core/test_*; do
  [ -x "$t" ] && [ -f "$t" ] || continue
  out=$("$t" 2>&1 | tail -1) || true
  printf '%s: %s\n' "$(basename "$t")" "$out"
  n=$(printf '%s' "$out" | grep -oE '^\[  PASSED  \] [0-9]+' | grep -oE '[0-9]+' || true)
  total=$((total + ${n:-0}))
done
printf 'GTest total: %s\n' "$total"
