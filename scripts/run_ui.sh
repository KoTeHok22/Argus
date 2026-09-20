#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
export ARGUS_ROOT="${ARGUS_ROOT:-$PWD}"
export PYTHONPATH="$PWD/argus_web${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m argus_web --host 0.0.0.0 --port "${ARGUS_WEB_PORT:-8080}"
