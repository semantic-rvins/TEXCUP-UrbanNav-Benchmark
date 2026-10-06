#!/bin/bash
# Activate the ROS-capable Python used for native result extraction.
set -e
MONITOR_BENCH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$MONITOR_BENCH/projects/GVINS/env.sh"
exec python -u "$MONITOR_BENCH/common/monitor_runs.py" "$@"
