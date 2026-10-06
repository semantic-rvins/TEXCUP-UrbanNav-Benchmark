#!/bin/bash
# Usage: bash run_icgvins.sh [rate] [output_dir] [bag] [ROS_port] [extra supervisor args]
set -e
IC_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IC_BENCH="$(cd -- "$IC_PROJECT/../.." && pwd)"
source "$IC_PROJECT/env.sh"
source "$IC_PROJECT/catkin_ws/devel/setup.bash"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export OPENCV_FOR_THREADS_NUM="${OPENCV_FOR_THREADS_NUM:-4}"
IC_RATE="${1:-1.0}"
IC_OUT="${2:-$IC_BENCH/results/IC-GVINS/run_$(date -u +%Y%m%dT%H%M%SZ)}"
IC_BAG="${3:-$IC_PROJECT/bags/texcup_icgvins_native.bag}"
IC_PORT="${4:-11961}"
if [ "$#" -ge 4 ]; then shift 4; else set --; fi
TEXCUP_DATA="${TEXCUP_DATA:-$IC_BENCH/data/tex_cup}"
exec python -u "$IC_PROJECT/run_session.py" --bag "$IC_BAG" --out "$IC_OUT" \
  --rate "$IC_RATE" --port "$IC_PORT" --data-dir "$TEXCUP_DATA" "$@"
