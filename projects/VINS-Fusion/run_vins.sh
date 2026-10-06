#!/bin/bash
# Usage: bash run_vins.sh [bag] [rate] [output_dir] [ROS_port] [extra supervisor args]
set -e
VINS_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VINS_BENCH="$(cd -- "$VINS_PROJECT/../.." && pwd)"
source "$VINS_PROJECT/../GVINS/env.sh"
source "$VINS_PROJECT/catkin_ws/devel/setup.bash"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export OPENCV_FOR_THREADS_NUM="${OPENCV_FOR_THREADS_NUM:-4}"
VINS_BAG="${1:-$VINS_PROJECT/bags/texcup_stereo_native.bag}"
VINS_RATE="${2:-1.0}"
VINS_OUT="${3:-$VINS_BENCH/results/VINS-Fusion/run_$(date -u +%Y%m%dT%H%M%SZ)}"
VINS_PORT="${4:-11952}"
if [ "$#" -ge 4 ]; then shift 4; else set --; fi
TEXCUP_DATA="${TEXCUP_DATA:-$VINS_BENCH/data/tex_cup}"
exec python -u "$VINS_PROJECT/run_session.py" --bag "$VINS_BAG" --out "$VINS_OUT" \
    --rate "$VINS_RATE" --port "$VINS_PORT" --data-dir "$TEXCUP_DATA" "$@"
