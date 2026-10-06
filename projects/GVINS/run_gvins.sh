#!/bin/bash
# Usage: bash run_gvins.sh [rate] [output_dir] [input_bag] [ROS_port]
set -e
GVINS_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GVINS_BENCH="$(cd -- "$GVINS_PROJECT/../.." && pwd)"
source "$GVINS_PROJECT/env.sh"
source "$GVINS_PROJECT/catkin_ws/devel/setup.bash"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
GVINS_RATE="${1:-1.0}"
GVINS_OUT="${2:-$GVINS_BENCH/results/GVINS/run_$(date -u +%Y%m%dT%H%M%SZ)}"
GVINS_BAG="${3:-$GVINS_PROJECT/bags/texcup_gvins_full.bag}"
GVINS_PORT="${4:-11951}"
TEXCUP_DATA="${TEXCUP_DATA:-$GVINS_BENCH/data/tex_cup}"
exec python -u "$GVINS_PROJECT/run_session.py" --bag "$GVINS_BAG" --out "$GVINS_OUT" \
    --rate "$GVINS_RATE" --port "$GVINS_PORT" --data-dir "$TEXCUP_DATA"
