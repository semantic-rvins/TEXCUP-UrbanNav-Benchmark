# Source this to activate the local RoboStack ROS Noetic environment.
# Override these paths when moving the benchmark to another machine.
BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
BENCH_ROS_ENV="${BENCH_ROS_ENV:-$BENCH_CONDA_ROOT/envs/benchmark_ros}"
if [[ ! -f "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh" || ! -d "$BENCH_ROS_ENV/conda-meta" ]]; then
    printf 'Missing Conda/ROS environment: %s (see RUN_INSTRUCTIONS.md)\n' "$BENCH_ROS_ENV" >&2
    return 1
fi
source "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh"
conda activate "$BENCH_ROS_ENV"
export PYTHONNOUSERSITE=1
