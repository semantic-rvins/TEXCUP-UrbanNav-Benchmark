# Source this to activate IC-GVINS's isolated RoboStack environment.
BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
BENCH_IC_ENV="${BENCH_IC_ENV:-$BENCH_CONDA_ROOT/envs/benchmark_icgvins}"
if [[ ! -d "$BENCH_IC_ENV/conda-meta" ]]; then
    printf 'Missing IC-GVINS environment: %s (see RUN_INSTRUCTIONS.md)\n' "$BENCH_IC_ENV" >&2
    return 1
fi
source "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh"
conda activate "$BENCH_IC_ENV"
export PYTHONNOUSERSITE=1
