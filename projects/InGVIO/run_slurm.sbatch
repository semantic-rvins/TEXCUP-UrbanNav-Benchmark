#!/bin/bash
#SBATCH -c 8
#SBATCH --mem=16G
#SBATCH -t 3:00:00
#SBATCH -J ingvio_texcup
#SBATCH -o ingvio_slurm_%j.log
# Submit from the repository root, or export BENCH_ROOT. See RUN.md for preparation.
BENCH_ROOT="${BENCH_ROOT:-${SLURM_SUBMIT_DIR:-$PWD}}"
bash "$BENCH_ROOT/projects/InGVIO/run_ingvio.sh" 1.0
