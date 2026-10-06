#!/bin/bash
#SBATCH -c 12
#SBATCH --mem=224000M
#SBATCH --time=48:00:00
#SBATCH -J okvis2x-texcup
#SBATCH -o okvis2x_slurm_%j.log

# Submit from the repository root, or export BENCH_ROOT. Use the activated
# OKVIS2-X build environment, or set BENCH_OKVIS_ENV for a Conda environment.
set -uo pipefail
ROOT="${BENCH_ROOT:-${SLURM_SUBMIT_DIR:-$PWD}}"
ROOT="$(cd "$ROOT" && pwd)" || exit 2
PROJ="$ROOT/projects/OKVIS2-X"
LATEST="$ROOT/results/OKVIS2-X/latest"
DATA_PYTHON="${BENCH_DATA_PYTHON:-$(command -v python3)}"
export TEXCUP_DATA="${TEXCUP_DATA:-$ROOT/data/tex_cup}"
JOB_ID="${SLURM_JOB_ID:-local}"
mkdir -p "$PROJ/results/run" "$LATEST/logs" || exit 2

status() {  # phase message -> results/OKVIS2-X/latest/status.json
  "$DATA_PYTHON" - "$1" "$2" "$JOB_ID" "$LATEST/status.json" <<'PY'
import json, sys, time
phase, msg, job, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
try:
    with open(out) as f:
        d = json.load(f)
except (OSError, ValueError):
    d = {"method": "OKVIS2-X", "app": "okvis_app_synchronous",
         "mode": "stereo VI + GNSS fusion", "job_id": job,
         "started_utc": time.strftime('%Y-%m-%dT%H:%M:%S+00:00', time.gmtime())}
d["phase"] = phase
d["message"] = msg
d["updated_utc"] = time.strftime('%Y-%m-%dT%H:%M:%S+00:00', time.gmtime())
with open(out, 'w') as f:
    json.dump(d, f, indent=2)
PY
}

images_ready() {
  "$DATA_PYTHON" - <<'PY'
import csv
from pathlib import Path
for camera in ("cam0", "cam1"):
    base = Path("data/texcup_euroc") / camera
    try:
        with (base / "data.csv").open() as f:
            rows = list(csv.reader(f))[1:]
        if not rows or any(len(row) < 2 or not (base / "data" / row[1]).is_file()
                           for row in rows):
            raise SystemExit(1)
    except OSError:
        raise SystemExit(1)
PY
}

cd "$PROJ" || exit 2
status preparing "checking native 2048x732 stereo PNGs"

# imu0/gps0 are prepared before submission; build missing camera PNGs here.
if ! images_ready; then
  if ! "$DATA_PYTHON" loaders/convert_texcup.py --out data/texcup_euroc \
      --skip-imu --skip-gps --workers 12; then
    status error "image conversion failed"
    exit 3
  fi
fi
if ! images_ready; then
  status error "image conversion incomplete"
  exit 3
fi

if [ -n "${BENCH_OKVIS_ENV:-}" ]; then
  BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
  if ! source "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh" || \
      ! conda activate "$BENCH_OKVIS_ENV"; then
    status error "could not activate BENCH_OKVIS_ENV"
    exit 2
  fi
fi

# Stereo VI + GNSS, including the configured final full-batch VI-BA.
status running "okvis_app_synchronous: sequence + final full-batch VI-BA"
t0=$(date +%s)
./upstream/build/okvis_app_synchronous config/okvis2_texcup.yaml data/texcup_euroc results/run \
    > "$LATEST/logs/okvis_app.log" 2>&1
rc=$?
t1=$(date +%s)
echo "APP_EXIT_CODE=$rc  RUNTIME_SEC=$((t1-t0))"

if [ -f "$ROOT/okvis2x_slurm_$JOB_ID.log" ]; then
  cp -f "$ROOT/okvis2x_slurm_$JOB_ID.log" "$LATEST/logs/"
fi
if [ "$rc" -eq 0 ] && [ -f results/run/okvis2-vio-global-final-ba_trajectory.csv ]; then
  status completed "exit 0; final-BA global trajectory written (runtime $((t1-t0)) s)"
else
  status error "app exit $rc or missing final-BA trajectory"
  if [ "$rc" -eq 0 ]; then rc=4; fi
fi
exit "$rc"
