# Run GICI-LIB on TEX-CUP

Run commands from the benchmark root in Bash after setting `BENCH`,
`TEXCUP_DATA` and `BENCH_CONDA_ROOT` in [run setup](../../RUN_INSTRUCTIONS.md).
Both variants have completed; see [final statistics](../../results/final/statistics.md).
GICI runs standalone on CPU without ROS. GICI-RTK uses GNSS only;
GICI-RRR additionally processes LORD IMU samples and raw port-camera images.

## Shared environment and pinned source

Create the environment once and clone into an absent target directory:

```bash
source "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh"
conda create -y -n benchmark_gici --file environments/benchmark_gici.explicit.txt
conda activate benchmark_gici
export PYTHONNOUSERSITE=1
export CPATH="$CONDA_PREFIX/include"
git clone https://github.com/chichengcn/gici-open.git projects/GICI-LIB/upstream
git -C projects/GICI-LIB/upstream checkout --detach \
  f2b8579f4fab9dff2b0950b11cc04aefe6d48e3a
```

The pinned environment includes GCC12, Ceres2.1, Eigen3.4, OpenCV4.13 and
SuiteSparse5.10. `CPATH` supplies headers to upstream third-party targets.
Use unpatched `upstream/` for RTK and `upstream-rrr/` with its separate patch
set for RRR. Do not apply RRR behavior patches to RTK.

## GNSS-only RTK

### 1. Build

```bash
mkdir -p results/GICI-RTK/logs
cmake -S projects/GICI-LIB/upstream -B projects/GICI-LIB/upstream/build-benchmark \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_C_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc" \
  -DCMAKE_CXX_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++" \
  > results/GICI-RTK/logs/configure.log 2>&1
cmake --build projects/GICI-LIB/upstream/build-benchmark \
  --target gici_main --parallel 4 > results/GICI-RTK/logs/build.log 2>&1
conda list --explicit > results/GICI-RTK/environment_explicit.txt
python -m unittest discover -s projects/GICI-LIB/tests -v
```

### 2. Prepare and run in the background

```bash
GICI_RUN="$BENCH/results/GICI-RTK/run_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$GICI_RUN"
ln -sfn "$(basename "$GICI_RUN")" results/GICI-RTK/latest
nohup setsid python -u projects/GICI-LIB/run_rtk.py \
  --out "$GICI_RUN" --data-dir "$TEXCUP_DATA" \
  > "$GICI_RUN/launcher.log" 2>&1 < /dev/null &
echo $! > "$GICI_RUN/supervisor.pid"
```

The runner cuts rover/base observations from **18:09:58 GPST**, resolves the
[RTK config](config/gici_rtk_dualfreq_post.yaml) and processes through EOF.
GLONASS is excluded from measurements; GPS1C/2W, Galileo1C/7Q and BeiDou2I
remain. No IMU, camera or lever correction is used. Source is unpatched.

### 3. Check completion and collect

```bash
watch -n 300 cat results/GICI-RTK/latest/status.json
# In a separate terminal:
tail -F results/GICI-RTK/latest/logs/gici.log
```

After `phase: completed` and supervisor exit:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect GICI-RTK=results/GICI-RTK/latest
```

The runner writes NMEA, antenna `est.csv`, evaluation and local statistics.
At EOF, after input descriptors reach their ends and output is unchanged for
30 seconds, it deliberately terminates GICI. Native return code `-15` with
this recorded EOF shutdown is expected. Other errors remain failures.

## RTK IMU camera RRR

### 1. Apply the saved patches in a separate worktree and build

Use the shared source/environment setup above, then:

```bash
git -C projects/GICI-LIB/upstream worktree add --detach ../upstream-rrr \
  f2b8579f4fab9dff2b0950b11cc04aefe6d48e3a
for patch in 0001-texcup-robustness-guards.patch 0002-record-origin-and-eof.patch; do
  git -C projects/GICI-LIB/upstream-rrr apply "../patches/$patch"
done
mkdir -p results/GICI-RRR/logs
cmake -S projects/GICI-LIB/upstream-rrr -B projects/GICI-LIB/upstream-rrr/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_C_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc" \
  -DCMAKE_CXX_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++" \
  > results/GICI-RRR/logs/configure.log 2>&1
cmake --build projects/GICI-LIB/upstream-rrr/build --target gici_main --parallel 4 \
  > results/GICI-RRR/logs/build.log 2>&1
conda list --explicit > results/GICI-RRR/environment_explicit.txt
```

Patch0001 adds missing-state/history and visual guards. Its fewer-than-eight
track fallback bypasses RANSAC and changes feature admission; this result is
**not an untouched upstream baseline**. Patch0002 logs the actual ENU origin
and input EOF. See [patch scope](../../GICI_PATCH_REVIEW.md).

### 2. Generate and validate native inputs

The image converter needs h5py/NumPy. These commands use Python from the
`benchmark_ros` environment in the root setup while leaving GICI activated:

```bash
DATA_PYTHON="${BENCH_ROS_ENV:-$BENCH_CONDA_ROOT/envs/benchmark_ros}/bin/python"
mkdir -p projects/GICI-LIB/data_converted
python projects/GICI-LIB/loaders/convert_imu.py \
  --log "$TEXCUP_DATA/lord_imu.log" \
  --out projects/GICI-LIB/data_converted/imu_rfu_180940.txt
"$DATA_PYTHON" projects/GICI-LIB/loaders/convert_images.py \
  --h5 "$TEXCUP_DATA/camera_data/images.h5" \
  --out projects/GICI-LIB/data_converted/port_images_180940_full.pack
"$DATA_PYTHON" projects/GICI-LIB/loaders/verify_rrr_inputs.py \
  --data-dir "$TEXCUP_DATA" \
  --imu projects/GICI-LIB/data_converted/imu_rfu_180940.txt \
  --images projects/GICI-LIB/data_converted/port_images_180940_full.pack \
  --out results/GICI-RRR/preparation.json \
  > results/GICI-RRR/logs/input_validation.log 2>&1
python -m unittest discover -s projects/GICI-LIB/tests -v
```

The image pack contains 40,372 raw **2048×732** port frames (about60.5GB).
Both inputs start at UTC1557425380. IMU rates rotate to RFU, with GPST converted
to UTC once. Image timestamps already use UTC. Validation checks IMU data,
image headers/timestamps and sample pixels against the source. Regenerate the
validation record whenever converted inputs change.

### 3. Launch and monitor

```bash
RRR_RUN="$BENCH/results/GICI-RRR/run_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RRR_RUN"
ln -sfn "$(basename "$RRR_RUN")" results/GICI-RRR/latest
nohup setsid python -u projects/GICI-LIB/run_rrr.py \
  --out "$RRR_RUN" --data-dir "$TEXCUP_DATA" \
  --final-results "$BENCH/results/final" \
  > "$RRR_RUN/launcher.log" 2>&1 < /dev/null &
echo $! > "$RRR_RUN/supervisor.pid"
```

The runner uses [the RRR config](config/gici_rrr_mono_dualfreq_post.yaml),
bounds rover/base data to18:09:40–19:16:59 UTC and IMU through19:17:00 UTC
for final-interval support. It retains native image timestamps and full images.
The fixed RFU antenna lever is `[-0.610,-0.052,0.010]`m; camera intrinsics and
extrinsics are in [the calibration guide](../../METHOD_SETUP_GUIDE.md).

```bash
watch -n 300 cat results/GICI-RRR/latest/status.json
# In a separate terminal:
tail -F results/GICI-RRR/latest/logs/warnings.log \
  results/GICI-RRR/latest/logs/errors.log
```

`completed` means replay, conversion, verification and final statistics have
finished. `error`/`stopped` are failures. Logged EOF plus30seconds of idle output
triggers the deliberate SIGTERM described above. RRR runs as fast as its CPU
processing permits, rather than waiting for wall-clock camera intervals.

### 4. Verify and recalculate

```bash
python projects/GICI-LIB/loaders/verify_rrr_output.py \
  --run "$BENCH/results/GICI-RRR/latest"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect GICI-RRR=results/GICI-RRR/latest
```

The runner records GICI's initial SPP ENU origin and rotates the antenna lever
with its full estimated ESA attitude in that fixed frame. GGA body position is
translated once; GGA altitude plus geoid separation gives ellipsoidal height.
Output verification checks the origin, NMEA checksums, GGA/ESA pairing and every
lever translation. Finite outliers and reset-related gaps remain in statistics.
