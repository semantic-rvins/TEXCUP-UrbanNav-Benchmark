# Run VINS-Fusion on TEX-CUP

The full replay has completed and its results are in
[`results/final`](../../results/final/README.md). This recipe preserves upstream
frame admission and optimizer settings, with the documented
[software fixes](../../BUG_FIXES.md#vins-fusion-keep-global-rotations-normalized).
Run the commands from the benchmark repository root in a Bash shell. No GPU is
required; this uses ROS 1 in an isolated Conda environment.

## 1. Set paths and install the environment

First export `TEXCUP_DATA="$PWD/data/tex_cup"`, as in the root run guide.

Run [RTKLIB](../RTKLIB/RUN.md) first: the input adapter needs its
`results/RTKLIB/est.csv`. The TEX-CUP directory must contain `lord_imu.log`,
`camera_data/images.h5`, `camera-calibration-stereo/`, and `ground_truth.log`.

```bash
export BENCH="$(pwd)"
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
"$BENCH_CONDA_ROOT/bin/conda" create -y -n benchmark_ros \
  --file "$BENCH/environments/benchmark_ros.explicit.txt"
source "$BENCH/projects/GVINS/env.sh"
mkdir -p "$BENCH/results/VINS-Fusion/logs" \
  "$BENCH/results/VINS-Fusion/baseline_preparation"
conda list --explicit > "$BENCH/results/VINS-Fusion/environment_explicit.txt"
```

Create the environment once. The explicit package list targets Linux x86-64
and includes the required image-transport package. `BENCH_ROS_ENV` overrides the
environment directory if needed. Reopen a fresh shell before switching to
another method's environment/workspace.

## 2. Clone pinned source, apply patches, and build

These clone/link commands are for a fresh setup. The ignored upstream clone
does not preserve benchmark changes; apply only the active patch series.

```bash
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git \
  "$BENCH/projects/VINS-Fusion/upstream"
git -C "$BENCH/projects/VINS-Fusion/upstream" checkout --detach \
  "$(cat "$BENCH/projects/VINS-Fusion/patches/vins_fusion_upstream.commit")"
while IFS= read -r patch; do
  git -C "$BENCH/projects/VINS-Fusion/upstream" apply \
    "$BENCH/projects/VINS-Fusion/patches/$patch"
done < "$BENCH/projects/VINS-Fusion/patches/series"
mkdir -p "$BENCH/projects/VINS-Fusion/catkin_ws/src"
for package in camera_models vins_estimator global_fusion; do
  ln -s "../../upstream/$package" \
    "$BENCH/projects/VINS-Fusion/catkin_ws/src/$package"
done
catkin config --workspace "$BENCH/projects/VINS-Fusion/catkin_ws" --init \
  --extend "$CONDA_PREFIX" --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DEigen3_DIR="$CONDA_PREFIX/share/eigen3/cmake" \
  -DEIGEN3_INCLUDE_DIR="$CONDA_PREFIX/include/eigen3"
catkin build --workspace "$BENCH/projects/VINS-Fusion/catkin_ws" \
  camera_models vins global_fusion -j2 --no-status \
  > "$BENCH/results/VINS-Fusion/baseline_preparation/build.log" 2>&1
source "$BENCH/projects/VINS-Fusion/catkin_ws/devel/setup.bash"
python projects/VINS-Fusion/tests/verify_patch_series.py \
  --out results/VINS-Fusion/baseline_preparation/patch_verification.json
python projects/VINS-Fusion/tests/test_global_rotation.py \
  > results/VINS-Fusion/baseline_preparation/rotation_regression.log 2>&1
python -m unittest discover -s projects/VINS-Fusion/tests -p test_extraction.py -v \
  > results/VINS-Fusion/baseline_preparation/extraction_tests.log 2>&1
```

The pinned revision is `be55a937a57436548ddfb1bd324bc1e9a9e828e0`.
The active patches, in order, are:

1. `0001-build-and-origin.patch`: C++17/OpenCV compatibility and ENU-origin logging.
2. `0002-runtime-path-and-diagnostics.patch`: output-path parameter and tracking/GPS telemetry.
3. `0003-normalize-global-rotations.patch`: normalized rotations, rigid global alignment, and unusable-update rejection.

Global fusion retains five iterations and variable poses throughout its graph.
Upstream tracks every stereo pair but admits every second pair to estimation.
Only the patches listed in `patches/series` are part of this build. `loop_fusion` is not
built or run. Source reproduction and rotation tests check implementation
invariants; they do not establish position accuracy.

## 3. Prepare and validate native inputs

```bash
mkdir -p projects/VINS-Fusion/bags
python -u projects/VINS-Fusion/loaders/make_rosbag_vins.py \
  --data-dir "$TEXCUP_DATA" --rtklib results/RTKLIB/est.csv \
  --downscale 1 --start-utc 1557425380 --end-utc 1557429420 \
  --out projects/VINS-Fusion/bags/texcup_stereo_native.bag \
  > results/VINS-Fusion/logs/build_bag.log 2>&1
python projects/VINS-Fusion/loaders/verify_bag.py \
  --bag projects/VINS-Fusion/bags/texcup_stereo_native.bag \
  --out results/VINS-Fusion/input_validation.json \
  > results/VINS-Fusion/logs/input_validation.log 2>&1
python projects/VINS-Fusion/loaders/make_gps_sidecar.py \
  --bag projects/VINS-Fusion/bags/texcup_stereo_native.bag \
  --rtklib results/RTKLIB/est.csv \
  --out projects/VINS-Fusion/bags/texcup_gps_upstream.bag
```

Builders refuse to overwrite an existing bag. A validated native bag can be
reused with its preparation JSON; rebuilding requires about 108 GiB of output
space. The sidecar is about 216 KB and can be prepared without rewriting images.
Keep the native bag and its preparation/validation records together.

The input contains 40,372 port/star pairs at **2048 × 732** and 404,495 IMU
samples, including a five-second IMU prebuffer. Acceleration and gyro are rates
in RFU `(−raw_x, raw_y, −raw_z)`. Timestamps are UTC; GPST source times lose
18 seconds exactly once. Both images use the port timestamp of their pair.

The small sidecar contains 4,040 RTKLIB positions interpolated onto the
20,186 estimator-admitted image timestamps, satisfying upstream's ±10 ms GPS
association. One player reads both bags in time order, excludes the native
bag's `/gps`, and remaps sidecar `/gps_baseline` to `/gps`.

Use the checked-in stereo and camera YAMLs. They preserve full-resolution
intrinsics, distortion, measured stereo geometry, and camera-to-RFU transform
`R = [[1,0,0],[0,0,1],[0,-1,0]]`, `t = [-0.303,0.011,0.033] m`. Online
extrinsic estimation remains enabled; IMU noise and VIO solver settings are
unchanged. See [README.md](README.md) for the remaining input conventions.

Global fusion has no antenna-lever model. GPS preparation approximates body
position by subtracting the RFU ALT1 lever `[-0.610,-0.052,0.010] m`, rotated
using RTKLIB course and zero roll/pitch. Low-speed headings use nearby valid
course estimates. Its covariance diagonal contains **sigma**, 0.05 m for Q1
and 0.75 m for other qualities, because this upstream factor divides directly
by the field. This is the consumer's convention, not NavSatFix variance semantics.

## 4. Start a detached replay and monitor it

```bash
VINS_RUN="$BENCH/results/VINS-Fusion/run_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$VINS_RUN"
ln -sfn "$(basename "$VINS_RUN")" "$BENCH/results/VINS-Fusion/latest"
nohup setsid bash projects/VINS-Fusion/run_vins.sh \
  projects/VINS-Fusion/bags/texcup_stereo_native.bag 1.0 "$VINS_RUN" 11952 \
  > "$VINS_RUN/launcher.log" 2>&1 < /dev/null &
```

Use a fresh output directory and an unoccupied ROS port. The launcher activates
the environment/workspace, copies runtime YAMLs, validates source and input
provenance, and caps CPU thread pools. At 1×, playback takes about 67 minutes
plus startup, queue draining, and scoring. For a separate startup check, append
`--duration 120` after the four arguments; this is never collected as a final run.

```bash
watch -n 5 cat results/VINS-Fusion/latest/status.json
tail -F results/VINS-Fusion/latest/logs/errors.log \
  results/VINS-Fusion/latest/logs/warnings.log
```

Status is replaced atomically every five seconds; use `watch` to reopen it.
`image_pairs_processed` counts tracking, while `vio_count` and `global_count`
follow upstream's half-rate estimation after initialization. `gps_accepted`,
last timestamps, and lag show progress. `invalid_rotation_count` must stay zero.
Full native logs are `logs/vins.log` and `logs/global_fusion.log`.

Stop a run by signaling its supervisor:

```bash
kill -TERM "$(cat results/VINS-Fusion/latest/supervisor.pid)"
```

## 5. Completion and statistics

The supervisor drains processing, checks every image pair and admitted output
timestamp, converts body poses to ALT1 antenna ECEF, and computes statistics.
Extraction uses estimated attitude and the actual logged fixed ENU origin;
the GT already represents ALT1 and is not transformed. Output before the first
completed global alignment is excluded while its epochs remain in the denominator.

Successful completion requires `phase: completed`, message
`Replay and statistics completed; collected into results/final`, and
`results/final/VINS-Fusion.source.json`. A deliberate SIGTERM of `vins_node`
after queue draining is its documented shutdown path; an unexpected exit during
playback sets `phase: error`.

To recompute the shared table for a completed run:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect VINS-Fusion=results/VINS-Fusion/latest
```

For a stopped or failed run, wait until all its processes have exited, then
preserve its failure status while scoring any usable output:

```bash
python common/finalize_runs.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --run VINS-Fusion=results/VINS-Fusion/latest
```

The common grid has **4,040 epochs**, 18:09:40–19:16:59 UTC inclusive.
Availability and horizontal threshold percentages use all epochs. RMSE,
maximum, and P95 use solved epochs; finite outliers are retained. Missing
output is unavailable, and an empty trajectory has undefined distance metrics.
Trajectory copies, errors, per-method statistics, and the combined table are
stored in [`results/final`](../../results/final/README.md).
