# Run IC-GVINS on TEX-CUP

Saved results are in [`results/final`](../../results/final/README.md); their
`IC-GVINS.source.json` records the exact patch set used. Changes to the patch set require a
fresh replay before judging their effect on accuracy. Completion establishes
output coverage; it does not establish estimator accuracy.
This recipe preserves upstream initialization and GNSS weighting, with the
documented [software fixes](../../BUG_FIXES.md#ic-gvins-validate-time-windows-before-indexing).
Run commands from the benchmark repository root in Bash. The pipeline uses
CPU and ROS 1; no GPU is required.

The current comparison reports the **pre-divergence prefix through 18:34:59 UTC**,
with **1,502 / 4,040 epochs (37.18% availability)**. The full source replay
later crashed with an unresolved SIGSEGV; its output is retained. The commands
below reproduce the full input preparation and replay. The saved comparison
selection is described in step 5.

## 1. Set paths and install the environment

First export `TEXCUP_DATA="$PWD/data/tex_cup"`, as in the root run guide.

Run [RTKLIB](../RTKLIB/RUN.md) first: IC-GVINS input conversion needs
`results/RTKLIB/rtklib_demo5.pos`. The TEX-CUP directory must contain
`lord_imu.log`, `camera_data/images.h5`, its camera calibration files, and
`ground_truth.log`.

```bash
export BENCH="$(pwd)"
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
"$BENCH_CONDA_ROOT/bin/conda" create -y -n benchmark_icgvins \
  --file "$BENCH/environments/benchmark_icgvins.explicit.txt"
source "$BENCH/projects/IC-GVINS/env.sh"
mkdir -p "$BENCH/results/IC-GVINS/logs" \
  "$BENCH/results/IC-GVINS/baseline_preparation"
conda list --explicit > "$BENCH/results/IC-GVINS/environment_explicit.txt"
```

Create the environment once. The explicit Linux x86-64 package list includes
yaml-cpp 0.8 and matching TBB runtime/development packages. Keep this separate
from `benchmark_ros`. `BENCH_IC_ENV` overrides the environment directory.
Use a fresh shell when switching method environments/workspaces.

## 2. Clone pinned source, apply patches, and build

These clone/link commands are for a fresh setup. The ignored source clone does
not preserve benchmark patches; apply the six listed here in order.

```bash
git clone https://github.com/i2Nav-WHU/IC-GVINS.git \
  "$BENCH/projects/IC-GVINS/upstream"
git -C "$BENCH/projects/IC-GVINS/upstream" checkout --detach \
  "$(cat "$BENCH/projects/IC-GVINS/patches/icgvins_upstream.commit")"
for patch in 03-runtime-diagnostics-and-shutdown.patch \
             04-time-window-bounds.patch \
             05-preserve-gnss-supported-states.patch \
             06-remap-prior-state-pointers.patch \
             07-feature-grid-bounds.patch \
             08-materialize-feature-velocity.patch; do
  git -C "$BENCH/projects/IC-GVINS/upstream" apply \
    "$BENCH/projects/IC-GVINS/patches/$patch"
done
python projects/IC-GVINS/loaders/verify_source.py
mkdir -p "$BENCH/projects/IC-GVINS/catkin_ws/src"
ln -s ../../upstream/ic_gvins "$BENCH/projects/IC-GVINS/catkin_ws/src/ic_gvins"
catkin config --workspace "$BENCH/projects/IC-GVINS/catkin_ws" --init \
  --extend "$CONDA_PREFIX" --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DEigen3_DIR="$CONDA_PREFIX/share/eigen3/cmake" \
  -DEIGEN3_INCLUDE_DIR="$CONDA_PREFIX/include/eigen3"
catkin build --workspace "$BENCH/projects/IC-GVINS/catkin_ws" \
  ic_gvins -j2 --no-status \
  > "$BENCH/results/IC-GVINS/baseline_preparation/build.log" 2>&1
source "$BENCH/projects/IC-GVINS/catkin_ws/devel/setup.bash"
python projects/IC-GVINS/tests/test_time_bounds.py \
  > results/IC-GVINS/baseline_preparation/bounds_tests.log 2>&1
python projects/IC-GVINS/tests/test_gnss_state_retention.py \
  > results/IC-GVINS/baseline_preparation/gnss_state_tests.log 2>&1
python projects/IC-GVINS/tests/test_prior_state_pointers.py \
  > results/IC-GVINS/baseline_preparation/prior_pointer_tests.log 2>&1
python projects/IC-GVINS/tests/test_feature_grid_bounds.py \
  > results/IC-GVINS/baseline_preparation/feature_grid_tests.log 2>&1
python projects/IC-GVINS/tests/test_feature_velocity_lifetime.py \
  > results/IC-GVINS/baseline_preparation/feature_velocity_tests.log 2>&1
python -m unittest discover -s projects/IC-GVINS/tests -p test_extraction.py -v \
  > results/IC-GVINS/baseline_preparation/extraction_tests.log 2>&1
```

The pinned revision is `644eed9e02c1239d6788f5b19123559735ca1b9e`.
Patch 03 adds runtime counters and joins the shutdown-monitor thread. Patch 04
validates IMU interval endpoints and signed state indices, and creates keyframes
only when their state node is valid. Unsupported insertions are logged.
Patch 05 retains a state while GNSS observations still reference it, even if
its visual keyframe is removed. Normal marginalization retires it later.
See the [state-lifetime bug explanation](../../BUG_FIXES.md#ic-gvins-preserve-states-still-used-by-gnss).
Patch 06 preserves states referenced by the marginalization prior and remaps
its pointers when deque storage moves. Patch 07 bounds feature-grid indices,
including image-edge remainders and undistorted points outside the image.
Patch 08 materializes the Eigen feature-velocity expression before its temporary
inputs expire. The [memory-safety explanation](../../BUG_FIXES.md#ic-gvins-state-pointers-and-feature-tracking-memory-safety)
describes the causes and regression checks.

The source verifier compares the checkout byte for byte with the pinned source plus the
active patch sequence; it also runs automatically before every replay.

Upstream stationary thresholds remain **0.002 rad/s and 0.1 m/s²**. GNSS
chi-square uncertainty scaling and the second optimization without a robust
loss are retained. Apply only the six patches listed above. The bounds fix prevents invalid indexing;
it adds no estimator reset, factor removal, or accuracy tuning. The runtime
manifest records the executable and actual `libic_gvins_core.so` build artifact.

## 3. Prepare and validate native inputs

```bash
mkdir -p projects/IC-GVINS/bags
python -u projects/IC-GVINS/loaders/make_icgvins_bag.py \
  --data-dir "$TEXCUP_DATA" --rtklib-pos results/RTKLIB/rtklib_demo5.pos \
  --start-utc 1557425380 --gnss-end-utc 1557429419 --end-utc 1557429420 \
  --downscale 1 --out projects/IC-GVINS/bags/texcup_icgvins_native.bag \
  > results/IC-GVINS/logs/build_bag.log 2>&1
python projects/IC-GVINS/loaders/verify_bag.py \
  --bag projects/IC-GVINS/bags/texcup_icgvins_native.bag \
  --out results/IC-GVINS/input_validation.json \
  > results/IC-GVINS/logs/input_validation.log 2>&1
```

The builder refuses to overwrite an existing bag. Reuse a validated bag with
its preparation JSON, or choose a fresh path when regeneration is needed.
The input contains **40,372 native 2048 × 732 port images**, 404,495 IMU samples,
and 4,014 GNSS fixes. The Q1/Q2 filter accepts 795 fixed and 3,219 float epochs;
26 Q4 DGPS epochs remain unavailable to this method's GNSS input.

Acceleration and gyro are **rates**, converted from raw LORD to
forward/right/down as `(raw_y, −raw_x, raw_z)`. The node forms increments
internally. Source GPST timestamps lose 18 seconds once to become ROS UTC;
camera timestamps are already UTC. IC-GVINS converts UTC to GPST seconds of
week internally. The five-second IMU prebuffer and final IMU tail support
initialization and the final images.

GNSS positions remain at the antenna. IC-GVINS models FRD IMU-to-ALT1 lever
`[-0.052,-0.610,-0.010] m` internally. NavSatFix covariance diagonals contain
**variances**, using fixed horizontal/vertical sigma 0.05/0.10 m and float
0.50/1.00 m. No antenna-to-body shift is applied to input fixes.

Use [icgvins_texcup.yaml](config/icgvins_texcup.yaml). Its camera-to-FRD
quaternion is xyzw `[0.5,0.5,0.5,0.5]`, translation `[0.011,-0.303,-0.033] m`.
Full-resolution mono intrinsics/distortion, IMU noise, and tracking/optimizer
settings remain configured as recorded. Online extrinsic and time-delay
estimation are enabled. The input verifier checks timestamp coverage, every
GPS position/variance, raw image/IMU samples, and calibration.

## 4. Start a detached replay and monitor it

```bash
IC_RUN="$BENCH/results/IC-GVINS/run_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$IC_RUN"
ln -sfn "$(basename "$IC_RUN")" "$BENCH/results/IC-GVINS/latest"
nohup setsid bash projects/IC-GVINS/run_icgvins.sh 1.0 "$IC_RUN" \
  projects/IC-GVINS/bags/texcup_icgvins_native.bag 11961 \
  > "$IC_RUN/launcher.log" 2>&1 < /dev/null &
```

Use a fresh output directory and an unoccupied port. Port **11961** is separate
from VINS-Fusion's **11952**. The launcher creates an isolated runtime YAML,
caps thread pools, and pins the IC node to the last four allowed logical CPUs.
At 1×, playback takes about 67 minutes plus startup, draining, and scoring.
Concurrent replays can share a machine; their elapsed time is not an isolated
speed measurement.

```bash
watch -n 300 cat results/IC-GVINS/latest/status.json
tail -F results/IC-GVINS/latest/logs/errors.log \
  results/IC-GVINS/latest/logs/warnings.log
```

Status is replaced atomically every five seconds. `images_received`,
`images_initializing`, `images_queued`, and `images_tracked` distinguish image
delivery from actual tracking. `keyframes_skipped`, `tracking_lost_count`,
`nav_count`, and timestamps show estimator progress. `gnss_states_retained`
counts states protected by patch 05; `state_lookup_failure_count` counts
failed state lookups. Native output is in
`logs/icgvins.log`. Error counts include nonfatal calibration rejections and
tracking-loss events; terminal `phase` and child exit codes establish whether
the run stopped unexpectedly.

Stop a run by signaling its supervisor:

```bash
kill -TERM "$(cat results/IC-GVINS/latest/supervisor.pid)"
```

## 5. Completion and statistics

The supervisor drains input, requests graceful shutdown, and requires a zero
estimator exit code and complete shutdown markers. Verification accounts for
every input image, checks every queued image was tracked, and requires
navigation through the final IMU samples. Initialization exclusions remain
explicitly counted.

`gvins_output/gvins.nav` contains IMU geodetic position and FRD-to-local-NED
attitude. Extraction rotates the ALT1 lever with this estimated attitude,
converts it through the current NED-to-ECEF basis, and adds it once to IMU
ECEF. GPST output is converted to UTC. The GT already represents ALT1 and is
not transformed.

Successful completion requires `phase: completed`, message
`Replay and statistics completed; collected into results/final`, and
`results/final/IC-GVINS.source.json`. Completed output can still have large
position errors; no outliers are removed to improve its score.

To ensure statistics are also collected if the rerun fails, start the existing
monitor after `status.json` appears. It checks every five minutes:

```bash
nohup setsid bash common/monitor_runs.sh --run "IC-GVINS=$IC_RUN" \
  --results results/final --state-dir "$IC_RUN/finalization_monitor" \
  --poll-seconds 300 > "$IC_RUN/finalization_monitor.log" 2>&1 < /dev/null &
```

To recompute the shared table for a completed run:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect IC-GVINS=results/IC-GVINS/latest
```

For a stopped or failed run, wait until all its processes have exited, then
preserve its failure status while scoring any usable output:

```bash
python common/finalize_runs.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --run IC-GVINS=results/IC-GVINS/latest
```

The common grid has **4,040 epochs**, 18:09:40–19:16:59 UTC inclusive.
Availability and horizontal threshold percentages use all epochs; missing
epochs fail thresholds. RMSE, maximum, and P95 use solved epochs. Finite
outliers are retained, and an empty trajectory has undefined distance metrics.
All trajectory copies, errors, statistics, and the combined table are in
[`results/final`](../../results/final/README.md).

### Recalculate the saved partial comparison

For the existing saved results, run without collecting a new trajectory:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
```

[table_selections.json](../../results/final/table_selections.json) selects
IC-GVINS timestamps before **18:35:00 UTC** exclusively. Availability and
threshold percentages still use the full **4,040-epoch** evaluation grid;
distance metrics use its **1,502** matched prefix epochs. Finite prefix
outliers remain included. This is a retrospective pre-divergence result,
not a successful full-route replay.

Before collecting a new IC-GVINS run, review or remove its selection entry
explicitly so the earlier run's cutoff is not reused unintentionally.
The original full trajectory, error grid, statistics and source failure status
remain in the per-method files under `results/final`.
