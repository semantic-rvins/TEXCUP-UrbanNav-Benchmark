# Run GVINS on TEX-CUP

The full replay has completed and its results are in
[`results/final`](../../results/final/README.md). The trajectory contains large
outliers, which remain in the score. This recipe reproduces the saved setup,
including the active changes listed in [BUG_FIXES.md](../../BUG_FIXES.md).
GVINS uses native mono images, RFU IMU rates, and rover-only raw GNSS; it does
not use RTKLIB positions or a base station. It runs on CPU through ROS 1.

Run commands from the benchmark repository root in Bash. First export
`TEXCUP_DATA="$PWD/data/tex_cup"`, following the root run guide. The data
directory must contain `asterx4_rover.obs`, `brdm1290_v304.19p`, `lord_imu.log`,
`camera_data/images.h5`, and `ground_truth.log`. Calibration is supplied in the
checked-in method YAML; the source mono calibration is
`camera-calibration-mono/texcup_port-camchain.yaml`.

## 1. Install or activate the environment

```bash
export BENCH="$(pwd)"
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
"$BENCH_CONDA_ROOT/bin/conda" create -y -n benchmark_ros \
  --file "$BENCH/environments/benchmark_ros.explicit.txt"
source "$BENCH/projects/GVINS/env.sh"
mkdir -p "$BENCH/results/GVINS/logs"
conda list --explicit > "$BENCH/results/GVINS/environment_explicit.txt"
```

Create the environment once. The explicit package list targets Linux x86-64
and supplies ROS Noetic, Ceres 2.1, Eigen 3.4, compatible OpenCV, and empy 3.3.4.
No GPU or system ROS installation is needed. `BENCH_ROS_ENV` overrides the
environment directory. Use a fresh shell when switching method workspaces.

## 2. Clone, pin, patch, and build

These clone/link commands are for a fresh setup. Both source clones are ignored
by Git, so the saved patches must be applied after cloning.

```bash
git clone https://github.com/HKUST-Aerial-Robotics/GVINS.git \
  "$BENCH/projects/GVINS/upstream"
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git \
  "$BENCH/projects/GVINS/gnss_comm"
git -C "$BENCH/projects/GVINS/upstream" checkout --detach \
  "$(cat "$BENCH/projects/GVINS/patches/gvins_upstream.commit")"
git -C "$BENCH/projects/GVINS/upstream" apply \
  "$BENCH/projects/GVINS/patches/gvins_upstream.patch"
git -C "$BENCH/projects/GVINS/gnss_comm" checkout --detach \
  "$(cat "$BENCH/projects/GVINS/patches/gnss_comm.commit")"
git -C "$BENCH/projects/GVINS/gnss_comm" apply \
  "$BENCH/projects/GVINS/patches/gnss_comm.patch"
mkdir -p "$BENCH/projects/GVINS/catkin_ws/src"
ln -s ../../gnss_comm "$BENCH/projects/GVINS/catkin_ws/src/gnss_comm"
ln -s ../../upstream/camera_model "$BENCH/projects/GVINS/catkin_ws/src/gvins_camera_model"
ln -s ../../upstream/feature_tracker "$BENCH/projects/GVINS/catkin_ws/src/gvins_feature_tracker"
ln -s ../../upstream/estimator "$BENCH/projects/GVINS/catkin_ws/src/gvins"
ln -s ../../loaders/texcup2bag "$BENCH/projects/GVINS/catkin_ws/src/texcup2bag"
catkin config --workspace "$BENCH/projects/GVINS/catkin_ws" --init \
  --extend "$CONDA_PREFIX" --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DEigen3_DIR="$CONDA_PREFIX/share/eigen3/cmake" \
  -DEIGEN3_INCLUDE_DIR="$CONDA_PREFIX/include/eigen3" \
  -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
catkin build --workspace "$BENCH/projects/GVINS/catkin_ws" -j2 -p2 --no-status \
  > "$BENCH/results/GVINS/logs/build.log" 2>&1
source "$BENCH/projects/GVINS/catkin_ws/devel/setup.bash"
python -m unittest discover -s projects/GVINS/tests -v \
  > results/GVINS/logs/timing_tests.log 2>&1
```

The pinned revisions are GVINS `d2cf40b49c6eb0e6ad3caa4f613713983be3fd74`
and gnss_comm `a2035fec1cc427bba184d25ff1fde8dcf23c9ce1`.

**The timing correction requires rebuilt C++ as well as YAML.** It preserves
the GNSS/local offset through resets and uses elapsed seconds for clock
propagation, including observation-time residuals and Jacobians. Build fixes
cover C++17/OpenCV compatibility; gnss_comm fixes guard empty observation
vectors and truncated RINEX records.

The saved GVINS patch also includes behavior changes beyond timing/data format:

- GNSS-camera pairing tolerance is 0.15 s.
- GNSS alignment and yaw release require two frames with sufficient GNSS,
  instead of every frame in the window.
- Translation/height-jump reboot triggers are disabled; IMU-bias checks remain.
- Ephemerides survive estimator resets for the bag's initial ephemeris delivery.

These are part of the reproduced result and are not described as an unchanged
upstream algorithm. See [BUG_FIXES.md](../../BUG_FIXES.md) for their scope.

## 3. Prepare and verify the native input bag

```bash
mkdir -p projects/GVINS/bags
rosrun texcup2bag rinex2bag "$TEXCUP_DATA/brdm1290_v304.19p" \
  "$TEXCUP_DATA/asterx4_rover.obs" \
  projects/GVINS/bags/gnss_180940.bag 1557425380.0 18 \
  > results/GVINS/logs/gnss_conversion.log 2>&1
python -u projects/GVINS/loaders/make_rosbag.py --data-dir "$TEXCUP_DATA" \
  --gnss-bag projects/GVINS/bags/gnss_180940.bag \
  --out projects/GVINS/bags/texcup_gvins_full.bag \
  --start-utc 1557425380.0 --downscale 1 \
  > results/GVINS/logs/bag_conversion.log 2>&1
python projects/GVINS/loaders/verify_bag.py \
  --bag projects/GVINS/bags/texcup_gvins_full.bag \
  --gnss-bag projects/GVINS/bags/gnss_180940.bag --data-dir "$TEXCUP_DATA" \
  --out results/GVINS/input_validation.json \
  > results/GVINS/logs/input_validation.log 2>&1
```

Use new bag paths when rebuilding; retain the validated inputs for repeat runs.
The checked-in navigation file supplies the parser-compatible RINEX header and
ionosphere parameters. The converter retains GPS/Galileo/BeiDou and excludes
GLONASS. GNSS payload times stay GPST; bag records and image/IMU headers are UTC.
RINEX-derived pseudorange/Doppler standard deviations use the converter's
nominal values, 0.16 m and 0.256 Hz.

The merged bag has **40,372 native 2048 × 732 port images**, 450,561 IMU samples,
and 8,661 GNSS messages, including 4,557 observation epochs. It occupies about
54 GiB. The IMU/GNSS tail extends beyond the camera data; scoring is restricted
to the common window. Input verification checks camera/GNSS timestamp coverage,
payload time/system/frequency fields, sampled raw pixels, and rotated IMU values.

Use [gvins_texcup.yaml](config/gvins_texcup.yaml): native intrinsics/distortion,
`gnss_local_online_sync: 0`, and **`gnss_local_time_diff: 18.0`**. Acceleration and
gyro are RFU rates `(−raw_x, raw_y, −raw_z)`. Camera-to-RFU rotation is
`[[1,0,0],[0,0,1],[0,-1,0]]`, translation `[-0.303,0.011,0.033] m`.
Online camera extrinsic estimation remains enabled. The launcher creates an
isolated runtime YAML and supplies its paths to both ROS nodes.

## 4. Start a detached replay and monitor it

```bash
GVINS_RUN="$BENCH/results/GVINS/run_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$GVINS_RUN"
ln -sfn "$(basename "$GVINS_RUN")" "$BENCH/results/GVINS/latest"
nohup setsid bash projects/GVINS/run_gvins.sh 1.0 "$GVINS_RUN" \
  projects/GVINS/bags/texcup_gvins_full.bag 11951 \
  > "$GVINS_RUN/launcher.log" 2>&1 < /dev/null &
```

Use a fresh output directory and an unoccupied ROS port. The wrapper activates
the environment/workspace and caps CPU thread pools. At 1×, playback takes
about 76 minutes plus startup, queue draining, extraction, and evaluation.

```bash
watch -n 5 cat results/GVINS/latest/status.json
tail -F results/GVINS/latest/logs/progress.log \
  results/GVINS/latest/logs/errors.log results/GVINS/latest/logs/warnings.log
```

Status is replaced atomically every five seconds. Reopen it with `watch`;
`tail -f status.json` can remain attached to an old version. Feature, pose, and
fused-output counts and their timestamps show progress. Native Ceres/glog
output remains in `logs/gvins.log`. Errors and resets describe runtime behavior;
an empty error log does not establish accurate positions.

To stop a run:

```bash
kill -TERM "$(cat results/GVINS/latest/supervisor.pid)"
```

## 5. Complete evaluation and collect the shared statistics

After playback, the supervisor drains output, closes the recorded bag,
extracts the trajectory, checks finite/time-ordered values, and evaluates
18:09:40–19:16:59 UTC inclusive. Successful completion is `phase: completed`
with message `Replay, extraction and evaluation completed; review eval.json and logs`.
`error` and `stopped` are separate terminal states.

The extractor converts georeferenced body poses to antenna ECEF using the
RFU IMU-to-ALT1 lever `[-0.610,-0.052,0.010] m`. Recovering body attitude from
the exported camera pose uses initial camera extrinsics, an approximation
when online extrinsics change. The GT already represents ALT1 and is not shifted.

**Run this collection step after successful completion.** The GVINS supervisor
does not automatically publish to the shared final folder.

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect GVINS=results/GVINS/latest
```

This produces `results/final/GVINS.*` and updates the shared statistics table.
All **4,040 epochs** remain in availability and horizontal threshold percentage
denominators. RMSE, maximum, and P95 use solved epochs; missing estimates are
unavailable and finite outliers are retained. The current completed result
contains large position outliers; its statistics are not an accuracy guarantee.
