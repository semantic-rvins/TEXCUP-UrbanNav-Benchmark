# Run InGVIO on TEX-CUP

The comparison reports the **upstream-default full-resolution replay**
(`gnss_chi2_test: 0`, `visual_noise: 0.18` — the shipped sportsfield values)
on the reduced dual-frequency GNSS; see [STATUS.md](STATUS.md). With that
default the filter diverges at the vehicle's first full stop (18:17:07 UTC), and
the resulting finite errors are retained. The estimator runs on CPU through ROS 1.

## 1. Prepare data and the ROS environment

Run commands from the benchmark root in Bash:

```bash
export BENCH="$PWD"
export BENCH_ROOT="$BENCH"
export TEXCUP_DATA="${TEXCUP_DATA:-$BENCH/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
source "$BENCH/projects/GVINS/env.sh"
```

Create the `benchmark_ros` environment using [GVINS steps 1–2](../GVINS/RUN.md)
if absent. Those steps also build the `texcup2bag` RINEX converter needed below;
a GVINS replay is not required. [Data setup](../../data/README.md) covers
downloading the publisher GNSS/IMU/ground-truth inputs, converting SBF to
observations, materializing the benchmark calibration profiles locally, and
downloading `camera_data/images.h5`. `BENCH_ROS_ENV` can override the ROS
environment path.

## 2. Clone, patch and build

For a fresh source/workspace directory:

```bash
git clone https://github.com/ChangwuLiu/InGVIO "$BENCH/projects/InGVIO/upstream"
git -C "$BENCH/projects/InGVIO/upstream" checkout --detach \
  "$(cat "$BENCH/projects/InGVIO/patches/UPSTREAM_COMMIT")"
git -C "$BENCH/projects/InGVIO/upstream" apply \
  "$BENCH/projects/InGVIO/patches/01-build-cxx17-eigen-feature_tracker-dep.patch"
git -C "$BENCH/projects/InGVIO/upstream" apply \
  "$BENCH/projects/InGVIO/patches/02-publish-ecef-odometry.patch"
mkdir -p "$BENCH/projects/InGVIO/catkin_ws/src"
for package in camera_model feature_tracker gnss_comm ingvio_estimator; do
  ln -s "$BENCH/projects/InGVIO/upstream/$package" \
    "$BENCH/projects/InGVIO/catkin_ws/src/$package"
done
catkin config --workspace "$BENCH/projects/InGVIO/catkin_ws" --init \
  --extend "$CONDA_PREFIX" --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DEigen3_DIR="$CONDA_PREFIX/share/eigen3/cmake" \
  -DEIGEN3_INCLUDE_DIR="$CONDA_PREFIX/include/eigen3" \
  -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
catkin build --workspace "$BENCH/projects/InGVIO/catkin_ws" -j8
```

The pin is `eca739e894c656a58625c63ef541c9fc31a77e3b`. The patches cover
build compatibility and an ECEF output publisher; see
[BUGFIX_NOTES.md](BUGFIX_NOTES.md).

## 3. Prepare input bags

Use the GVINS workspace for its converter in this shell:

```bash
source "$BENCH/projects/GVINS/catkin_ws/devel/setup.bash"
mkdir -p projects/InGVIO/bags
rosrun texcup2bag rinex2bag "$TEXCUP_DATA/brdm1290_v304.19p" \
  "$TEXCUP_DATA/asterx4_rover.obs" \
  projects/InGVIO/bags/gnss_180940.bag 1557425380.0 18
python projects/GVINS/loaders/make_rosbag.py --data-dir "$TEXCUP_DATA" \
  --gnss-bag projects/InGVIO/bags/gnss_180940.bag \
  --out projects/InGVIO/bags/texcup_ingvio.bag \
  --start-utc 1557425380.0 --end-utc 1557429420.0 --downscale 1
rosbag info projects/InGVIO/bags/texcup_ingvio.bag
```

The bag carries `/imu0`, native 2048 × 732 `/cam0/image_raw`, and the
`/ublox_driver/` GNSS topics. IMU axes are RFU, ROS timestamps are UTC and GNSS
observation payloads remain GPST. InGVIO uses `gnss_local_offset: -18.0`.
GLONASS is excluded. The image window starts at 18:09:40 UTC and includes a
one-second tail beyond the final evaluation epoch. Allow about 60 GB for the
merged bag. Use the checked-in camera and estimator YAMLs together.

## 4. Replay and inspect logs

In a fresh Bash shell, restore the exports from step 1. The wrapper activates
the ROS environment and InGVIO workspace itself:

```bash
INGVIO_RUN="$BENCH/results/InGVIO/run_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$INGVIO_RUN"
ln -sfn "$(basename "$INGVIO_RUN")" "$BENCH/results/InGVIO/latest"
bash projects/InGVIO/run_ingvio.sh 1.0 \
  "$BENCH/projects/InGVIO/bags/texcup_ingvio.bag" \
  "$INGVIO_RUN/ingvio_out.bag" > "$INGVIO_RUN/launcher.log" 2>&1
```

Alternatively, `sbatch projects/InGVIO/run_slurm.sbatch` uses the default bag
and writes `projects/InGVIO/results/ingvio_out.bag`; the subsequent extraction
must use that path. The wrapper's ROS port is **11953**. It writes node logs
under `projects/InGVIO/results/logs/` and the resolved estimator YAML under
`projects/InGVIO/results/ingvio_mono_texcup.yaml`. Only run one instance at a
time because these paths are shared.

The wrapper records `/ingvio_estimator/pose_ecef`, `/pose_w` and `/pose_spp`,
plays the bag and shuts down the nodes. It does **not** build bags, generate a
live `status.json`, extract a trajectory or run statistics. Check
`launcher.log`, the node log and `rosbag info` on the output before treating a
replay as complete; the wrapper's exit code alone does not validate the run.

## 5. Extract and collect

After a completed replay:

```bash
source "$BENCH/projects/GVINS/env.sh"
source "$BENCH/projects/InGVIO/catkin_ws/devel/setup.bash"
python projects/InGVIO/loaders/extract_est.py \
  --bag "$INGVIO_RUN/ingvio_out.bag" --out "$INGVIO_RUN/est.csv"
cp -r projects/InGVIO/results/logs "$INGVIO_RUN/logs"
cp projects/InGVIO/results/ingvio_mono_texcup.yaml "$INGVIO_RUN/"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" --collect "InGVIO=$INGVIO_RUN"
```

The extractor converts IMU ECEF poses to the ALT1 antenna using the estimated
body-to-ECEF rotation and RFU lever `[-0.610,-0.052,0.010] m`. Its timestamps
are UTC seconds of day. The common collector validates the trajectory;
invalid/non-monotonic output requires inspection rather than silently declaring
completion. For interrupted or failed runs, record their actual status and use
the explicit incomplete-collection procedure in [STATISTICS.md](../../STATISTICS.md).

The common evaluation window has 4,040 one-second epochs, 18:09:40–19:16:59
UTC inclusive. Availability and threshold percentages use all epochs; distance
statistics use matched solutions and retain finite outliers.
