# Run the benchmark

Run commands in **Bash**, beginning at the repository root. Each linked method
guide includes public source checkout, pinned revision, patch application,
build, input validation, background launch and completion checks.

## 1. Set paths and obtain the data

```bash
export BENCH="$PWD"
export TEXCUP_DATA="${TEXCUP_DATA:-$BENCH/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
git lfs install
# The retained LFS data inputs are independent broadcast-navigation products.
git lfs pull --include="data/tex_cup/*.19p"
mkdir -p "$TEXCUP_DATA"
if [ "$TEXCUP_DATA" != "$BENCH/data/tex_cup" ]; then
  cp -n "$BENCH/data/tex_cup/brdm1290.19p" "$TEXCUP_DATA/"
  cp -n "$BENCH/data/tex_cup/brdm1290_v304.19p" "$TEXCUP_DATA/"
fi
python3 data/prepare_auxiliary.py --data-dir "$TEXCUP_DATA"
```

The [data guide](data/README.md) links the original
[TEX-CUP archive](https://rnl-data.ae.utexas.edu/texcup/). Raw/reduced observation
files, LORD IMU, ground truth and images are downloaded or generated locally
and ignored by Git. The auxiliary helper downloads LORD/GT, normalizes GT
comment encoding, and copies benchmark calibration configurations to the
legacy paths used by input loaders. It validates downloads automatically and
preserves existing files. Ground truth is not copied into `results/final`.

Prepare the GNSS observations using the tested [RTKLIB conversion recipe](data/PROCESSING.md),
including the pinned tool version, time window, signal masks and saved profiles.
The resulting local dataset layout is:

```text
asterx4_rover.obs
asterx4_base_1hz.obs
brdm1290.19p
brdm1290_v304.19p
lord_imu.log
camera-calibration-mono/texcup_port-camchain.yaml
camera-calibration-stereo/texcup_stereo-camchain_ovins.yaml
ground_truth.log
```

RTKLIB-EX and GICI-RTK need GNSS and GT only; visual methods also need IMU and
camera data. Broadcast-navigation files are independent of the receiver
recordings; see their [provenance](data/NAVIGATION.md).

The validated observation profile retains GPS 1C/2W, Galileo 1C/7Q and BeiDou
2I (code, phase, Doppler and SNR), excludes GLONASS, and samples the inclusive
18:09:58–19:17:17 GPST window at 1 Hz. Keep GPST epochs unchanged; estimator
output converters handle UTC mapping. The recipe does not reconstruct the
former full-recording OBS files. The publisher LORD file contains the same
numerical samples as the historical input, with differences confined to comments.
Refer to [source provenance](data/SOURCE_ARCHIVE.md) before claiming exact
reproduction of the saved trajectories. The benchmark has not been rerun with
these newly prepared GNSS observations.

Download `images.h5` from the [TEX-CUP camera archive](https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/camera/)
only when preparing visual inputs (approximately 75 GB):

```bash
mkdir -p "$TEXCUP_DATA/camera_data"
curl --fail --location --continue-at - \
  --output "$TEXCUP_DATA/camera_data/images.h5" \
  https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/camera/images.h5
```

Camera images, generated ROS bags and image packs stay local. Set `TEXCUP_DATA`
when using an external dataset directory; no external estimator checkout is
needed for these downloads.

All visual methods use native **2048 × 732** images. The shared input/reference
contract is in [METHOD_SETUP_GUIDE.md](METHOD_SETUP_GUIDE.md). Review
[BUG_FIXES.md](BUG_FIXES.md) before applying saved source patches.

## 2. Prepare the required environment

The executed methods use CPU processing. These instructions were validated on
Linux x86-64 with Conda; they do not require a GPU or system ROS installation.
Install Conda at `BENCH_CONDA_ROOT`, then initialize it:

```bash
source "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh"
```

Create only the environments needed, once:

```bash
# GVINS and VINS-Fusion; also provides h5py for GICI image preparation.
conda create -y -n benchmark_ros --file environments/benchmark_ros.explicit.txt
# IC-GVINS uses its own matching TBB runtime and headers.
conda create -y -n benchmark_icgvins --file environments/benchmark_icgvins.explicit.txt
# Standalone GICI binaries, without ROS.
conda create -y -n benchmark_gici --file environments/benchmark_gici.explicit.txt
```

These are exact Linux package URLs, including compiler/build dependencies.
RTKLIB instead uses a C/C++ compiler, CMake >=3.16 and Make, plus the small
Python statistics environment described in [STATISTICS.md](STATISTICS.md).
The method guides activate their environment before building/running.

## 3. Run a method

VINS-Fusion and IC-GVINS consume the RTKLIB position solution; run RTKLIB first
when preparing their inputs. OKVIS2-X can use the committed RTKLIB trajectory.
GVINS, GICI-RTK and GICI-RRR can run independently. InGVIO reuses the GVINS
GNSS input converter; follow its guide to build that dependency.

| Guide | What it runs |
|---|---|
| [RTKLIB-EX](projects/RTKLIB/RUN.md) | Build, GNSS processing, conversion, scoring |
| [GICI-RTK](projects/GICI-LIB/RUN.md#gnss-only-rtk) | GNSS-only RTK |
| [GICI-RRR](projects/GICI-LIB/RUN.md#rtk-imu-camera-rrr) | Raw GNSS/IMU/mono processing |
| [VINS-Fusion](projects/VINS-Fusion/RUN.md) | Stereo VIO and global position fusion in ROS |
| [IC-GVINS](projects/IC-GVINS/RUN.md) | Position/IMU/mono fusion in a separate ROS environment |
| [GVINS](projects/GVINS/RUN.md) | Raw GNSS/IMU/mono replay in ROS |
| [InGVIO](projects/InGVIO/RUN.md) | Raw GNSS/IMU/mono invariant-filter replay in ROS |
| [OKVIS2-X](projects/OKVIS2-X/RUN.md) | Stereo VI + GNSS fusion (final BA), no ROS |

These eight methods have saved outputs. IC-GVINS stopped with an
error; the current table uses its explicitly selected pre-divergence prefix.
InGVIO is the upstream-default dual-frequency replay and diverges at the
vehicle's first stop; OKVIS2-X is included from its collected trajectory and uses
the stereo visual-inertial + GNSS final-BA trajectory.
Supplied SeA-RVINS batch/latent/scalar records are also available; follow their
[import instructions](results/SeA-RVINS/README.md) to reproduce conversion and
scoring. These records do not provide a complete recipe for rerunning the estimator.

Prepare inputs once; allow roughly 60 GB for each mono bag/GICI image pack and
116 GB for the stereo bag, plus source HDF5, builds and outputs. Concurrent
replays require enough RAM, disk bandwidth and CPU to keep pace. For simplest
reproduction run one at a time. VINS-Fusion and IC-GVINS have separate ROS
masters/ports, output folders and environments when run together. Never run
two copies on the same port or write into an existing run directory.

## 4. Check completion and collect results

Supervised runs write `results/METHOD/latest/status.json`, `launcher.log` and
`logs/`. Reopen status periodically, for example:

```bash
watch -n 300 cat results/VINS-Fusion/latest/status.json
```

`phase: completed` identifies completed processing. `error` or `stopped` are
terminal failures; lack of errors alone does not establish accuracy. The
method guide states whether collection is automatic or gives the explicit
collection command. Final files are copies in `results/final`, so their use
does not depend on the local `latest` links.

Use [STATISTICS.md](STATISTICS.md) to recalculate the table, finalize failed
runs, or monitor at five-minute intervals and optionally suspend after scoring.
Successful processing can still yield large position errors. All finite errors
within each method's stated reporting interval remain in the metrics. The
current IC-GVINS prefix and the full 4,040-epoch denominator are documented in
[STATISTICS.md](STATISTICS.md).
