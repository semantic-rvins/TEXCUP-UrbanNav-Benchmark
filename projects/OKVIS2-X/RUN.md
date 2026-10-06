# Run OKVIS2-X on TEX-CUP

The saved comparison result uses stereo visual-inertial/GNSS estimation and
final bundle adjustment. It was imported from the contributor's run, whose
available metadata are described in [STATUS.md](STATUS.md). The commands below
use portable paths and preserve its source pin and configuration; they were
not executed as a new full replay during integration. The synchronous
application runs on CPU without ROS.

## 1. Set paths and prepare data

Run commands from the benchmark root in Bash:

```bash
export BENCH="$PWD"
export BENCH_ROOT="$BENCH"
export TEXCUP_DATA="${TEXCUP_DATA:-$BENCH/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
```

Follow [data setup](../../data/README.md) to download the publisher IMU and
ground truth, materialize benchmark calibration profiles locally, and
download `camera_data/images.h5`. The GNSS input is
`results/final/RTKLIB.est.csv`, which is already included. To regenerate it,
follow [RTKLIB](../RTKLIB/RUN.md). No external estimator repository supplies
input data.

## 2. Create an environment and build the pinned source

Use a separate environment for OKVIS2-X dependencies. The shared GICI explicit
list provides the base compiler and numerical libraries; OKVIS additionally
needs PCL, Boost and GeographicLib. Keep Eigen/OpenCV constrained when adding
them:

```bash
source "$BENCH_CONDA_ROOT/etc/profile.d/conda.sh"
conda create -y -n benchmark_okvis --file environments/benchmark_gici.explicit.txt
conda install -y -n benchmark_okvis -c conda-forge \
  'eigen=3.4' 'opencv=4' libboost-devel geographiclib-cpp pcl h5py
conda activate benchmark_okvis
export BENCH_DATA_PYTHON="$CONDA_PREFIX/bin/python"
mkdir -p results/OKVIS2-X
conda list --explicit > results/OKVIS2-X/environment_explicit.txt

git clone https://github.com/ethz-mrl/OKVIS2-X projects/OKVIS2-X/upstream
git -C projects/OKVIS2-X/upstream checkout --detach \
  38043e4afe56d9b32a98434cc74e723737dd2bce
git -C projects/OKVIS2-X/upstream \
  -c url.https://github.com/.insteadOf=git@github.com: \
  submodule update --init --recursive
cmake -S projects/OKVIS2-X/upstream -B projects/OKVIS2-X/upstream/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DUSE_NN=OFF -DHAVE_LIBREALSENSE=OFF -DBUILD_ROS2=OFF \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_C_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc" \
  -DCMAKE_CXX_COMPILER="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++"
cmake --build projects/OKVIS2-X/upstream/build --parallel 8
```

No source patch is applied. Upstream vendors Ceres 2.2 and builds its supereight2
submodule. The additional dependencies above are not represented by a complete
OKVIS-specific environment lock; save the resolved explicit list for a new run.
An existing suitable micromamba/Conda environment can be activated instead.
The converter's Python needs NumPy, h5py and OpenCV (`cv2`).

## 3. Convert the dataset

```bash
"$BENCH_DATA_PYTHON" projects/OKVIS2-X/loaders/convert_texcup.py \
  --data-dir "$TEXCUP_DATA" --rtklib-csv results/final/RTKLIB.est.csv \
  --out projects/OKVIS2-X/data/texcup_euroc --skip-images
"$BENCH_DATA_PYTHON" projects/OKVIS2-X/loaders/convert_texcup.py \
  --data-dir "$TEXCUP_DATA" --rtklib-csv results/final/RTKLIB.est.csv \
  --out projects/OKVIS2-X/data/texcup_euroc --skip-imu --skip-gps
```

`cam0`/`cam1` contain native 2048 × 732 port/star PNGs, roughly 40,372 pairs
and 63 GB. `imu0/data.csv` contains RFU IMU rates with UTC nanosecond stamps.
The GNSS adapter writes `gps0/data.csv` and `gps0/anchor.json`:

- RTKLIB quality 1/2 positions are retained; other quality flags are excluded.
- Standard deviations `(E,N,U)` are `(0.05,0.05,0.10) m` for fixed and
  `(0.50,0.50,1.00) m` for float solutions.
- ECEF antenna positions are expressed in ENU with the first ground-truth
  position as the coordinate origin. The origin is saved for exact inversion.

The current RTKLIB file produces 4,531 GNSS input rows; 26 quality-4 rows are
excluded. Input coverage extends beyond the 4,040-epoch evaluation interval.
The configuration's antenna lever is `[-0.610,-0.052,0.010] m` in RFU.

## 4. Run and monitor

With the build environment active and `BENCH_DATA_PYTHON` exported:

```bash
sbatch projects/OKVIS2-X/run_okvis.sbatch
```

Without Slurm, the same script can run directly:

```bash
bash projects/OKVIS2-X/run_okvis.sbatch
```

The script uses the active environment by default. If activation is needed,
set `BENCH_OKVIS_ENV` to the Conda environment name/path and
`BENCH_CONDA_ROOT` to its Conda installation; keep `BENCH_DATA_PYTHON` set to an
interpreter with the converter dependencies. It verifies both camera exports,
regenerates missing images, and runs `okvis_app_synchronous` with the committed
configuration. Native outputs go to `projects/OKVIS2-X/results/run/`.

Monitor `results/OKVIS2-X/latest/status.json` and
`results/OKVIS2-X/latest/logs/okvis_app.log`. The script reaches `completed`
only after a successful application exit and creation of the final-BA global
trajectory. Extraction and statistics are the next step. Run only one instance
at a time; the script uses shared output paths.

The Slurm template requests 224000 MB, a conservative allocation from the
contributor's machine. The reported peak memory was about 44 GB and application
runtime about seven hours. Adapt resources to the machine; final bundle
adjustment needs additional time and memory after frame processing ends.

## 5. Convert the global output and collect

```bash
"$BENCH_DATA_PYTHON" projects/OKVIS2-X/loaders/global_to_est.py \
  --traj projects/OKVIS2-X/results/run/okvis2-vio-global-final-ba_trajectory.csv \
  --anchor projects/OKVIS2-X/data/texcup_euroc/gps0/anchor.json \
  --out results/OKVIS2-X/latest/est.csv
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect OKVIS2-X=results/OKVIS2-X/latest
```

Upstream's global CSV already includes the IMU-to-antenna lever. The converter
inverts the saved ENU frame to ECEF without adding the lever again or fitting
an alignment to ground truth. The primary comparison uses this offline
final-bundle-adjusted trajectory; the optional causal converter is not needed
for the current metric table.

Evaluation covers 18:09:40–19:16:59 UTC inclusive. Availability and threshold
percentages use all 4,040 epochs; RMSE, MAX and P95 use matched solutions and
retain finite outliers. For failed runs, use the incomplete-collection
procedure in [STATISTICS.md](../../STATISTICS.md).
