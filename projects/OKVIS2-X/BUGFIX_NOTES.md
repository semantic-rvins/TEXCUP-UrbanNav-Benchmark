# OKVIS2-X build and data-adapter requirements

Upstream is pinned to `38043e4afe56d9b32a98434cc74e723737dd2bce`
([ethz-mrl/OKVIS2-X](https://github.com/ethz-mrl/OKVIS2-X)). No estimator source
patch is supplied. The contributed recipe uses Eigen 3.4, OpenCV 4, vendored
Ceres 2.2, and `USE_NN=OFF`, `HAVE_LIBREALSENSE=OFF`, `BUILD_ROS2=OFF`.
[RUN.md](RUN.md) gives portable build and execution commands.

## Dependencies

Keep Eigen 3.4 and OpenCV 4 constraints when installing PCL and the other
build dependencies. Use `geographiclib-cpp` for the C++ headers/libraries.
`CMAKE_POLICY_VERSION_MINIMUM=3.5` permits older vendored CMake projects to
configure with recent CMake. Submodules must be initialized; an HTTPS rewrite
can replace their SSH URLs. The synchronous application uses CPU and does not
require the optional neural-network or camera-device components.

## Input and output adapters

- `loaders/convert_texcup.py` reads the canonical
  `results/final/RTKLIB.est.csv`. It retains quality 1/2 rows, converts antenna
  ECEF positions to ENU, and records the first ground-truth position as the
  coordinate origin in `gps0/anchor.json`.
- Per-axis `(E,N,U)` standard deviations are `(0.05,0.05,0.10) m` for fixed
  solutions and `(0.50,0.50,1.00) m` for float solutions. Other quality flags
  are excluded. These are explicit GNSS weighting/input-selection choices;
  they are not inferred RTKLIB covariance or software defect fixes.
- Images remain at native 2048 × 732 resolution. The converter supports
  `--skip-images`, `--skip-imu` and `--skip-gps` for separate preparation steps.
  IMU rates are RFU; timestamps are UTC nanoseconds.
- The config supplies `r_SA = [-0.610,-0.052,0.010] m`. Upstream's global
  trajectory already applies this antenna lever. `global_to_est.py` only
  inverts the saved ENU coordinate frame; it does not fit an alignment to the
  ground-truth trajectory or apply the lever twice.

## Configuration and resources

The checked-in profile enables final bundle adjustment, disables loop closure
and online camera extrinsic estimation, and supplies the TEX-CUP camera and
LORD IMU parameters. The IMU noise and initial accelerometer bias are explicit
benchmark settings, not proof that every upstream default is unchanged.
No numeric configuration was changed during this merge.

The contributed replay report measured roughly 44 GB peak resident memory and
seven hours of application runtime. The Slurm template conservatively requests
224000 MB; adjust resource requests to the machine. This allocation is not a
claim that the algorithm needs 200 GB. Exporting the stereo images requires
approximately 63 GB of additional disk space.

The saved result has limited runtime metadata; see [STATUS.md](STATUS.md).
