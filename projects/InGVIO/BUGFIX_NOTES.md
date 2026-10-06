# InGVIO patch/config review: software fixes only

The reconciled benchmark runs the **upstream-default algorithm** with
**software** fixes only. This mirrors the IC-GVINS review in the repository root
[`BUG_FIXES.md`](../../BUG_FIXES.md).

Upstream: `ChangwuLiu/InGVIO` @ `eca739e894c656a58625c63ef541c9fc31a77e3b`
(recorded in `patches/UPSTREAM_COMMIT`). The clone bundles `camera_model`,
`feature_tracker`, `gnss_comm` and `ingvio_estimator`.

## Active patches (software only)

### `01-build-cxx17-eigen-feature_tracker-dep.patch` — build/compat
Pure build/compatibility, no estimator behavior:
- C++11/14 -> C++17 in all four packages (required by the RoboStack toolchain).
- `feature_tracker/CMakeLists.txt`: replaced a hardcoded
  `/usr/include/eigen3` include with `find_package(Eigen3)` +
  `${EIGEN3_INCLUDE_DIR}` (Eigen 3.4 lives under the conda/micromamba prefix).
- `ingvio_estimator` `cmake/ROS1.cmake` + `package.xml`: declared the missing
  `feature_tracker` dependency. Its generated message headers are only found
  automatically under monolithic `catkin_make`; isolated `catkin build` needs
  the declared dependency. No source logic changed.

### `02-publish-ecef-odometry.patch` — output-only
Adds a global-pose publisher only. Upstream publishes world-frame odometry but
no global/ECEF pose, so the global solution cannot otherwise be recorded from a
bag. The patch adds `/ingvio_estimator/pose_ecef` (`nav_msgs/Odometry`) computed
exactly as the filter's own measurement model maps world -> ECEF
(`p_ecef = T_enu2ecef * R_w2enu(yof) * p_w`, using the filter's YOF state when
present, else the aligner's initial yaw), plus the matching `R_i2ecef` and
velocity, guarded against NaN. It touches only `IngvioFilter`'s visualize path;
it reads existing state and adds no measurement, factor, reset or state change.

## Configuration (dataset adapters only)
Consistent with `METHOD_SETUP_GUIDE.md`; every estimator parameter equals the
shipped `config/sportsfield/ingvio_mono.yaml` values:
- Camera intrinsics/distortion + extrinsic for the native 2048x732 port camera.
- `gravity_norm: 9.7935` — the physical value for the Austin site, not tuning.
- `use_fix_time_offset: 1`, `gnss_local_offset: -18.0` — GPST->UTC data mapping.
- Topic names and the mono/GNSS I/O wiring.
IMU process/bias noise, all init covariances, `visual_noise` (0.18),
`gnss_chi2_test` (0) and `gnss_strong_reject` (1) are the shipped sportsfield
defaults, unchanged.

## Result
Active baseline = upstream sportsfield algorithm + two software patches (01, 02)
+ TEX-CUP data adapters. With `gnss_chi2_test: 0` the filter diverges at the
vehicle's first full stop; the finite errors are retained in the statistics, as
with the other reconciled upstream-default baselines.
