# IC-GVINS on TEX-CUP

**Software fixes cover state lifetime and feature-tracking memory safety.** Follow [RUN.md](RUN.md)
to clone pinned source, apply the active software patches, build, prepare native
inputs, run in the background, and compute statistics. Software fixes are
explained in [BUG_FIXES.md](../../BUG_FIXES.md); measured results are in
[results/final](../../results/final/README.md).

The comparison table uses the **pre-divergence prefix through 18:34:59 UTC**:
**1,502 / 4,040 epochs (37.18% availability)**. The complete saved trajectory
remains available; the source replay later stopped with an unresolved SIGSEGV.
The prefix selection and its limits are recorded in
[table_selections.json](../../results/final/table_selections.json) and
[STATUS.md](STATUS.md).

IC-GVINS uses RTKLIB antenna positions, LORD IMU rates, and the mono port camera
through ROS 1. It runs on CPU with native **2048 × 732** images. Upstream
stationary thresholds and GNSS weighting are preserved. Active patches are 03
(diagnostics/shutdown), 04 (time-window/index bounds), 05 (GNSS-supported
state retention), 06 (prior pointers), 07 (feature-grid bounds), and 08
(Eigen temporary lifetime). These are the complete active patch set.

| Input or calibration | Setting |
|---|---|
| LORD raw to FRD IMU | `(y, −x, z)` for acceleration and gyro rates |
| IMU-to-ALT1 lever in FRD | `[-0.052, -0.610, -0.010]` m |
| Camera-to-FRD quaternion, xyzw | `[0.5, 0.5, 0.5, 0.5]` |
| Camera translation in FRD | `[0.011, -0.303, -0.033]` m |
| Port intrinsics, fx/fy/cx/cy | `[1355.07090197, 1360.08043642, 1013.57243982, 330.36528878]` |
| Radtan, k1/k2/p1/p2 | `[-0.04362915, 0.03696032, 0.00059401, 0.00057094]` |
| Online extrinsic/time-delay estimation | Enabled; initial delay 0 s |
| Fixed GNSS sigma, horizontal/vertical | `0.05/0.10` m |
| Float GNSS sigma, horizontal/vertical | `0.50/1.00` m |

The bag contains 40,372 images, 404,495 IMU samples, and 4,014 GNSS messages.
The Q1/Q2 input filter excludes 26 Q4 DGPS epochs. GNSS covariance fields
contain squared sigmas; antenna positions enter unchanged because IC-GVINS
models the lever internally. ROS timestamps are UTC; the node converts them
to GPST internally.

Output conversion adds the ALT1 lever to IMU position using estimated
FRD-to-local-NED attitude. GT already represents antenna 2 / ALT1. All 4,040
epochs from 18:09:40 through 19:16:59 UTC remain in percentage denominators;
finite outliers within the selected prefix are retained. The prefix is a
partial result and does not demonstrate full-route accuracy or completion.

[STATUS.md](STATUS.md) describes completion checks. Live progress is
`results/IC-GVINS/latest/status.json`. Error counts include nonfatal calibration
rejections and tracking losses; completion alone does not establish accuracy.
