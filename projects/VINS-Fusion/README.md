# VINS-Fusion on TEX-CUP

**Full replay completed.** Follow [RUN.md](RUN.md) to clone pinned source,
apply the active software patches, build, prepare native inputs, run in the
background, and compute statistics. See [BUG_FIXES.md](../../BUG_FIXES.md) for
fix explanations and [results/final](../../results/final/README.md) for results.

VINS-Fusion uses **2048 × 732 stereo images, RFU IMU rates, and RTKLIB positions**
through ROS 1. It runs on CPU. Upstream tracks every pair and admits every second
pair to VIO; global fusion retains five iterations and variable poses throughout
its graph. The active patches provide build compatibility, output diagnostics,
and normalized rotations. Archived algorithm patches are excluded.

| Input or calibration | Setting |
|---|---|
| LORD raw to RFU IMU | `(−x, y, −z)` for acceleration and gyro rates |
| Camera-to-RFU rotation | `[[1,0,0],[0,0,1],[0,-1,0]]` |
| Port camera translation in RFU | `[-0.303, 0.011, 0.033]` m |
| Stereo intrinsics/distortion | Native source calibration; see [camera YAMLs](config/texcup_cam0.yaml) |
| Right camera transform | Port transform composed with measured stereo calibration |
| IMU-to-ALT1 lever in RFU | `[-0.610, -0.052, 0.010]` m |
| Online camera extrinsic estimation | Enabled |
| GNSS sigma | Q1: 0.05 m; other RTKLIB qualities: 0.75 m |
| ROS time / GNSS source time | UTC / GPST converted by subtracting 18 s |

The native bag supplies 40,372 stereo pairs and 404,495 IMU samples. A small GPS
sidecar supplies 4,040 positions interpolated onto estimator-admitted image
timestamps. The player excludes the native bag's GPS topic. Global fusion has
no lever model, so input body-position conversion uses RTKLIB course and zero
roll/pitch; this remains an approximation. The consumer reads its NavSatFix
covariance field as sigma, despite the field's standard variance convention.

Output conversion adds the ALT1 lever using estimated attitude and the actual
fixed ENU origin. GT already represents antenna 2 / ALT1. Scoring uses all
4,040 epochs from 18:09:40 through 19:16:59 UTC inclusive.

[STATUS.md](STATUS.md) describes completion checks. Live progress is
`results/VINS-Fusion/latest/status.json`; detailed logs remain with that run.
