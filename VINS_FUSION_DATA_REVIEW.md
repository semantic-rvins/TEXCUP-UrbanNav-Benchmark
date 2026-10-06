# VINS-Fusion input/output contract and software fixes

The saved result uses upstream estimator settings with the build, diagnostics
and rotation software fixes listed below. Follow the
[run guide](projects/VINS-Fusion/RUN.md) to reproduce this implementation.

## Active source and GPS association

Apply the three patches listed in
[patches/series](projects/VINS-Fusion/patches/series), in order, to upstream
`be55a937a57436548ddfb1bd324bc1e9a9e828e0`. They provide build compatibility,
origin/path diagnostics, and valid quaternion/rigid-transform handling.
Global optimization uses the upstream five iterations. Tracking
receives every image pair; upstream admits every second pair to estimation.

The small `texcup_gps_upstream.bag` sidecar interpolates RTKLIB positions onto
those admitted image timestamps. One rosbag player reads the native image/IMU
bag and the sidecar together, excludes the original bag's `/gps` topic, and
remaps `/gps_baseline` to `/gps`. This preserves the original image and IMU
payloads/timestamps while meeting global fusion's ±10 ms association rule.

## Data contract

| Item | Setting |
| --- | --- |
| IMU | LORD left/forward/down → RFU with `(-x,y,-z)`, for both acceleration and gyro. Feed rates in m/s² and rad/s; VINS integrates them. |
| Time | ROS uses UTC Unix seconds. Subtract 18 s once from LORD GPST; keep camera UTC and the already-UTC RTKLIB CSV. |
| Images | Port → cam0, star → cam1; unrectified `mono8`, native 2048×732. Each pair uses the port timestamp. Maximum raw skew 1.597 ms is below the 3 ms sync gate. |
| Intrinsics | `texcup_cam0.yaml` and `texcup_cam1.yaml` match the native stereo calibration, including distortion. |
| Extrinsics | Camera→RFU: port rotation `[[1,0,0],[0,0,1],[0,-1,0]]`, translation `[-0.303,0.011,0.033] m`. Star transform is `T_body_cam0 * inverse(T_cam1_cam0)`; baseline 0.49767 m. |
| Calibration settings | `estimate_extrinsic: 1`, `estimate_td: 0`, `td: 0`. Starting calibration matches the requested UrbanRTK parameters. |
| GNSS | Latitude/longitude in degrees; ellipsoidal height in metres. This consumer divides residuals by `position_covariance[0]`, so the adapter puts sigma there: Q1 0.05 m, other qualities 0.75 m, using the larger adjacent sigma for interpolation. |
| Output | Body pose in the actual first accepted GPS position's fixed GeographicLib ENU frame. Convert with `p_ecef = origin_ecef + R_ecef_from_enu * p_enu`. |
| Reference point | GT is antenna 2 / ALT1. Add RFU IMU-to-ALT1 lever `[-0.610,-0.052,0.010] m` once using estimated attitude; do not transform GT. |

Sampled actual bag pixels and IMU values match the raw data after these
conversions. Camera calibration and bag metadata are checked against provenance.
The sidecar is round-trip checked, and its GPS timestamps must all belong to
upstream-admitted images.

The GNSS input lever removal remains approximate: global fusion has no lever
factor, so the adapter uses RTKLIB course as yaw and zero roll/pitch. It restores
the lever on output with estimated attitude. Two rotations of this 0.612 m lever
can differ by at most about 1.225 m per pose. The GNSS quality-based sigmas are
chosen weights, not RTKLIB's measured covariance; Q4 DGPS also receives 0.75 m.
These limitations affect accuracy even when encodings and frame directions match.

## Rotation software fix

For body B, local VIO world L and global ENU world G, the patch computes:

```text
R_G_from_L = R_G_from_B * transpose(R_L_from_B)
t_G_from_L = p_G - R_G_from_L * p_L
```

This matches upstream's `T_global_body * inverse(T_local_body)` for rigid poses.
Normalization preserves the attitude of finite, nonzero quaternions. The
compiled regression checks unit rotations and newest-pose mapping; it does not
guarantee trajectory accuracy. The live monitor and final verification reject
invalid output rotations. No GNSS weights or factors are changed by this fix.

The scored output is live global odometry, not the retrospectively optimized
`global_path`. The estimator combines VIO relative factors and robust GPS factors;
its result is not guaranteed to be more accurate than RTKLIB alone. All 4,040
benchmark epochs remain in percentage denominators, and finite outliers are kept.

Use [VINS-Fusion RUN.md](projects/VINS-Fusion/RUN.md) for exact commands,
[BUG_FIXES.md](BUG_FIXES.md) for software-fix reproduction, and
`results/VINS-Fusion/latest/status.json` for runtime status.
