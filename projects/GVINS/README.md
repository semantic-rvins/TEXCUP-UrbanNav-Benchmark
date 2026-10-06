# GVINS on TEX-CUP

**Full replay and evaluation completed; large outliers remain.** Follow
[RUN.md](RUN.md) to clone pinned source, apply patches, build, prepare native
inputs, run in the background, and collect statistics. See
[BUG_FIXES.md](../../BUG_FIXES.md) for patch scope and
[results/final](../../results/final/README.md) for measured results.

GVINS uses **2048 × 732 mono port images, RFU IMU rates, and rover-only raw GNSS**
through ROS 1 on CPU. It does not consume RTKLIB positions or base-station data.
GPS/Galileo/BeiDou pseudorange and Doppler come from RINEX; GLONASS is excluded.

| Input or calibration | Setting |
|---|---|
| LORD raw to RFU | `(−x, y, −z)` for acceleration and gyro rates |
| Camera-to-RFU rotation | `[[1,0,0],[0,0,1],[0,-1,0]]` |
| Camera translation in RFU | `[-0.303, 0.011, 0.033]` m |
| IMU-to-ALT1 lever in RFU | `[-0.610, -0.052, 0.010]` m |
| Camera intrinsics/distortion | Native mono calibration in [gvins_texcup.yaml](config/gvins_texcup.yaml) |
| Online camera extrinsic estimation | Enabled |
| ROS timestamps / GNSS payload times | UTC / GPST |
| GNSS-local offset | `gnss_local_time_diff: 18.0`; online sync disabled |

**The active source patch includes behavior changes beyond timing fixes:**
0.15 s GNSS-camera pairing, two-frame GNSS alignment/yaw gates, disabled
translation/height-jump reboot triggers, and ephemeris retention through
resets. These are disclosed parts of this setup, not an unchanged upstream
algorithm. The timing fixes also require rebuilt C++; YAML alone is insufficient.

The input bag contains 40,372 images, 450,561 IMU samples, and 8,661 GNSS
messages. Scoring uses all 4,040 epochs from 18:09:40 through 19:16:59 UTC.
GT already represents antenna 2 / ALT1. Output conversion uses estimated
camera attitude and initial camera extrinsics to recover body attitude; this
remains an approximation while online extrinsics vary.

[STATUS.md](STATUS.md) describes completion checks. Live status is
`results/GVINS/latest/status.json`. After the supervisor completes, the manual
collection command in [RUN.md](RUN.md#5-complete-evaluation-and-collect-the-shared-statistics)
updates `results/final`; GVINS does not collect automatically.
