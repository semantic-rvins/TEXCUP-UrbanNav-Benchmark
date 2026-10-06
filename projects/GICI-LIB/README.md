# GICI-LIB on TEX-CUP

[RUN.md](RUN.md) contains the pinned public source, environment, build, input
preparation, replay and statistics commands for both completed variants.

| Variant | Sensors | Source changes |
|---|---|---|
| GICI-RTK | Rover/base GNSS | Unpatched upstream |
| GICI-RRR | Rover/base GNSS, RFU IMU, native port camera | Guards, low-feature fallback, origin/EOF diagnostics |

Use the active `*_dualfreq_post.yaml` profiles. Both exclude GLONASS from
measurements and start at 18:09:40 UTC (18:09:58 GPST). GICI-RTK exports
antenna positions directly. GICI-RRR translates IMU output to ALT1 using
its estimated attitude and recorded fixed ENU origin.

The RRR low-feature fallback changes feature acceptance. Read the
[patch review](../../GICI_PATCH_REVIEW.md) and
[calibration guide](../../METHOD_SETUP_GUIDE.md) when interpreting this baseline.
Current results, including large finite outliers, are in the
[shared statistics table](../../results/final/statistics.md).
