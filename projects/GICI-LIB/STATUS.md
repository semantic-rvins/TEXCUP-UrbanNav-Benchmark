# GICI-LIB status

**GICI-RTK and GICI-RRR have completed** the current 4,040-epoch protocol.
GICI-RTK uses rover/base GNSS without source changes. GICI-RRR uses rover/base
GNSS, LORD IMU and native port-camera images with the documented patch set.
RRR antenna conversion uses its estimated attitude and recorded fixed ENU
origin; finite outliers remain in the saved statistics.

See [current statistics](../../results/final/statistics.md),
[run instructions](RUN.md) and [source-fix scope](../../GICI_PATCH_REVIEW.md).
Processing completion does not establish positioning accuracy.

A new replay writes live status and logs under `results/GICI-RTK/latest` or
`results/GICI-RRR/latest`. A `completed` phase indicates finished processing;
`error` or `stopped` retains failure status. RRR automatically collects its
verified result into `results/final`; follow the RTK run guide's collection
command for the GNSS-only result.
