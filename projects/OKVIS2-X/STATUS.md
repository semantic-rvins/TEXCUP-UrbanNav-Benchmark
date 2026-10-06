# OKVIS2-X status

**Included saved result.** The contributor reports a completed stereo
visual-inertial/GNSS replay with final bundle adjustment. Its collected
trajectory covers 4,040 / 4,040 evaluation epochs (100% availability).
See [current statistics](../../results/final/statistics.md) and
[trajectory provenance](../../results/final/OKVIS2-X.source.json).

The provenance records the trajectory and a completed run status, but its
run manifest contains only the display name. Complete build logs, input
validation and runtime configuration are not included in that metadata. The
source pin and replay recipe are preserved in [RUN.md](RUN.md); this merge
reviewed their portability without rebuilding or rerunning the estimator.

The primary result is the **final bundle-adjusted global antenna trajectory**,
not the causal trajectory. Inputs are native stereo images, RFU IMU samples,
and quality-filtered RTKLIB-EX positions. No OKVIS2-X estimator source patch is
used; fixed GNSS uncertainties and dataset-specific IMU settings remain part
of the reproduced configuration. See [BUGFIX_NOTES.md](BUGFIX_NOTES.md).
