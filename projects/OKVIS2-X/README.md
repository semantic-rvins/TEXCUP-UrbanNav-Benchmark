# OKVIS2-X

[OKVIS2-X](https://github.com/ethz-mrl/OKVIS2-X) runs stereo visual-inertial
estimation with a GNSS position stream. The TEX-CUP adapter uses native
2048 × 732 port/star images, LORD IMU in RFU axes, and the collected
[RTKLIB-EX trajectory](../../results/final/RTKLIB.est.csv).

**Included saved result.** The contributor's final bundle-adjusted global
trajectory is collected in [results/final](../../results/final/README.md).
This checkout integration did not rerun the estimator. [STATUS.md](STATUS.md)
describes the available provenance.

- [RUN.md](RUN.md): source pin, dependencies, preparation, replay and scoring.
- [BUGFIX_NOTES.md](BUGFIX_NOTES.md): dependency requirements, GNSS adapter and
  configuration choices. There are no OKVIS2-X source patches.
- [Configuration](config/okvis2_texcup.yaml) and [data/output converters](loaders/).
- [Calibration and evaluation protocol](../../METHOD_SETUP_GUIDE.md).

The input adapter retains RTKLIB quality 1/2 solutions and assigns fixed
per-axis uncertainty by quality. The ENU origin is the first ground-truth
position and is recorded in `gps0/anchor.json`; output conversion inverts that
same coordinate transformation. The global output already refers to the ALT1
antenna, so its lever arm is not added again. The comparison uses final bundle
adjustment and should be described as an offline result.
