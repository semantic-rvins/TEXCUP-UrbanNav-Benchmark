# InGVIO

[InGVIO](https://github.com/ChangwuLiu/InGVIO) combines monocular visual-inertial
estimation with rover raw GNSS. The TEX-CUP adapter uses the native 2048 × 732
port camera, LORD IMU in RFU axes, and GPS/Galileo/BeiDou observations.

**Included result.** The comparison reports the upstream-default
full-resolution replay on the reduced dual-frequency GNSS: 3,976 / 4,040 solved
epochs (98.42% availability), 1928.57 m horizontal RMSE. The collected antenna
trajectory and statistics are in [results/final](../../results/final/README.md);
[STATUS.md](STATUS.md) describes the run and its provenance.

- [RUN.md](RUN.md): pinned source, environment, build, input preparation,
  replay, extraction and collection commands.
- [BUGFIX_NOTES.md](BUGFIX_NOTES.md): the two software patches and the
  dataset-adapter configuration. Every estimator parameter is the shipped
  upstream sportsfield value (`gnss_chi2_test: 0`, `visual_noise: 0.18`).
- [Configurations](config/), [software patches](patches/), and
  [output converter](loaders/extract_est.py).
- [Calibration and evaluation protocol](../../METHOD_SETUP_GUIDE.md).
