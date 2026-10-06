# InGVIO status

**Included.** InGVIO (ChangwuLiu/InGVIO @ `eca739e`, an invariant-filter
monocular visual-inertial–raw-GNSS estimator) is in the comparison with the
upstream-default full-resolution replay on the reduced dual-frequency GNSS:
**3,976 / 4,040 solved epochs (98.42% availability)**, **1928.57 m** horizontal
RMSE, 11.92 m median, 161.77 m P95 and 43,336 m maximum; 3-D RMSE 2291.38 m.
Percentage metrics use the full 4,040-epoch grid; distance metrics use the
solved epochs and retain their finite outliers.

Run: native 2048 × 732 images (`--downscale 1`), the shipped sportsfield
algorithm (`gnss_chi2_test: 0`, `visual_noise: 0.18`, `gnss_strong_reject: 1`),
software patches 01 (build/c++17/Eigen/dependency) and 02 (output-only ECEF
publisher), replay at rate 1.0 with the tracker keeping up (40,337 / 40,372
frames). The trajectory covers 18:10:44–19:16:59 UTC; the ~64 s GVIO alignment
warm-up at the window start is the only gap. With the shipped default the
filter diverges at the vehicle's first full stop (18:17:07 UTC): the epoch-level
strong-reject gate is a no-op at more than 14 residual rows, so downtown NLOS
pseudoranges go unrejected. The resulting finite errors are retained, as for
the other upstream-default baselines.

Follow [RUN.md](RUN.md) for the pinned source, build, input preparation,
replay and collection. Provenance is in
[InGVIO.source.json](../../results/final/InGVIO.source.json).
