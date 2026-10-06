# Benchmark figures

`fig_error_cdf.png` and its vector PDF use the current results in
[results/final](../results/final/README.md). The upper panel shows horizontal
errors for **SeA-RVINS (batch)**, **SeA-RVINS (latent)**, and **SeA-RVINS (scalar)**.
The lower panel uses the methods, order and labels from
[comparison.json](../results/final/comparison.json): RTKLIB-EX, GICI-RTK,
GICI-RRR, VINS-Fusion, IC-GVINS, GVINS, InGVIO, OKVIS2-X, and the three SeA-RVINS variants.
GICI-RTK is the GNSS-only result.

From the repository root, using the [statistics environment](../STATISTICS.md)
and [locally downloaded ground truth](../data/README.md):

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
python common/plot_error_cdf.py --results results/final --out-prefix figures/fig_error_cdf
```

The plotting commands read saved error grids and do not themselves need
TEX-CUP source data. Recalculating those error grids requires local ground
truth as shown above.

The standalone [CDF PNG](fig_error_cdf_only.png) and [PDF](fig_error_cdf_only.pdf)
show **RTKLIB-EX, GICI-RRR, VINS-Fusion, SeA-RVINS (batch), SeA-RVINS (latent),
and SeA-RVINS (scalar)**,
without the upper time-series panel. The CDF plotting area is
**6.08667 × 2.35636 inches**, preserving the original height and width per log
decade. The legend sits inside the lower right corner, with no
title or explanatory footer. Dashed vertical lines mark **1.0 m** and **1.5 m**.
The displayed x-axis is cropped to **10⁻²–10³ m**.
Regenerate them with:

```bash
python common/plot_error_cdf.py --cdf-only
```

This writes `fig_error_cdf_only.{png,pdf,provenance.json}` on a 7.08267 × 3.04636 inch
canvas. The two-panel figure remains available separately.

The additional [CDF without batch (PNG)](fig_error_cdf_only_no_batch.png) and
[vector PDF](fig_error_cdf_only_no_batch.pdf) retain the same plotting-area
dimensions and axis settings. Their top and left white margins are cropped,
leaving 0.02 inch padding around the labels. In this version, latent is mustard (solid) and scalar is green
(dashed), exchanging their colors from the batch-containing figure. Regenerate
this version with:

```bash
python common/plot_error_cdf.py --cdf-only --swap-sea-colors --trim-top-left \
  --method RTKLIB --method GICI-RRR --method VINS-Fusion \
  --method SeA-RVINS-latent-robust --method SeA-RVINS-scalar-robust \
  --out-prefix figures/fig_error_cdf_only_no_batch
```

Every CDF uses the complete **4,040-epoch** denominator. Missing estimates
remain unavailable; finite outliers within the reported time selection are
retained. IC-GVINS uses the same explicit pre-divergence prefix as the
statistics table: before **18:35:00 UTC**, with **1,502 / 4,040** available
epochs (**37.18%**). The plot validates the full source error grid, then applies
the time selection from [table_selections.json](../results/final/table_selections.json)
on a copy. Its full saved source data remain unchanged.
This retrospective partial result does not demonstrate full-route performance.
The two-panel logarithmic
axis extends to the largest recorded horizontal error; the standalone crop
only changes the displayed range, without filtering or renormalizing the CDF.
Each adjacent `.provenance.json` records inputs, labels, applied time
selections and plot settings.
CDF values count errors **≤ x**; the statistics table's `<1.0 m` and `<1.5 m`
columns retain their strict thresholds.
