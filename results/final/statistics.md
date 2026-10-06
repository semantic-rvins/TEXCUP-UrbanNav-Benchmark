# Final benchmark statistics

2019-05-09 **18:09:40–19:16:59 UTC**, inclusive: **4,040 epochs**.

Percentages use all epochs. RMSE, MAX and P95 use solved epochs; missing errors are undefined and all finite outliers within the stated reporting interval are retained.

Status distinguishes completed processing from explicitly collected failed or partial runs. Source provenance identifies imported results and local replays.

| Method | Status | Total epochs | Solved epochs | Avail. (%) | Hor. RMSE (m) | Hor. MAX (m) | Hor. <1.0 m (%) | Hor. <1.5 m (%) | Hor. P95 (m) | 3D RMSE (m) | 3D MAX (m) | 3D P95 (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| RTKLIB-EX | completed | 4040 | 4040 | 100.00 | 4.95 | 58.56 | 46.14 | 59.23 | 8.41 | 10.31 | 66.99 | 21.52 |
| GICI-RTK | completed | 4040 | 3907 | 96.71 | 4.69 | 177.00 | 77.60 | 87.43 | 5.70 | 7.25 | 265.26 | 8.64 |
| GICI-RRR | completed | 4040 | 3891 | 96.31 | 92.44 | 1122.59 | 48.54 | 55.97 | 140.57 | 92.99 | 1122.73 | 147.72 |
| VINS-Fusion | completed | 4040 | 4036 | 99.90 | 5.10 | 35.86 | 44.26 | 57.82 | 9.68 | 10.48 | 59.07 | 22.38 |
| IC-GVINS | error (pre-divergence partial) | 4040 | 1502 | 37.18 | 7.68 | 30.47 | 9.16 | 14.31 | 21.95 | 8.10 | 30.47 | 21.97 |
| GVINS | completed | 4040 | 3664 | 90.69 | 436.70 | 19793.25 | 22.48 | 39.33 | 28.76 | 4289.51 | 229001.61 | 75.42 |
| InGVIO | completed | 4040 | 3976 | 98.42 | 1928.57 | 43336.34 | 5.00 | 6.58 | 161.77 | 2291.38 | 43555.92 | 223.53 |
| OKVIS2-X | completed | 4040 | 4040 | 100.00 | 19.29 | 69.44 | 31.29 | 44.98 | 50.17 | 30.94 | 91.05 | 73.07 |
| SeA-RVINS (batch) | completed | 4040 | 4040 | 100.00 | 0.38 | 2.91 | 97.15 | 99.63 | 0.90 | 1.30 | 5.49 | 2.67 |
| SeA-RVINS (latent) | completed | 4040 | 4040 | 100.00 | 0.39 | 1.60 | 96.16 | 99.90 | 0.90 | 1.41 | 4.04 | 3.01 |
| SeA-RVINS (scalar) | completed | 4040 | 4040 | 100.00 | 0.39 | 1.63 | 97.00 | 99.98 | 0.86 | 1.73 | 5.85 | 3.37 |

**IC-GVINS partial result:** scoring epochs from 18:09:40 UTC up to 18:35:00 UTC (exclusive). 1,502 retained solved epochs; all percentages still use **4,040 epochs**. Later epochs count as unavailable in this table. Conservative pre-divergence reporting boundary; not a proven exact failure instant. Earlier recoverable errors are retained without magnitude-based filtering. Full saved output: [IC-GVINS.statistics.json](IC-GVINS.statistics.json).

The time selections are saved in [table_selections.json](table_selections.json) and apply to the combined Markdown, CSV and JSON tables. The original trajectories, error grids and per-method statistics remain untruncated. Partial distance metrics describe the selected interval, not full-route accuracy.

Download ground truth following [data instructions](../../data/README.md), then recalculate every method from its saved antenna trajectory:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
```
