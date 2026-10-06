# Final benchmark results

See [STATISTICS.md](../../STATISTICS.md) for the scoring environment, commands,
evaluation protocol, and recovery of stopped runs.

This folder holds antenna trajectories and statistics for completed methods and
explicitly collected failed or partial runs, identified by the Status column.
It contains real file copies, so recalculation does not depend on the
per-run `latest` links. Ground truth is TEX-CUP antenna 2 / ALT1.
Download ground truth from the publisher using [data instructions](../../data/README.md).
It stays in the local dataset directory and is not copied into this results folder.

- `statistics.md`, `statistics.csv`, `statistics.json`: combined results.
- `comparison.json`: methods, display labels and row order for this comparison.
  Only these methods enter the generated tables; pending methods are added here
  after their trajectories and provenance have been collected.
- `table_selections.json`, when present: explicit time prefixes for the combined tables;
  later epochs count as unavailable while the full 4,040-epoch denominator is retained.
  These selections do not alter the per-method files below.
- `METHOD.est.csv`: antenna trajectory in UTC seconds of day, ECEF or LLA.
- `METHOD.errors.csv`: all 4,040 evaluation epochs, including NaN gaps.
- `METHOD.eval.json`, `METHOD.statistics.json`, `METHOD.error.png`: per-method results.
- `METHOD.source.json`, `manifest.json`: original run and calculation provenance.
  Paths are repository-relative; symbolic environment references and original
  run identifiers are metadata, not prerequisites for recalculating statistics.
  Input metadata, code revisions and saved configuration values are retained.

From the repository root, with NumPy and Matplotlib installed:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
```

To add or replace a completed method, then recalculate all methods:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect METHOD=results/METHOD/latest
```

Availability and threshold percentages use all 4,040 epochs, including missing
estimates. Distance statistics use solved epochs; no missing error is replaced
by zero. Runtime logs and ROS bags are generated when methods are replayed;
the original run directories recorded in provenance need not be distributed.

Failed runs require explicit `--allow-incomplete` when collecting. Their original
status and recovery exclusions remain in `METHOD.source.json`. A failed method
with no usable output has zero availability and undefined distance statistics.
