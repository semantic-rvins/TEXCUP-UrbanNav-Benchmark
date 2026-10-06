# Reproduce the current comparison

The [main README](README.md) lists the included methods in table order and
links their exact run guides. Start with [dataset and environment setup](RUN_INSTRUCTIONS.md),
then apply the [documented source fixes](BUG_FIXES.md) for the selected method.

To use the saved results only, follow [STATISTICS.md](STATISTICS.md).
[comparison.json](results/final/comparison.json) defines the eleven included rows;
[table_selections.json](results/final/table_selections.json) records the
IC-GVINS pre-divergence interval. The standard statistics command preserves both.
Figure reproduction is documented in [figures/README.md](figures/README.md).

InGVIO and OKVIS2-X have collected trajectories in the comparison. Their
[InGVIO](projects/InGVIO/RUN.md) and [OKVIS2-X](projects/OKVIS2-X/RUN.md) guides
describe the available run setup and provenance limitations.
