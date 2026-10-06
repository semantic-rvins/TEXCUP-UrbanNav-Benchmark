# Reproduce the benchmark statistics

The current table is [results/final/statistics.md](results/final/statistics.md).
The committed `results/final` directory contains antenna trajectories,
per-epoch errors and provenance. Ground truth is downloaded from the
[TEX-CUP publisher and prepared locally](data/SOURCE_ARCHIVE.md). Scoring
saved trajectories requires that local ground truth, but no estimator builds,
ROS bags or source-run directories. Reading the saved statistics needs no
data download.

The supplied **SeA-RVINS (batch)**, **SeA-RVINS (latent)**, and **SeA-RVINS (scalar)** records are included
alongside the local method replays. [SeA-RVINS import instructions](results/SeA-RVINS/README.md)
describe Git LFS retrieval, antenna/UTC conversion and provenance. Their saved
common trajectories can be scored with the locally prepared TEX-CUP ground
truth, without downloading the SeA-RVINS record archives.

## Install the scoring environment

Use Python 3.12 and the versions in
[statistics-requirements.txt](environments/statistics-requirements.txt):

```bash
python3.12 -m venv .venv-statistics
source .venv-statistics/bin/activate
python -m pip install -r environments/statistics-requirements.txt
```

The pinned stack was verified with Python 3.12.14. Run the following commands
from the benchmark repository root.

## Recalculate the saved table

First prepare `ground_truth.log` using [data setup](data/README.md). Then:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
```

This recomputes every `METHOD.est.csv` using the locally prepared ground truth,
then updates `statistics.md`, `statistics.csv`, `statistics.json` and each
method's error grid and statistics. Source-run paths are repository-relative
provenance identifiers describing where outputs were produced. They are not
required input paths: recalculation does not read those locations. See
[PATH_SETUP.md](PATH_SETUP.md) for the repository layout and path conventions.

[comparison.json](results/final/comparison.json) specifies the included methods,
display labels and row order: RTKLIB-EX, GICI-RTK (GNSS only), GICI-RRR,
VINS-Fusion, IC-GVINS, GVINS, InGVIO, OKVIS2-X, SeA-RVINS (batch), SeA-RVINS (latent),
and SeA-RVINS (scalar). RTKLIB-EX uses the existing `RTKLIB` filename prefix.
Recalculation selects only the methods in this file. InGVIO is the
upstream-default dual-frequency replay and OKVIS2-X the final-BA stereo VI+GNSS
run; their method guides describe the run configuration and provenance.

The combined table uses the IC-GVINS pre-divergence prefix, **18:09:40–18:34:59 UTC**,
as recorded in [table_selections.json](results/final/table_selections.json).
It retains **1,502 solved scoring epochs** and uses **4,040 epochs** for all percentages
(availability **37.18%**). Later epochs count as unavailable in the combined
Markdown, CSV and JSON tables. This is an explicitly selected diagnostic
interval, not a full-route accuracy result. The per-method trajectory, error grid
and statistics retain the complete saved output. Before replacing a run,
review or remove its selection entry so the earlier cutoff is not reused.

Regenerate the current horizontal-error time series and CDF figure with:

```bash
python common/plot_error_cdf.py --results results/final --out-prefix figures/fig_error_cdf
```

This writes PNG, PDF and a provenance JSON. CDF percentages also use all 4,040
epochs, including missing estimates. The plot uses the current result files.
Plot membership and labels follow `comparison.json`; the standalone CDF uses
its documented subset. Any plot containing IC-GVINS applies the same
pre-divergence time selection as the table while keeping all 4,040 epochs in
the denominator.

To recalculate in a separate directory, copy the complete `results/final`
directory and pass the copy to `--results`. Keep `comparison.json`,
`table_selections.json` and `manifest.json` together with the trajectories.
Pass the independently downloaded ground truth using `--gt`; the collector
does not copy it into the result directory.

## Evaluation protocol

| Setting | Definition |
|---|---|
| Date and window | 2019-05-09, **18:09:40–19:16:59 UTC**, inclusive |
| Total epochs | **4,040** ground-truth epochs at 1 Hz |
| Reference position | Antenna 2 / **ALT1**; `ground_truth.log` already gives this antenna position |
| Estimate format | CSV with `utc_sec` in UTC seconds of day and antenna ECEF `ecef_x,ecef_y,ecef_z` in metres, or geodetic `lat_deg,lon_deg,h_ell` |
| Association | Nearest estimate within ±0.5 s of each GT epoch; GT interpolated in ECEF at that estimate's timestamp |
| Horizontal error | Norm of east/north error in the evaluator's fixed ENU frame |
| 3D error | `hypot(horizontal error, vertical error)` |
| Availability | `100 × solved epochs / 4040` |
| Horizontal <1.0 m / <1.5 m | Strict thresholds; percentages use **all 4,040 epochs** |
| RMSE, MAX, P95 | Solved epochs only; P95 uses NumPy's linear percentile |
| Missing estimates | Unavailable; fail both horizontal thresholds; errors remain undefined |
| Outliers | All finite positioning errors are retained; no GT-based rejection or alignment |

`METHOD.errors.csv` contains every evaluation epoch, with `NaN` for unavailable
errors. A run with no usable trajectory has availability 0%, threshold
percentages 0%, and undefined (`null` in JSON) RMSE/MAX/P95. Completion status
reports whether processing finished; it does not certify positioning accuracy.

The central collector uses the same protocol for every method. Estimator
initialization losses and missing GNSS/vision output remain in the denominator.

## Add a completed run

After the method's supervisor and its collection process have exited:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect METHOD=results/METHOD/latest
```

Repeat `--collect` to replace more than one method. Collection copies
estimator outputs and records configuration, input validation, run status and
provenance. Ground truth remains at its local data path.
Concurrent collections are serialized with a file lock.

For a new results directory, supply ground truth explicitly:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/calculate_statistics.py --results results/new_comparison \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect METHOD=results/METHOD/latest
```

Without `--gt`, the default is `$TEXCUP_DATA/ground_truth.log` when
`TEXCUP_DATA` is set, otherwise `data/tex_cup/ground_truth.log` under this
repository. An explicitly configured `TEXCUP_DATA` is honored even when the
path is missing, so a misspelled dataset path fails. There is no bundled
ground-truth fallback.

To score one existing antenna trajectory without collecting it:

```bash
python common/evaluate.py --est path/to/est.csv --out results/check \
  --gt "$TEXCUP_DATA/ground_truth.log" --name METHOD
python common/summarize_statistics.py --results results/check
```

## Include output from stopped or failed runs

The ordinary collector rejects incomplete runs. For stopped VINS-Fusion and
IC-GVINS runs, use the native-output finalizer from the ROS environment described
in [RUN_INSTRUCTIONS.md](RUN_INSTRUCTIONS.md). VINS bag recovery additionally
requires ROS Noetic's `rosbag`, `rospy` and `nav_msgs` Python bindings; these are
already present in the method's `benchmark_ros` environment. Scoring saved final
CSVs remains independent of ROS.

```bash
source projects/GVINS/env.sh
python common/finalize_runs.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --run VINS-Fusion=results/VINS-Fusion/latest \
  --run IC-GVINS=results/IC-GVINS/latest
```

The finalizer refuses active runs. It preserves `status.json` and native files,
writes recovery copies and exclusion counts under each run's `finalization/`
directory, and records `finalization.json`. It uses the existing antenna
converters. VINS recovery requires the recorded ENU origin and first usable
global alignment; it excludes output before alignment and invalid rotations.
IC recovery excludes malformed or nonfinite required navigation values.
Unconvertible output becomes unavailable; finite position outliers remain.

The Status column retains `error`, `stopped`, or another failure state. If a
supervisor disappeared before updating a nonterminal status, separate provenance
records `supervisor_exited`. Original runtime status is never rewritten as
completed. An absent native output still produces a 0/4,040 result.

For another stopped method that already has a valid antenna `est.csv`, explicit
collection is available with `--allow-incomplete`:

```bash
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect METHOD=results/METHOD/latest --allow-incomplete
```

## Monitor runs and optionally suspend Ubuntu

The Linux monitor checks the pinned run directories every five minutes, waits
for supervisors and their recorded children to exit, then runs the finalizer.
Use a new state directory for each monitoring session:

```bash
source projects/GVINS/env.sh
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/monitor_runs.py --results results/final \
  --state-dir results/monitor_session --poll-seconds 300 \
  --run VINS-Fusion=results/VINS-Fusion/latest \
  --run IC-GVINS=results/IC-GVINS/latest
```

The monitor resolves `latest` at startup, so subsequent symlink updates cannot
redirect the session. Its `status.json` and `finalize.log` show progress.
Add `--suspend` only when you want Ubuntu suspended after the table is saved.
Suspension uses `systemctl suspend` with the current user's existing permission;
it is withheld if finalization or result verification fails.

## Verification

```bash
python -m unittest discover -s common/tests -v
```

Tests use synthetic GT, temporary run directories and mocked suspension. The
ROS-bag recovery test is skipped when ROS bindings are absent; run the same
suite in `benchmark_ros` for that check. Tests cover common endpoint inclusion,
all-epoch denominators, finite outliers, incomplete-run status, zero output,
native-file preservation and independent recalculation after source files move.
