# SeA-RVINS supplied results

The supplied [SeA-RVINS](https://github.com/semantic-rvins/semantic-rvins.github.io)
archives represent three variants: **SeA-RVINS (batch)**,
**SeA-RVINS (latent)**, and **SeA-RVINS (scalar)**. All enter the comparison.

These are imported saved results, not estimator replays launched by this
benchmark repository. The archive names identify batch/latent/scalar robust variants.
They do not contain the complete estimator configuration or source revision,
so this adapter reproduces conversion and scoring rather than the original run.

The distributed archives use a sanitized record format. Direct ground-truth
records, including position, velocity and attitude, have been removed. Saved
estimates, signed error vectors, timestamps and available processing diagnostics
are retained. **An estimated position and its signed error vector can still
reconstruct the corresponding ground-truth position.** These archives therefore
remove explicit ground-truth fields; they do not make those positions
unrecoverable.

The [archive cleanup tool](../../projects/SeA-RVINS/loaders/sanitize_archives.py)
checks that retained timestamps, estimates and signed errors stay unchanged.
The normal import and scoring commands below use local ground truth.

## Download the saved records

The three `.7z` archives are stored with Git LFS. After cloning or pulling:

```bash
git lfs install
git lfs pull --include="results/SeA-RVINS/*.7z"
```

| Variant | Supplied archive |
|---|---|
| SeA-RVINS (batch) | `results_09112026_batch_robust.7z` |
| SeA-RVINS (latent) | `results_09082026_latent_robust.7z` |
| SeA-RVINS (scalar) | `results_09092026_scalar_robust.7z` |

## Convert the records and update the table

Use the [Python statistics environment](../../STATISTICS.md), plus the `7z`
command from p7zip (or `7zz` from 7-Zip). First
[download and prepare the TEX-CUP ground truth](../../data/README.md).
Run from the benchmark root:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python projects/SeA-RVINS/loaders/import_results.py \
  --archive results/SeA-RVINS/results_09112026_batch_robust.7z \
  --variant batch --out results/SeA-RVINS/converted/batch \
  --gt "$TEXCUP_DATA/ground_truth.log"
python projects/SeA-RVINS/loaders/import_results.py \
  --archive results/SeA-RVINS/results_09082026_latent_robust.7z \
  --variant latent --out results/SeA-RVINS/converted/latent \
  --gt "$TEXCUP_DATA/ground_truth.log"
python projects/SeA-RVINS/loaders/import_results.py \
  --archive results/SeA-RVINS/results_09092026_scalar_robust.7z \
  --variant scalar --out results/SeA-RVINS/converted/scalar \
  --gt "$TEXCUP_DATA/ground_truth.log"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" \
  --collect SeA-RVINS-batch-robust=results/SeA-RVINS/converted/batch \
  --collect SeA-RVINS-latent-robust=results/SeA-RVINS/converted/latent \
  --collect SeA-RVINS-scalar-robust=results/SeA-RVINS/converted/scalar
python common/plot_error_cdf.py --results results/final \
  --out-prefix figures/fig_error_cdf
python common/plot_error_cdf.py --cdf-only
```

The converted working directories are ignored. Their antenna trajectories and
conversion provenance are copied into `results/final/SeA-RVINS-*.{est.csv,source.json}`.
The small common trajectories are committed normally, so statistics can be
recalculated using the locally prepared ground truth without downloading the
SeA-RVINS LFS archives. Figures can also be regenerated from saved error grids.

Check the importer with `python -m unittest discover -s projects/SeA-RVINS/tests -v`.

## Position and time contract

The importer reads `estimated.antenna_pos_enu_m`, which already includes the
estimated-attitude IMU-to-antenna translation. It does not apply another lever.
It converts this fixed ENU position to ECEF using the recorded framework's base
origin `[-742080.4125,-5462031.7412,3198339.6909]` metres. The importer checks signed
errors against the independently downloaded ground truth in this fixed frame;
no alignment is fitted to GT. Estimate conversion uses the saved antenna
position and fixed origin, independently of the ground truth.

Record UTC timestamps are checked against their GPS timestamps with the
18-second offset. GNSS epochs supply the trajectory; camera-only records do
not become additional GNSS solutions. The common evaluator compares ALT1
antenna positions over **18:09:40–19:16:59 UTC**, retaining all 4,040 epochs in
percentage denominators and all finite errors in distance statistics.

Each conversion records counts and frame/time validation
and the limits of available configuration provenance. The batch, latent,
and scalar labels do not establish a controlled causal comparison
without the missing full configurations.

The batch archive contains exactly 4,040 GNSS epochs, all in the common window,
with no duplicate epochs or UTC/GPST mismatches. Its timestamps and saved
antenna/error identity against local ground truth pass the same importer.

The latent and scalar archives each contain 4,101 GNSS epochs: 4,040 in the common window
and 61 after it, with no duplicate epochs or UTC/GPST mismatches. Both contain
40,162 vision logs, including 40,156 with positive inlier counts. Their saved
visibility snapshots report the LoD1 service disabled. These diagnostics
establish observed processing, rather than a complete configuration manifest.
