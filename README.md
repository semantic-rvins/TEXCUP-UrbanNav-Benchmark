# TEX-CUP Urban-Navigation Benchmark

[TEX-CUP](https://radionavlab.ae.utexas.edu/texcup-desc/) (The University of Texas
Challenge for Urban Positioning) is a public dataset for evaluating multi-sensor
localization and navigation in Austin, Texas. This benchmark uses the
selected 2019-05-09 trajectory: approximately **20 km**, with **about half of the driving
in deep urban conditions**. The dataset provides synchronized raw Multi-GNSS
observations (covering both rover and base), IMU measurements, stereo camera imagery and post-processed ground
truth, making it suitable for validating tightly coupled GNSS–visual–inertial
localization and navigation systems.

This repository provides a common evaluation workflow and eleven saved results
in the [current comparison table](results/final/statistics.md). It contains data
converters, calibration, required source patches, run instructions and scoring
scripts. Download TEX-CUP recordings directly from the
[publisher's archive](https://rnl-data.ae.utexas.edu/texcup/) and prepare them
locally using [data instructions](data/README.md). Source recordings and
ground-truth copies are not bundled. The run guides explain how to obtain
each method's public source code.

Git LFS remains in use for the independent broadcast-navigation inputs and
supplied SeA-RVINS result archives. TEX-CUP SBF, IMU, ground truth and images
come from the publisher; [run setup](RUN_INSTRUCTIONS.md#1-set-paths-and-obtain-the-data)
and the [tested RTKLIB conversion recipe](data/PROCESSING.md) give download,
sampling, signal selection and configuration commands.

| Method | Inputs | Included result | Instructions |
|---|---|---|---|
| RTKLIB-EX | Rover/base GNSS | Completed | [Run](projects/RTKLIB/RUN.md) |
| GICI-RTK | Rover/base GNSS; no IMU or camera | Completed | [Run](projects/GICI-LIB/RUN.md#gnss-only-rtk) |
| GICI-RRR | Rover/base GNSS, IMU, mono camera | Completed | [Run](projects/GICI-LIB/RUN.md#rtk-imu-camera-rrr) |
| VINS-Fusion | RTKLIB-EX positions, IMU, stereo cameras | Completed | [Run](projects/VINS-Fusion/RUN.md) |
| IC-GVINS | RTKLIB-EX positions, IMU, mono camera | Pre-divergence partial result | [Run and limitations](projects/IC-GVINS/RUN.md) |
| GVINS | Rover raw GNSS, IMU, mono camera | Completed | [Run](projects/GVINS/RUN.md) |
| InGVIO | Rover raw GNSS, IMU, mono camera | Collected trajectory | [Run and provenance limits](projects/InGVIO/RUN.md) |
| OKVIS2-X | RTKLIB-EX positions, IMU, stereo cameras | Collected final-BA trajectory | [Run](projects/OKVIS2-X/RUN.md) |
| [SeA-RVINS (batch)](https://github.com/semantic-rvins/semantic-rvins.github.io) | Supplied estimator records | Imported | [Import](results/SeA-RVINS/README.md) |
| [SeA-RVINS (latent)](https://github.com/semantic-rvins/semantic-rvins.github.io) | Supplied estimator records | Imported | [Import](results/SeA-RVINS/README.md) |
| [SeA-RVINS (scalar)](https://github.com/semantic-rvins/semantic-rvins.github.io) | Supplied estimator records | Imported | [Import](results/SeA-RVINS/README.md) |

All saved trajectories can be rescored. The method guides distinguish replay
instructions from the configuration evidence available for each supplied result.

- [Run setup](RUN_INSTRUCTIONS.md): dataset, environments and method run guides.
- [Data preparation](data/README.md): archive downloads, local processing and navigation provenance.
- [Data and calibration](METHOD_SETUP_GUIDE.md): timestamps, IMU axes, images,
  intrinsics, extrinsics and antenna reference points.
- [Required fixes](BUG_FIXES.md): exact active patches and their scope.
- [Statistics](STATISTICS.md): regenerate the table or collect a new result.
- [Error figures](figures/README.md): regenerate horizontal-error and CDF plots,
  including the [version without batch](figures/fig_error_cdf_only_no_batch.pdf).

To recalculate the saved results without building an estimator or downloading
camera images, first prepare local ground truth from the publisher:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r environments/statistics-requirements.txt
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python data/prepare_auxiliary.py --data-dir "$TEXCUP_DATA"
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
```

[comparison.json](results/final/comparison.json) fixes the methods, labels and
row order. RTKLIB-EX retains the internal filename prefix `RTKLIB`.

Evaluation uses TEX-CUP antenna 2 / ALT1 ground truth over
**18:09:40–19:16:59 UTC on 2019-05-09**, inclusive: **4,040 epochs**.
Availability and horizontal threshold percentages use all epochs. IC-GVINS
retains the pre-divergence prefix through **18:34:59 UTC**, with
**1,502/4,040 solved epochs (37.18%)**; its distance metrics describe that
partial interval. The complete saved output is preserved separately from this
table selection. Processing completion does not guarantee accuracy.

**Ground-truth reference note:** Post-processing IMU-to-antenna lever-arm
estimation and calibration using the ground-truth data verified that the
benchmark's `ground_truth.log` samples refer to **antenna 2 / ALT1**. The
publisher download has the same numeric samples; preparation and the
archive's frame description are discussed in [source provenance](data/SOURCE_ARCHIVE.md).
This is the antenna labeled
ALT1 in the [TEX-CUP sensor CAD diagram (Fig. 6)](https://radionavlab.ae.utexas.edu/wp-content/uploads/texcup.pdf).
