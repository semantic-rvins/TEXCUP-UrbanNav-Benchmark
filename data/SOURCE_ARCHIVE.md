# Obtain TEX-CUP inputs from the publisher

The TEX-CUP notice requests that users refrain from distributing its data.
Download the recordings from the [University of Texas Radio Navigation
Laboratory archive](https://rnl-data.ae.utexas.edu/texcup/) into your private
local dataset directory. This repository provides processing instructions,
benchmark configuration and results instead of redistributing
source recordings or the converted observations.

The archive [readme.txt](https://rnl-data.ae.utexas.edu/texcup/readme.txt)
describes the sensors and source formats. These are the original locations
for the 2019-05-09 recording:

| Input | Publisher location | Local consumer path under `TEXCUP_DATA` |
|---|---|---|
| Rover GNSS | [rover AsteRx4](https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/asterx4/asterx4.sbf) | `asterx4_rover.obs`, after [RTKLIB conversion](PROCESSING.md) |
| Base GNSS | [reference AsteRx4](https://rnl-data.ae.utexas.edu/texcup/2019May09-reference/asterx4/asterx4.sbf) | `asterx4_base_1hz.obs`, after [RTKLIB conversion](PROCESSING.md) |
| LORD IMU | [original LORD log](https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/lord/lord_imu.log) | `lord_imu.log`, unchanged publisher bytes |
| Ground truth | [ATLANS-C trajectory](https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/atlans/ground_truth.log) | `ground_truth.log`, comments transcoded to UTF-8 |
| Camera images, optional | [stereo images](https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/camera/images.h5) | `camera_data/images.h5` |
| Mechanical calibration | [sensor CAD drawing](https://rnl-data.ae.utexas.edu/texcup/extrinsics/extrinsic_calibration.pdf) | Reference material; no automatic download |

The two `brdm1290` navigation files are separate broadcast-navigation inputs;
their attribution and handling are described in [the data inventory](README.md).

## Prepare IMU, ground truth and camera configuration

From the repository root, use Python 3 and the standard library:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python3 data/prepare_auxiliary.py --data-dir "$TEXCUP_DATA"
```

The helper downloads about 55 MB, validates downloads automatically, retains
original files under `raw/auxiliary/`, and writes `auxiliary-manifest.json`
with source URLs and preparation details.
It copies benchmark camera profiles from `common/config/` to the legacy
paths expected by the bag validators. Camera images are optional and are
not downloaded by this command.

The helper reuses matching files and refuses to overwrite a differing file.
If you have older local inputs, preserve them and choose a new `TEXCUP_DATA`
directory, or move
them aside yourself before running it. With the default dataset location,
these downloads and generated copies are ignored by Git.

Then follow [the GNSS processing instructions](PROCESSING.md) to convert
the raw SBF recordings with the saved RTKLIB conversion configuration.
For visual methods, separately obtain the approximately 75 GB `images.h5`
from the publisher camera link and place it at the table's local path.

To rescore saved trajectories after preparing ground truth:

```bash
# Use the statistics environment described in RUN_INSTRUCTIONS.md.
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log"
```

The scoring tool reads this private local ground-truth file directly. Any
legacy `results/final/ground_truth.log` copy is ignored by Git and must not
be committed.

## Ground-truth preparation and reference point

The publisher's trajectory contains the same numerical samples as the
historical scoring input; differences are confined to comments. The helper
standardizes comment encoding from Latin-1 to UTF-8 without changing
coordinates, timestamps or attitude values. The evaluator also accepts the
publisher's Latin-1 file directly. No pose correction or lever-arm translation
is applied. Scoring uses **4,040 epochs from 18:09:40 through 19:16:59 UTC**,
inclusive.

There is a provenance discrepancy to retain explicitly: the publisher
README describes this file as the **ATLANS-C IMU frame origin**. The
benchmark's documented interpretation is **antenna 2 / ALT1**, based on
its prior empirical lever-arm analysis; see [the reference-point
contract](../METHOD_SETUP_GUIDE.md). Matching numerical samples does not
independently establish the physical reference point. These instructions
preserve the scoring input without introducing a reference-point correction.

## LORD IMU and time handling

The publisher LORD file and historical benchmark input contain the same
**500,044 numerical rows**; differences are confined to comments. The helper
preserves the publisher file. It contains GPS week/seconds and
acceleration/angular-rate samples; the existing method converters apply
the [timestamp and IMU-axis rules](../METHOD_SETUP_GUIDE.md).

## Camera calibration profiles

The benchmark profiles are retained as configuration in
[common/config/texcup_mono_calibration.yaml](../common/config/texcup_mono_calibration.yaml)
and
[common/config/texcup_stereo_calibration.yaml](../common/config/texcup_stereo_calibration.yaml).
They contain the calibration parameters consumed by the existing method
configs. The helper creates private compatibility copies at
`camera-calibration-mono/texcup_port-camchain.yaml` and
`camera-calibration-stereo/texcup_stereo-camchain_ovins.yaml` under
`TEXCUP_DATA`.

These named camchain profiles are not files offered by the publisher's
listed archive. The [mono calibration recording
directory](https://rnl-data.ae.utexas.edu/texcup/camera-calibration-mono/camera/)
offers `images.h5` and `aprilgrid.yaml`; the [stereo calibration recording
directory](https://rnl-data.ae.utexas.edu/texcup/camera-calibration-stereo/camera/)
offers `images.h5` and `checkerboard.yaml`. Those are calibration recordings
and pattern specifications, rather than ready-made benchmark camchain
results. Re-estimating calibration from those recordings is a separate
workflow and may produce different parameters. Neither calibration
recording is needed to use the retained benchmark configuration.
