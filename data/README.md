# Obtain and prepare benchmark inputs

Download TEX-CUP recordings from the [original archive](https://rnl-data.ae.utexas.edu/texcup/).
This repository does not redistribute raw or reduced observations, LORD IMU,
ground truth, or camera recordings. Keep downloaded and generated files in the
ignored `data/tex_cup/` directory or set `TEXCUP_DATA` to another local directory.
No separate estimator checkout is required to obtain source data.

The archive notice requests that users refrain from distributing its data.
Link to the archive and share processing scripts/configuration instead of
committing dataset copies. Downloaders verify HTTPS and preserve existing
files; local manifests record source URLs and preparation details.

| Expected local file | Preparation |
|---|---|
| `asterx4_rover.obs` | Convert rover `asterx4.sbf` with the [RTKLIB recipe](PROCESSING.md) |
| `asterx4_base_1hz.obs` | Convert and downsample the reference receiver's SBF to 1 Hz |
| `lord_imu.log` | Download using [auxiliary preparation](SOURCE_ARCHIVE.md) |
| `ground_truth.log` | Download publisher GT; normalize comment encoding without changing numeric rows |
| `camera-calibration-mono/texcup_port-camchain.yaml` | Local copy of tracked benchmark [mono configuration](../common/config/texcup_mono_calibration.yaml) |
| `camera-calibration-stereo/texcup_stereo-camchain_ovins.yaml` | Local copy of tracked benchmark [stereo configuration](../common/config/texcup_stereo_calibration.yaml) |
| `camera_data/images.h5` | Optional camera download; see [run setup](../RUN_INSTRUCTIONS.md) |
| `brdm1290.19p`, `brdm1290_v304.19p` | Independent broadcast-navigation files; [provenance](NAVIGATION.md) |

Start at the repository root:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python3 data/prepare_auxiliary.py --data-dir "$TEXCUP_DATA"
```

Then follow [GNSS observation processing](PROCESSING.md). Its saved conversion
profiles select GPS 1C/2W, Galileo 1C/7Q and BeiDou 2I, retaining code, phase,
Doppler and SNR. The cloud-validated window contains 4,040 epochs at 1 Hz from
18:09:58 through 19:17:17 GPST on 2019-05-09. These windowed inputs are not
byte-identical replacements for the former full-recording bundled files.

The public rover SBF ends with a truncated navigation block after this window;
the [processing guide](PROCESSING.md) preserves that finding and distinguishes
it from passing observation validation. The base full-file CRC audit passes.
Neither conversion validates positioning or navigation export.

Calibration YAMLs are derived benchmark configuration, rather than source
recordings. `prepare_auxiliary.py` copies them to the local paths required by
existing loaders. The publisher supplies calibration recordings and target
definitions, not these generated camchain YAMLs.

Ground truth is never copied into final results during collection. The shared
timestamp, frame and reference contract remains in
[METHOD_SETUP_GUIDE.md](../METHOD_SETUP_GUIDE.md).
