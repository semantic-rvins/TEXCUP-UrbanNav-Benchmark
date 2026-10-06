# Download and convert the TEX-CUP GNSS recordings

TEX-CUP asks users to refrain from distributing its recordings. Download from
the [original archive](https://rnl-data.ae.utexas.edu/texcup/) and process them
locally. Downloaded SBF, generated RINEX and conversion logs stay outside
version control. The [source archive guide](SOURCE_ARCHIVE.md) covers LORD
IMU, ground truth, camera data and calibration separately.

This recipe validates source observations without running a positioning
solution or rerunning the benchmark. It produces the **evaluation window**,
not byte-identical copies of the previously bundled full-recording files.
Saved benchmark results retain their original provenance; generating these
observations does not demonstrate that those results can be reproduced.

## Conversion configuration

The checked-in profiles are
[texcup_rover_sbf.json](../projects/RTKLIB/config/texcup_rover_sbf.json) and
[texcup_base_sbf.json](../projects/RTKLIB/config/texcup_base_sbf.json).
[sbf_to_rinex.py](../projects/RTKLIB/loaders/sbf_to_rinex.py) reads these profiles,
invokes RTKLIB `convbin`, and records its exact native command with each run.

| Setting | Value |
|---|---|
| Converter | RTKLIB EX 2.5.1, commit `62d4677ed8425a4e2748c6d390b500d1afb493fc` |
| Output format | RINEX 3.04 observations |
| Inclusive bounds | 2019-05-09 **18:09:58–19:17:17 GPST** |
| Equivalent UTC bounds | 2019-05-09 18:09:40–19:16:59 UTC |
| Interval / tolerance | 1 second / 0.005 seconds |
| Expected count | 4,040 observation epochs per receiver |
| GPS signals | 1C, 2W |
| Galileo signals | 1C, 7Q (E1, E5b) |
| BeiDou signals | 2I (B1I) |
| Observation fields | C: pseudorange, L: carrier phase, D: Doppler, S: SNR |
| Antenna selection | Main, SBF index 0; empty receiver options |
| Excluded systems | GLONASS, QZSS, SBAS, NavIC |

The original measurement records contain antenna index 0. The dataset name
ALT1 does not select SBF AUX1/AUX2. Do not add an AUX receiver option.

`convbin` uses command-line flags and **does not read a positioning `.conf`
through `-k`**. The existing
[texcup_rtk_demo5.conf](../projects/RTKLIB/config/texcup_rtk_demo5.conf) belongs
to the separate `rnx2rtkp` positioning workflow in
[RTKLIB/RUN.md](../projects/RTKLIB/RUN.md); it is not the conversion profile.

## Build the pinned converter

Requirements: Linux, Git, GCC supporting C99, Make, Python 3.11 or later,
and verified HTTPS access to GitHub and `rnl-data.ae.utexas.edu`. Qt and
CMake are not required for `convbin`. Run from the repository root; reuse
the existing isolated checkout rather than creating a Git worktree.

```bash
set -euo pipefail
export BENCH="$PWD"
export TEXCUP_DATA="${TEXCUP_DATA:-$BENCH/data/tex_cup}"
export RTK_SOURCE="$BENCH/projects/RTKLIB/upstream/RTKLIB-2.5.1"
mkdir -p "$BENCH/projects/RTKLIB/upstream"
if [ ! -d "$RTK_SOURCE/.git" ]; then
  git clone --depth 1 --branch v2.5.1 \
    https://github.com/rtklibexplorer/RTKLIB.git "$RTK_SOURCE"
fi
test "$(git -C "$RTK_SOURCE" rev-parse HEAD)" = \
  62d4677ed8425a4e2748c6d390b500d1afb493fc
test -z "$(git -C "$RTK_SOURCE" diff --name-only HEAD)"
make -C "$RTK_SOURCE/app/consapp/convbin/gcc" -j4 convbin
export CONVBIN="$RTK_SOURCE/app/consapp/convbin/gcc/convbin"
"$CONVBIN" -?
```

The checks preserve an existing checkout with a different revision or local
source edits instead of replacing it. A repeat build reuses valid outputs.

## Download the original SBF files

| Receiver | Original source URL | Validated size |
|---|---|---:|
| Rover | https://rnl-data.ae.utexas.edu/texcup/2019May09-rover/asterx4/asterx4.sbf | 25,179,136 bytes |
| Base/reference | https://rnl-data.ae.utexas.edu/texcup/2019May09-reference/asterx4/asterx4.sbf | 777,034,292 bytes |

```bash
python3 data/download_texcup.py --receiver rover --download \
  --destination "$TEXCUP_DATA/raw/rover"
python3 data/download_texcup.py --receiver base --download \
  --destination "$TEXCUP_DATA/raw/base"
```

Omit `--download` to inspect each publisher listing. The downloader keeps
TLS verification enabled, checks HTTP Content-Length when supplied, and
validates the download against the recorded profiles before installing
the final file. It writes a local download manifest and preserves existing
files; choose a new destination for another download.

If a download changes, investigate the archive revision before altering a profile.
The publisher also supplies RINEX 2.11, but its rover `SEPT1290.19O` omits
BeiDou, so it cannot replace SBF for this selected signal set.

## Audit source integrity

```bash
mkdir -p "$TEXCUP_DATA/audit"
python3 projects/RTKLIB/loaders/inspect_sbf.py \
  "$TEXCUP_DATA/raw/base/asterx4.sbf" \
  --output "$TEXCUP_DATA/audit/base-sbf-integrity.json"
```

The base audit passes: 4,106,306 complete CRC-valid frames, no skipped
bytes, checksum failures or truncated blocks. Frame counts are not unique
observation-epoch counts.

Run the rover audit separately. **Its expected exit code is 1**, so it must
not be treated as a successful full-file integrity check:

```bash
rover_audit_exit=0
python3 projects/RTKLIB/loaders/inspect_sbf.py \
  "$TEXCUP_DATA/raw/rover/asterx4.sbf" \
  --output "$TEXCUP_DATA/audit/rover-sbf-integrity.json" || rover_audit_exit=$?
printf 'Whole-rover audit exit status: %s (expected 1)\n' "$rover_audit_exit"
test "$rover_audit_exit" -eq 1
python3 - <<'PY'
import json, os
from pathlib import Path
report = json.loads((Path(os.environ['TEXCUP_DATA']) / 'audit/rover-sbf-integrity.json').read_text())
assert report['valid'] is False and report['error_count'] == 2
assert report['statistics']['crc_error_candidates'] == 0
assert report['statistics']['truncated_blocks'] == 1 and report['statistics']['skipped_bytes'] == 52
PY
```

This captures the failing audit status without ending a shell that has
`set -e` enabled, and checks that the report contains the documented defect.
It does not change the audit result to a full-file pass.

The original rover file has 198,244 complete CRC-valid frames followed by
a truncated GPSRawCA navigation block (ID 4017): 52 of its declared 60 bytes,
starting at offset 25,179,084. An independent HTTP byte-range request returned
the same tail with `Content-Range: bytes 25179084-25179135/25179136`, confirming
the defect is in the publisher file. The preceding CRC-valid frame is at
19:25:54 GPST, after this recipe's end; the incomplete frame's unverified
header has the same timestamp. Preserve those bytes and the audit failure.
The selected window is validated independently below; the entire rover
recording is not certified complete.

## Convert and validate

Use empty output directories. The wrapper checks the input size and conversion
settings before conversion and refuses to replace earlier outputs.

```bash
python3 projects/RTKLIB/loaders/sbf_to_rinex.py \
  "$TEXCUP_DATA/raw/rover/asterx4.sbf" --convbin "$CONVBIN" \
  --output-dir "$TEXCUP_DATA/processed/rover"
python3 projects/RTKLIB/loaders/sbf_to_rinex.py \
  "$TEXCUP_DATA/raw/base/asterx4.sbf" --convbin "$CONVBIN" \
  --profile projects/RTKLIB/config/texcup_base_sbf.json \
  --output-dir "$TEXCUP_DATA/processed/base"
```

Each output directory contains the selected observation file, conversion
log, RTKLIB trace, native command, validation report and reproduction manifest.
Both real recordings were converted in the cloud with this recipe and pass
strict validation: 4,040 contiguous 1 Hz epochs with the exact C/L/D/S signal
headers and finite usable code/phase measurements. The validator rejects
LFS pointers, empty files, malformed records, nonfinite measurements,
unexpected signals, duplicate or missing epochs, and incorrect bounds.
Validation concerns observation structure and numerical fields, not
positioning accuracy.

The native rover command is equivalent to:

```bash
"$CONVBIN" -r sbf -v 3.04 -od -os \
  -mask GL1C,GL2W,EL1C,EL7Q,CL2I -y R -y J -y S -y I \
  -ts 2019/05/09 18:09:58 -te 2019/05/09 19:17:18.010 \
  -ti 1 -tt 0.005 -trace 2 \
  -o "$TEXCUP_DATA/processed/rover/read-ahead.obs" \
  "$TEXCUP_DATA/raw/rover/asterx4.sbf"
```

**That native output still requires the wrapper's crop.** This RTKLIB revision
stops before an exact `-te` boundary, and some SBF formats emit an observation
when the next epoch is read. The wrapper reads an extra 1.010 seconds, then
keeps whole records through **19:17:17 GPST inclusive** and corrects header
time bounds. It copies measurement fields and LLI/SSI columns unchanged.
The literal `L` in each `-mask` token is required; bare `G1C`/`E7Q` are silently
ignored. `-od` and `-os` add Doppler and SNR. Do not enable `-halfc` or
`-RCVSTDS`, which alter phase/flag handling or add receiver deviation fields.

The pinned decoder reports rejected navigation messages in the raw inputs.
A diagnostic against the unchanged decoder accounted for every negative
decode result (33 rover, 4,348 base): navigation/SBAS correction messages,
with **zero rejected observation messages** in the selected interval.
The conversion's `E=` counter is an error count, not a Galileo satellite
count. This recipe does not validate navigation-file generation or replace
the separately attributed `brdm1290` inputs.

Comparison with the publisher rover RINEX exercised 55,224 shared GPS/Galileo
satellite records and 377,525 common finite measurements. Common SNR values
agree exactly; other common values differ by at most 0.001 in their printed
units. RTKLIB retains some GPS phase measurements with unresolved half-cycle
ambiguity (LLI bit 2) that the publisher converter blanks. Satellite rows,
LLI/SSI and headers can therefore differ. BeiDou has no publisher-RINEX
comparison, and whole-file RINEX contents also vary with generation dates and
local input paths.

For method scripts that expect observations directly under `TEXCUP_DATA`,
install the generated files only after checking that they do not already
exist:

```bash
test ! -e "$TEXCUP_DATA/asterx4_rover.obs"
test ! -e "$TEXCUP_DATA/asterx4_base_1hz.obs"
cp "$TEXCUP_DATA/processed/rover/asterx4_rover.obs" "$TEXCUP_DATA/asterx4_rover.obs"
cp "$TEXCUP_DATA/processed/base/asterx4_base_1hz.obs" "$TEXCUP_DATA/asterx4_base_1hz.obs"
```

The ancillary [source archive guide](SOURCE_ARCHIVE.md) explains the remaining
inputs and the frame/trajectory checks needed before evaluating another run.
Cropping this evaluation window does not recreate the original full-recording
observation files.

Helper regression checks do not require downloaded data or RTKLIB:

```bash
python3 -m unittest discover -s projects/RTKLIB/tests -v
```
