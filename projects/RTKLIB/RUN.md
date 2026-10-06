# Run RTKLIB-EX 2.5.1

Run from the benchmark root in Bash, with `BENCH` and `TEXCUP_DATA` set as in
[run setup](../../RUN_INSTRUCTIONS.md). No source patch, IMU, camera calibration
or antenna lever correction is required. The output already represents ALT1.

## 1. Obtain the pinned public source

For a fresh checkout:

```bash
mkdir -p projects/RTKLIB/upstream
git clone --branch v2.5.1 https://github.com/rtklibexplorer/RTKLIB.git \
  projects/RTKLIB/upstream/RTKLIB-2.5.1
git -C projects/RTKLIB/upstream/RTKLIB-2.5.1 checkout --detach \
  62d4677ed8425a4e2748c6d390b500d1afb493fc
export RTK_SOURCE="$BENCH/projects/RTKLIB/upstream/RTKLIB-2.5.1"
export RTK_OUT="$BENCH/results/RTKLIB"
mkdir -p "$RTK_OUT"
```

The clean public clone at this revision was independently built with the
commands below.

## 2. Prepare observations and build

TEX-CUP observations are downloaded and generated locally, rather than bundled.
Follow [the SBF processing guide](../../data/PROCESSING.md) to download the
original rover/base recordings and convert them with the pinned `convbin`.
The profiles
[rover](config/texcup_rover_sbf.json) and [base](config/texcup_base_sbf.json)
retain GPS 1C/2W, Galileo 1C/7Q and BeiDou 2I, with code, phase, Doppler and SNR
fields, over **18:09:58–19:17:17 GPST** at 1 Hz. Each output has 4,040 epochs.
Do not strip these already selected observations again. Use the output names
`asterx4_rover.obs` and `asterx4_base_1hz.obs` under `$TEXCUP_DATA`.
Prepare ground truth using [data setup](../../data/README.md). Broadcast
navigation is retained separately; see its source attribution there.

The JSON profiles configure the SBF conversion wrapper. The positioning
`.conf` below is used by `rnx2rtkp` and is not a `convbin` conversion config.

With a C/C++ compiler, Make and CMake >=3.16:

```bash
cmake -S "$RTK_SOURCE" -B "$RTK_SOURCE/build-benchmark" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_QT=TRUE \
  > "$RTK_OUT/build.log" 2>&1
cmake --build "$RTK_SOURCE/build-benchmark" --target rnx2rtkp --parallel 4 \
  >> "$RTK_OUT/build.log" 2>&1
"$RTK_SOURCE/bin/rnx2rtkp" --version
ldd "$RTK_SOURCE/bin/rnx2rtkp"
```

Expect `rnx2rtkp RTKLIB EX 2.5.1`; its shared library must resolve to this
source tree's `lib/`. The binary is in source `bin/`.

## 3. Run GNSS processing

```bash
cp projects/RTKLIB/config/texcup_rtk_demo5.conf "$RTK_OUT/texcup_rtk_demo5.conf"
"$RTK_SOURCE/bin/rnx2rtkp" \
  -k "$RTK_OUT/texcup_rtk_demo5.conf" \
  -ts 2019/05/09 18:09:58 -te 2019/05/09 19:17:17 \
  -o "$RTK_OUT/rtklib_demo5.pos" \
  "$TEXCUP_DATA/asterx4_rover.obs" \
  "$TEXCUP_DATA/asterx4_base_1hz.obs" \
  "$TEXCUP_DATA/brdm1290.19p" > "$RTK_OUT/run.log" 2>&1
```

Require exit code 0. Start and end times are **GPST**, 18 seconds ahead of UTC;
scoring ends at 19:16:59 UTC. The profile
uses kinematic dual-frequency RTK, `pos1-navsys=41` (GPS/Galileo/BeiDou),
RINEX-header base ECEF `[-742080.4125,-5462031.7412,3198339.6909]` metres,
ECEF/GPST solution output, and residual status output. Remaining settings use
RTKLIB-EX defaults, including fix-and-hold with AR ratio 3. Reusing `RTK_OUT`
replaces the same output filenames; use another directory to retain an attempt.

## 4. Convert, score and collect

Activate the [statistics environment](../../STATISTICS.md), then:

```bash
python projects/RTKLIB/loaders/pos_to_csv.py \
  "$RTK_OUT/rtklib_demo5.pos" "$RTK_OUT/est.csv" "$RTK_OUT/eval_extra.json"
python common/evaluate.py --est "$RTK_OUT/est.csv" --out "$RTK_OUT" \
  --gt "$TEXCUP_DATA/ground_truth.log" --name RTKLIB \
  --cut 18:09:40 --end 19:16:59
python common/calculate_statistics.py --results results/final \
  --gt "$TEXCUP_DATA/ground_truth.log" --collect "RTKLIB=$RTK_OUT"
```

The converter subtracts 18 seconds and preserves antenna ECEF. The saved
historical run contains 4,557 native solutions and solves all 4,040 evaluation epochs.
It used longer observations; the validated conversion above contains only the
evaluation window. A fresh replay from it is not claimed to reproduce the
saved trajectory or its solution count.
Keep `rtklib_demo5.pos` for IC-GVINS and `est.csv` for VINS-Fusion input generation.
`eval_extra.json` describes all native solutions, while the central table uses
only the shared evaluation window. RTKLIB runs directly and has no live
`status.json`; successful processing, conversion and evaluation establish its
completion for collection.
