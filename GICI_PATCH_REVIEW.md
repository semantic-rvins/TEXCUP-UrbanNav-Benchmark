# GICI source patches used for the comparison

Both selected GICI methods use upstream revision
`f2b8579f4fab9dff2b0950b11cc04aefe6d48e3a`, with separate source trees:

| Method | Inputs | Active patches |
|---|---|---|
| GICI-RTK | Rover/base GNSS | None; unpatched upstream source |
| GICI-RRR | Rover/base GNSS, IMU and mono camera | 0001, then 0002 |

The saved GICI-RTK source diff is empty. GICI-RRR includes software guards and
feature-admission changes described below. Follow the
[GICI run guide](projects/GICI-LIB/RUN.md) for matching builds and configurations.

## GICI-RRR: guards and feature admission

| Change | Classification |
|---|---|
| Return an invalid/default result when no completed estimator state exists | Software guard against invalid container access |
| Require retained GNSS/ambiguity history before accessing previous measurements or constructing relative factors | Software guard; operations requiring missing history are skipped |
| Return immediately when there are no reference features | Software guard against empty OpenCV input |
| With fewer than eight tracks, bypass RANSAC and accept remaining tracks | **Changes feature acceptance** by omitting the geometric outlier test |
| Reject visual initialization with fewer than eight matches or an empty/non-3×3 essential matrix | Matrix-shape protection and an initialization-admission policy |
| Log the actual initial ENU origin and input EOF | Diagnostics/output-conversion support |

The first five rows come from
[0001-texcup-robustness-guards.patch](projects/GICI-LIB/patches/0001-texcup-robustness-guards.patch);
the last comes from
[0002-record-origin-and-eof.patch](projects/GICI-LIB/patches/0002-record-origin-and-eof.patch).
The few-track fallback has no dedicated counter in the saved run, so its
frequency and accuracy impact are unknown. RRR should therefore retain an
explicit feature-admission caveat.

## Configuration and data contract

Solver iteration limits, GNSS outlier thresholds, ambiguity-resolution
settings and the RRR three-keyframe window match the pinned upstream samples.
The benchmark supplies its selected signals and TEX-CUP sensor calibration/noise
values. Removing the feature-admission changes would require a separate replay.

Both methods start at 18:09:58 GPST, corresponding to 18:09:40 UTC, and exclude
GLONASS. GICI-RTK outputs antenna positions directly. GICI-RRR rotates LORD
data into RFU and adds the fixed IMU-to-ALT1 lever using its logged initial ENU
origin and estimated attitude. RRR consumes native 2048×732 mono images with
the documented intrinsics/extrinsics. These are data preparation and output
conversion settings.

See [calibration](METHOD_SETUP_GUIDE.md) and the saved
[RTK provenance](results/final/GICI-RTK.source.json) and
[RRR provenance](results/final/GICI-RRR.source.json). The compact metadata
summarize the source revision, configuration and output conversion used for
the comparison.
