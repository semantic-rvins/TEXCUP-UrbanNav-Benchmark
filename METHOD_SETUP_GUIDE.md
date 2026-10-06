# Data preparation and calibration

This is the input/output contract for the executed methods. Follow the
[run guides](RUN_INSTRUCTIONS.md) for commands and [BUG_FIXES.md](BUG_FIXES.md)
for required source patches. [Data setup](data/README.md) downloads TEX-CUP
recordings from the publisher and prepares them locally, including
[RTKLIB SBF conversion](data/PROCESSING.md). The
[mono](common/config/texcup_mono_calibration.yaml) and
[stereo](common/config/texcup_stereo_calibration.yaml) YAMLs are benchmark
calibration profiles retained as reproducibility settings; the preparation
helper copies them to the ignored legacy data paths used by loaders. Method
configurations below contain the converted parameters, and runtime YAML
snapshots record the effective settings.

## Shared time, images and reference point

| Item | Required value |
|---|---|
| Dataset | TEX-CUP, 2019-05-09 |
| Start | 18:09:40 UTC = 18:09:58 GPST; UTC Unix1557425380 |
| Evaluation end | 19:16:59 UTC inclusive; 4,040 epochs |
| Images | HDF5 port=left, star=right; unrectified grayscale2048×732, about10Hz |
| IMU | LORD rates: acceleration m/s², angular rate rad/s, about100Hz |
| Ground truth | `ground_truth.log` is antenna2 / ALT1; do not translate GT |
| GNSS systems | GPS1C/2W, Galileo1C/7Q, BeiDou2I; GLONASS excluded |
| Base ECEF | `[-742080.4125,-5462031.7412,3198339.6909]`m from RINEX header |

Ground truth is downloaded separately; the preparation helper standardizes
its comment encoding while preserving numeric rows. The evaluator also accepts
the publisher's Latin-1 file directly. The
[publisher source guide](data/SOURCE_ARCHIVE.md) documents the archive
reference-frame description and the benchmark's interpretation.
The established benchmark reference remains antenna 2 / ALT1.

RINEX and raw LORD time are GPST. Image `tstamp` is already UTC. Subtract18s
once when converting LORD/RTKLIB time to UTC. ROS bag timestamps are UTC.
GVINS GNSS payloads retain GPST, so set `gnss_local_time_diff:18.0` with
`gnss_local_online_sync:0`. IC-GVINS converts ROS UTC to GPST internally.
A short IMU prelude/tail supports integration; scoring still uses the same
window. RRR bounds GNSS at the evaluation end and IMU one second later.

## IMU frame and lever arm

Raw LORD axes are **x left, y forward, z down**. Rotate both acceleration and
angular rate into the frame expected by the implementation:

| Consumer | Supplied frame and raw conversion | IMU→ALT1 lever in that frame |
|---|---|---|
| GICI-RRR, VINS-Fusion, GVINS, InGVIO, OKVIS2-X | Right/forward/up (RFU): `(-x,y,-z)` | `[-0.610,-0.052,+0.010]`m |
| IC-GVINS | Forward/right/down (FRD): `(y,-x,z)` | `[-0.052,-0.610,-0.010]`m |
| RTKLIB-EX, GICI-RTK | No IMU input | No output lever needed |

Feed rates, not integrated increments. Keep IMU noise and bias random-walk
entries in each method's linked YAML: those parameters have implementation
specific meanings/units and are not interchangeable between estimators.

## Native camera calibration

Intrinsics are `[fx,fy,cx,cy]` pixels; radtan distortion is `[k1,k2,p1,p2]`.
The mono and stereo calibrations are different profiles of the same cameras.

| Profile | Intrinsics | Distortion |
|---|---|---|
| Mono port (GICI-RRR, GVINS, IC-GVINS, InGVIO) | `[1355.07090197,1360.08043642,1013.57243982,330.36528878]` | `[-0.04362915,0.03696032,0.00059401,0.00057094]` |
| Stereo port (VINS-Fusion, OKVIS2-X) | `[1373.879,1378.680,991.935,333.389]` | `[-0.02918,0.03616,0.00357,-0.00408]` |
| Stereo star (VINS-Fusion, OKVIS2-X) | `[1319.512,1324.766,1021.862,345.805]` | `[-0.03596,0.03498,0.00372,-0.00253]` |

Use `--downscale 1` and matching native YAML dimensions. Regenerate old
half-resolution inputs. Intrinsics/dimensions scale with resolution;
distortion and rigid extrinsics do not. These inputs are unrectified, so an
additional image-rectification rotation must not enter the raw camera extrinsic.

`T_body_cam` means `p_body = R_body_cam * p_cam + t_body_cam`. Camera axes are
right/down/forward. Camera→RFU for the port camera is:

```text
R = [[1,0,0], [0,0,1], [0,-1,0]]
t = [-0.303,0.011,0.033] metres
```

IC-GVINS uses the equivalent camera→FRD quaternion **xyzw**`[0.5,0.5,0.5,0.5]`
and translation`[0.011,-0.303,-0.033]`m. VINS-Fusion's star transform is
`T_body_cam0 * inverse(T_cam1_cam0)`, with about0.498m baseline; keep the full
calibrated matrix, including its small off-axis terms, in the stereo YAML.

## Method-specific preparation and output

| Method | Active config | Adapter behavior |
|---|---|---|
| RTKLIB-EX | [texcup_rtk_demo5.conf](projects/RTKLIB/config/texcup_rtk_demo5.conf) | Dual-frequency GNSS, navigation mask41; output antenna ECEF/GPST→UTC |
| GICI-RTK | [RTK YAML](projects/GICI-LIB/config/gici_rtk_dualfreq_post.yaml) | Cut rover/base at18:09:58GPST; antenna NMEA→ECEF |
| GICI-RRR | [RRR YAML](projects/GICI-LIB/config/gici_rrr_mono_dualfreq_post.yaml) | RFU IMU, mono image pack; fixed GNSS/camera extrinsics; body→ALT1 output |
| VINS-Fusion | [stereo YAML](projects/VINS-Fusion/config/texcup_stereo_config.yaml) | RTKLIB positions plus RFU stereo; online camera extrinsics, fixed delay0 |
| IC-GVINS | [IC YAML](projects/IC-GVINS/config/icgvins_texcup.yaml) | RTKLIB Q1/Q2 positions, FRD mono; online camera extrinsics and delay, initial delay0 |
| GVINS | [GVINS YAML](projects/GVINS/config/gvins_texcup.yaml) | Rover-only supported first-band code/Doppler; mono RFU; online camera extrinsics, fixed delay0 |
| InGVIO | [InGVIO YAML](projects/InGVIO/config/ingvio_mono_texcup.yaml) | Rover raw GNSS, mono RFU; fixed GNSS time offset -18s; ECEF body output translated to ALT1 |
| OKVIS2-X | [OKVIS2-X YAML](projects/OKVIS2-X/config/okvis2_texcup.yaml) | RTKLIB-EX Q1/Q2 antenna positions in fixed ENU, stereo RFU; final-BA global antenna output |

**GICI-RRR:** `body_to_imu_rotation:[0,0,0]`; fixed
`gnss_extrinsics:[-0.610,-0.052,0.010]`. Disable GLONASS under
`gnss_estimator_base_options.gnss_common.system_exclude`, not only under AR.
Output conversion uses the logged initial SPP ENU origin and full estimated
ESA attitude. GGA altitude plus geoid separation gives ellipsoidal height.
The runners supply the exact origin from that run's initialization log.

**GVINS:** keep GPST in GNSS payloads and UTC in bag stamps. Apply the source
timing patch before building; YAML alone is insufficient. GNSS factors assume
a colocated antenna. The extractor adds ALT1 using attitude recovered from
camera orientation and initial extrinsics; identity fallback when attitude
association fails remains an output approximation. Existing GNSS alignment,
pairing and reset-policy changes are disclosed in the bug-fix guide.

**VINS-Fusion:** port/star form a stereo pair at the port timestamp. Tracking
receives every pair; upstream estimates alternate pairs. A small GPS sidecar
interpolates RTKLIB positions to those admitted image timestamps to satisfy
±10ms global-fusion association. The original bag GPS topic is excluded.
Input lever removal uses course yaw with zero roll/pitch, an approximation;
output restores the lever with estimated attitude. `position_covariance[0]`
contains **sigma**, 0.05m for Q1 and0.75m otherwise, because this consumer
uses it directly as a residual divisor. These are chosen weights. Output
uses live global odometry in the logged fixed ENU frame, translated to ALT1;
pre-alignment output is unavailable. See [the detailed contract](VINS_FUSION_DATA_REVIEW.md).

**IC-GVINS:** feed antenna positions unchanged; its GNSS factor models the lever.
The loader retains Q1/Q2 and excludes Q4. NavSatFix covariance stores **variance**:
fixed sigma horizontal/vertical0.05/0.10m, float0.50/1.00m. The extractor
converts IMU output with estimated FRD→current-local-NED attitude and adds
ALT1 exactly once. Native initialization and GNSS outlier policies are retained.

**InGVIO:** the two source patches add build compatibility and ECEF pose output.
The extractor adds the RFU lever using the published estimated attitude.
The committed YAML uses `gnss_chi2_test: 1` and `visual_noise: 0.12`; these are
estimator settings beyond data calibration. The collected trajectory has no
effective configuration snapshot, so its exact replay settings cannot be
confirmed from the saved metadata. See [the run guide](projects/InGVIO/RUN.md)
and [patch/configuration notes](projects/InGVIO/BUGFIX_NOTES.md).

**OKVIS2-X:** use the saved `results/final/RTKLIB.est.csv`; its times are already
UTC. Keep Q1/Q2 and feed unshifted ALT1 positions, with sigma E/N/U
0.05/0.05/0.10m for Q1 and 0.50/0.50/1.00m for Q2. The converter uses the first
GT position as a fixed ENU coordinate origin, without fitting a trajectory
alignment. The estimator models `r_SA: [-0.610,-0.052,0.010]` internally.
Its final-BA global output already contains the antenna lever; the output
converter only maps fixed ENU back to ECEF. See [the run guide](projects/OKVIS2-X/RUN.md).

## Position errors

All common trajectories contain **ALT1 antenna ECEF** and UTC seconds of day.
For a body output, `p_antenna = p_body + R_world_body * lever_body`; the attitude
must correspond to the output epoch. Frame conversion does not align to GT.
[STATISTICS.md](STATISTICS.md) defines time association, fixed ENU error,
all-epoch denominators and missing-output handling.
