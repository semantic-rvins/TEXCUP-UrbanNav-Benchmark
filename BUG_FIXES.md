# Required source fixes and baseline scope

Upstream clones are ignored. **Apply the saved patches to the pinned revision
and rebuild on every fresh checkout.** A YAML change alone does not implement
a C++ timing or rotation fix. The [method run guides](RUN_INSTRUCTIONS.md)
provide clone, patch and build commands.

| Method | Required active patches | Scope |
|---|---|---|
| RTKLIB-EX | None | Public v2.5.1 source |
| GICI-RTK | None | GNSS-only RTK using unpatched pinned upstream source |
| GICI-RRR | `0001-texcup-robustness-guards.patch`, then `0002-record-origin-and-eof.patch` | Software guards, diagnostics **and feature-admission changes** |
| VINS-Fusion | Three files in [patches/series](projects/VINS-Fusion/patches/series) | Build/origin diagnostics and unit-rotation software fix |
| IC-GVINS | Patches `03` through `08`, in order | Diagnostics/shutdown, interval/index guards, state/prior lifetime, feature-grid bounds, Eigen temporary lifetime |
| GVINS | `gnss_comm.patch` in gnss_comm; `gvins_upstream.patch` in GVINS | Parser/build/timing fixes **and alignment/reset-policy changes** |
| InGVIO | `01-build-cxx17-eigen-feature_tracker-dep.patch`, then `02-publish-ecef-odometry.patch` | C++17/Eigen build fix and ECEF pose publisher; upstream-default configuration, see [BUGFIX_NOTES.md](projects/InGVIO/BUGFIX_NOTES.md) |
| OKVIS2-X | None | RTKLIB-EX GNSS conversion and native stereo input; [reproduced settings and dependencies](projects/OKVIS2-X/BUGFIX_NOTES.md) |

GVINS and GICI-RRR include estimator-behavior changes described below; they
should not be described as data-format-only upstream baselines.
InGVIO's committed configuration is the upstream sportsfield default
(`gnss_chi2_test: 0`, `visual_noise: 0.18`); the collected trajectory is that
upstream-default replay.
SeA-RVINS batch/latent/scalar results are imported saved records; their
[provenance guide](results/SeA-RVINS/README.md) defines the available evidence.

## GICI-RTK: unpatched GNSS-only baseline

The RTK result uses upstream revision
`f2b8579f4fab9dff2b0950b11cc04aefe6d48e3a` without source patches.
Its inputs are rover/base GNSS; no IMU or camera is used. Follow the
[GICI run guide](projects/GICI-LIB/RUN.md) for the matching configuration.

## GICI-RRR: guards and low-feature behavior

RRR uses the same pinned upstream revision in a separate worktree.
[Patch 0001](projects/GICI-LIB/patches/0001-texcup-robustness-guards.patch)
guards missing GNSS/ambiguity history, empty estimator states and empty visual
input. It also bypasses RANSAC and accepts remaining tracks when fewer than
eight are tracked, and adds visual-initialization admission checks.
**The low-feature rules change estimator behavior.** The saved result has no
counter showing how often the fallback was used.

[Patch 0002](projects/GICI-LIB/patches/0002-record-origin-and-eof.patch) records
the actual fixed ENU origin and EOF, supporting estimated-attitude lever
conversion and supervised termination.

See the [GICI patch review](GICI_PATCH_REVIEW.md) for the exact differences
between the two selected GICI implementations.

## VINS-Fusion: keep global rotations normalized

**Upstream revision:** `be55a937a57436548ddfb1bd324bc1e9a9e828e0`.
Apply the three patches in [series](projects/VINS-Fusion/patches/series).

Upstream global fusion converts matrices to quaternions without enforcing unit
length. Norm errors can propagate into the VIO-to-global rotation and therefore
position estimates.
[Patch 0003](projects/VINS-Fusion/patches/0003-normalize-global-rotations.patch)
normalizes incoming, initialized, relative and optimized rotations. It forms
the global alignment from unit quaternions, maps the newest local pose onto
its optimized global pose, and rejects unusable solver updates.

GNSS factors and weights are retained. Global optimization uses five iterations
and the upstream alternate-frame estimator cadence. A GPS sidecar interpolates
positions onto those admitted image timestamps without changing camera/IMU
timestamps. The [data contract](VINS_FUSION_DATA_REVIEW.md) explains the
approximate input lever removal and live-output scoring.

```bash
source projects/GVINS/env.sh
python projects/VINS-Fusion/tests/test_global_rotation.py
python -m unittest discover -s projects/VINS-Fusion/tests -p test_extraction.py -v
```

The compiled regression checks unit rotations and newest-pose mapping.
Runtime verification checks actual output rotations; these checks do not
guarantee trajectory accuracy.

## IC-GVINS: validate time windows before indexing

**Upstream revision:** `644eed9e02c1239d6788f5b19123559735ca1b9e`.
Apply patches 03–08 in order as described in the [run guide](projects/IC-GVINS/RUN.md).
Patch 03 supplies diagnostics and orderly shutdown.
[Patch 04](projects/IC-GVINS/patches/04-time-window-bounds.patch) validates
intervals before indexing IMU/state buffers.

Online camera-delay estimation can make queued frame timestamps non-increasing.
The patch checks ordering and both IMU endpoints, keeps missing-state indices
signed, and validates neighboring states. A keyframe is admitted only after
its state is created; unsupported intervals are skipped and logged.
GNSS insertion validates replacement intervals before changing the window.

These guards can affect measurement admission. Native initialization thresholds,
GNSS weighting, solver settings, camera-extrinsic estimation and delay estimation
remain in use.

```bash
source projects/IC-GVINS/env.sh
python projects/IC-GVINS/tests/test_time_bounds.py
python -m unittest discover -s projects/IC-GVINS/tests -p test_extraction.py -v
```

The compiled regression checks missing endpoints, invalid ordering, empty
buffers and state lookup, while preserving valid IMU duration/increments.

## IC-GVINS: preserve states still used by GNSS

[Patch 05](projects/IC-GVINS/patches/05-preserve-gnss-supported-states.patch)
keeps a state while a GNSS observation references its timestamp, using the
existing 0.0001-second matching tolerance. Removing an associated visual
keyframe previously deleted this state while retaining the GNSS observation;
subsequent factor construction could no longer find it.

Visual keyframes can still be removed. Normal marginalization handles the
GNSS observation and supported state together. States without GNSS support
retain ordinary removal and IMU merging. This changes state retention to
preserve measurements; it does not alter their weights.

```bash
source projects/IC-GVINS/env.sh
python projects/IC-GVINS/loaders/verify_source.py
python projects/IC-GVINS/tests/test_gnss_state_retention.py
```

The regression compiles the actual removal method and covers timestamp
matching, duplicate observations, mixed cleanup and IMU sample merging.

## IC-GVINS: state pointers and feature tracking memory safety

| Patch | Defect and correction |
|---|---|
| [06-remap-prior-state-pointers.patch](projects/IC-GVINS/patches/06-remap-prior-state-pointers.patch) | Deque mutations can invalidate marginalization-prior pointers. Preserve prior-supported states, identify blocks by state timestamp and pose/mix identity, then rebind after mutations while preserving parameter order and extrinsic pointers. |
| [07-feature-grid-bounds.patch](projects/IC-GVINS/patches/07-feature-grid-bounds.patch) | Image-edge remainders and undistorted points can index outside the feature grid. Clamp finite coordinates to a valid cell and omit nonfinite points from counting. |
| [08-materialize-feature-velocity.patch](projects/IC-GVINS/patches/08-materialize-feature-velocity.patch) | An `auto` Eigen expression refers to expired temporary vectors. Store the computed feature velocity as `Vector3d` while its inputs are alive. |

AddressSanitizer reproduced the state-pointer and feature-grid defects in
focused fixtures and the feature-velocity lifetime defect in an estimator
replay. A 300-second instrumented replay with all active patches completed
without sanitizer errors. This validates the tested prefix, not the full route.

**Remaining failure:** prolonged visual tracking loss can stop marginalization
while the state window grows. A fixed iteration limit in state search can
reject timestamps present in a large window. The saved full run crashed in
Ceres Schur elimination; its precise invalid-memory cause remains unresolved.
The comparison therefore reports the explicitly selected pre-divergence prefix
through **18:34:59 UTC**, retaining the full **4,040-epoch** percentage denominator.
See [IC-GVINS status](projects/IC-GVINS/STATUS.md) and
[statistics selection](STATISTICS.md).

```bash
source projects/IC-GVINS/env.sh
python projects/IC-GVINS/loaders/verify_source.py
python projects/IC-GVINS/tests/test_prior_state_pointers.py
python projects/IC-GVINS/tests/test_feature_grid_bounds.py
python projects/IC-GVINS/tests/test_feature_velocity_lifetime.py
```

These focused software regressions do not establish full-route completion or
accuracy. Upstream GNSS reweighting and optimization's dependence on visual
updates remain part of this implementation.

## GVINS: timing and RINEX parser fixes

Pinned revisions: GVINS `d2cf40b49c6eb0e6ad3caa4f613713983be3fd74`,
gnss_comm `a2035fec1cc427bba184d25ff1fde8dcf23c9ce1`.
Apply the [saved patches](projects/GVINS/patches/) before building with the
[GVINS run guide](projects/GVINS/RUN.md).

Camera/IMU/bag stamps use UTC; GNSS payloads retain GPST. Set
`gnss_local_online_sync: 0` and `gnss_local_time_diff: 18.0`.
The patch initializes and preserves the time offset across resets, uses elapsed
seconds for clock propagation, and propagates clock bias to observation time
with matching optimization/marginalization Jacobians. The gnss_comm patch
sizes observation vectors safely, handles truncated records and supplies
parser/build compatibility.

**Estimator-behavior changes:** the GVINS patch widens GNSS-camera pairing to
0.15 s, permits alignment/yaw estimation when at least two window frames have
sufficient GNSS, retains ephemerides across VIO resets, and disables
position-jump-triggered reboots. The saved score includes these adaptations,
the documented `max_solver_time: 0.08`, and selected calibration/noise values.

```bash
source projects/GVINS/env.sh
python -m unittest discover -s projects/GVINS/tests -v
```

Compiled timing tests cover resets, irregular epochs and observation-time
residuals/Jacobians. Removing behavior changes would require a separate replay.
