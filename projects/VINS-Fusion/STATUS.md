# VINS-Fusion status

**Full replay, output verification, and statistics completed.** The active
implementation preserves upstream frame admission and optimizer settings and
applies the software patches listed in `patches/series`. Source, build, input,
rotation, and extraction checks passed.

The replay tracked all 40,372 stereo pairs, produced 20,176 VIO/global poses,
and accepted 4,037 GPS associations. No estimator resets or invalid rotations
were recorded. These checks establish processing coverage; accuracy is reported
in [the shared statistics](../../results/final/statistics.md).

Follow [RUN.md](RUN.md) for reproduction. For a new replay,
`results/VINS-Fusion/latest/status.json` is live and updates every five seconds.
Completion requires `phase: completed`, successful output verification, and
message `Replay and statistics completed; collected into results/final`.
A failed or stopped run keeps its terminal status when usable output is scored.
