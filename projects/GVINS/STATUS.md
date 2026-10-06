# GVINS status

**Full replay, extraction, and evaluation completed.** The recorded setup uses
pinned GVINS and gnss_comm source with the saved patches, native-resolution
images, and corrected GNSS/UTC handling. Build, focused timing checks, and
input-bag verification passed. Large trajectory outliers are retained in
[the shared statistics](../../results/final/statistics.md).

The completed replay exported 18,225 finite, time-ordered estimates and solved
3,664 of the 4,040 evaluation epochs. Runtime completion does not establish
position accuracy. The active patch also contains GNSS pairing/alignment and
reset-policy changes; see [BUG_FIXES.md](../../BUG_FIXES.md) for the distinction
between those changes and software fixes.

Follow [RUN.md](RUN.md) for reproduction. Live status for a new run is
`results/GVINS/latest/status.json`. Completion is `phase: completed` with
message `Replay, extraction and evaluation completed; review eval.json and logs`.
Then execute the run guide's manual statistics collection step to update
`results/final/GVINS.*`; this supervisor does not collect automatically.
