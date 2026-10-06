# IC-GVINS status

The current comparison uses the **pre-divergence prefix through 18:34:59 UTC**:
**1,502 / 4,040 epochs (37.18% availability)**. All percentage metrics use the
full evaluation grid from 18:09:40 through 19:16:59 UTC. Distance metrics use
the matched prefix epochs, retaining their finite outliers. This retrospective
selection does not establish full-route accuracy or completion.

The source replay later stopped with **SIGSEGV (`-11`)**. Its full trajectory,
error grid and per-method statistics remain in `results/final`; the combined
comparison applies [table_selections.json](../../results/final/table_selections.json).
The exact source and active patches are recorded in
[IC-GVINS.source.json](../../results/final/IC-GVINS.source.json).

Active patches 03–08 address diagnostics/shutdown, time-window/index safety,
GNSS/prior state lifetime, feature-grid bounds and Eigen temporary lifetime.
Upstream initialization and GNSS weighting remain unchanged. Focused software
checks passed, but the cause of the later invalid access inside the solver
remains unresolved. See [BUG_FIXES.md](../../BUG_FIXES.md) for the patch scope
and remaining limitation.

Follow [RUN.md](RUN.md) for the pinned source, build, input preparation, replay
and statistics commands. A new replay writes live status to
`results/IC-GVINS/latest/status.json`. Completion requires `phase: completed`,
successful output verification, and the message
`Replay and statistics completed; collected into results/final`.
A failed run retains its failure status when usable output is scored.
