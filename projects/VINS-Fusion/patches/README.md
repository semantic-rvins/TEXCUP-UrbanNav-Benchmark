# Active VINS-Fusion patches

Use the pinned revision in `vins_fusion_upstream.commit` and apply only the
filenames in `series`, in their listed order:

```bash
while IFS= read -r patch; do
  git -C upstream apply "$PWD/patches/$patch"
done < patches/series
```

Run this from `projects/VINS-Fusion` after cloning and checking out the pinned
revision. The active series contains build compatibility and origin logging,
runtime diagnostics/output-path handling, and the unit-rotation software fix.
It preserves upstream frame admission and optimizer settings.

See the root [BUG_FIXES.md](../../../BUG_FIXES.md) for the full reproduction
instructions and verification commands.
