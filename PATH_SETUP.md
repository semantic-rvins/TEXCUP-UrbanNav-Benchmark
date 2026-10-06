# Paths and environments

Start with [RUN_INSTRUCTIONS.md](RUN_INSTRUCTIONS.md). All supported launch
commands are written for a Bash shell at the benchmark root:

```bash
export BENCH="$PWD"
export TEXCUP_DATA="${TEXCUP_DATA:-$BENCH/data/tex_cup}"
export BENCH_CONDA_ROOT="${BENCH_CONDA_ROOT:-$HOME/miniconda3}"
```

Pass `--data-dir "$TEXCUP_DATA"` to input builders and supervisors, and
`--gt "$TEXCUP_DATA/ground_truth.log"` when evaluating or recalculating saved
outputs. [Download and prepare the publisher ground truth](data/README.md)
first; it is not copied into `results/final`.

GVINS/VINS-Fusion use `projects/GVINS/env.sh`; `BENCH_ROS_ENV` can override its
Conda environment prefix. IC-GVINS uses `projects/IC-GVINS/env.sh`; see that file
for its environment-prefix override. GICI commands explicitly activate
`benchmark_gici`. Package locks are in [environments](environments/).

Run supervisors resolve template input/output/calibration paths against the
checkout and write the effective configuration into each fresh run directory.
Use the wrappers in each method's run guide to create these configurations.

| Location | Version control policy |
|---|---|
| `projects/*/config`, `loaders`, `patches`, `tests`, run wrappers | Tracked reproducibility inputs |
| `environments/` | Tracked dependency locks |
| `data/tex_cup/` | Ignored local TEX-CUP downloads and processed inputs; download/preparation instructions are tracked under `data/` |
| `data/tex_cup/brdm1290{,_v304}.19p` | Independent broadcast navigation inputs retained with Git LFS; see [data instructions](data/README.md) |
| `common/config/texcup_*_calibration.yaml` | Tracked benchmark calibration profiles; the preparation helper materializes legacy local paths |
| `data/tex_cup/camera_data/` | Ignored camera download; see [data instructions](data/README.md) |
| `results/final/` | Tracked canonical estimator trajectories, statistics and provenance; no bundled TEX-CUP ground truth |
| Upstream clones, catkin/build folders, bags and converted image packs | Ignored; recreate using method guide |
| `results/METHOD/`, logs, PIDs, `latest` links | Ignored local runtime outputs |
| `results/SeA-RVINS/*.7z` | Supplied SeA-RVINS batch/latent/scalar records, tracked with Git LFS |
| `projects/*/results/` | Ignored generated outputs |

Saved `.source.json` files summarize the available source revision, settings,
frame/time conventions and run status. They are compact metadata rather than
a complete audit of the original run. Paths are relative to the repository root;
`source_run` identifies the originating generated run, whose directory need
not be distributed. Statistics read the committed trajectories and separately
downloaded ground truth, not those run directories. Original configuration
snapshots describe the saved runs; the new windowed SBF recipe
does not claim to recreate every historical input byte. Figure provenance uses
the same path convention.

Locations outside the repository use symbolic references such as
`<conda-env:benchmark_ros>`, `<home>` and `<temporary>`. These are evidence
labels, not paths or commands to execute.
New collections and plots apply these conventions automatically.
