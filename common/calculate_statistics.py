#!/usr/bin/env python3
"""Collect final antenna trajectories and recompute all method statistics.

Examples (from the repository root):
  python common/calculate_statistics.py --gt "$TEXCUP_DATA/ground_truth.log" --collect RTKLIB=results/RTKLIB
  python common/calculate_statistics.py --gt "$TEXCUP_DATA/ground_truth.log" --results results/final

The results folder is flat: METHOD.est.csv, METHOD.eval.json,
METHOD.errors.csv, METHOD.statistics.json, METHOD.error.png and a shared table.
"""
import argparse
import csv
from datetime import datetime, timezone
import fcntl
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

from evaluate import default_ground_truth, load_gt
from summarize_statistics import metrics_from_grid, summarize
from provenance import compact_metadata, normalize_file, portable_metadata

HERE = Path(__file__).resolve().parent
DEFAULT_RESULTS = HERE.parent / 'results/final'


def validate_trajectory(path, allow_empty=False):
    with path.open() as source:
        rows = list(csv.DictReader(source))
    if not rows:
        if allow_empty:
            return 0
        raise ValueError(f'Empty trajectory: {path}')
    coords = ('ecef_x', 'ecef_y', 'ecef_z') if 'ecef_x' in rows[0] else ('lat_deg', 'lon_deg', 'h_ell')
    values = np.array([[float(row[k]) for k in ('utc_sec', *coords)] for row in rows])
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError(f'Trajectory must have finite coordinates and increasing UTC times: {path}')
    return len(rows)


def select_table_metrics(errors, selection):
    """Retain an explicit time prefix without reducing the epoch denominator."""
    cutoff = selection['end_utc_exclusive']
    if not re.fullmatch(r'(?:[01]\d|2[0-3]):[0-5]\d:[0-5]\d', cutoff):
        raise ValueError(f'Invalid exclusive table cutoff: {cutoff}')
    hour, minute, second = map(int, cutoff.split(':'))
    end = hour*3600 + minute*60 + second
    if not errors[0, 0] < end <= errors[-1, 0] + 1:
        raise ValueError('Table cutoff must fall within the evaluation window')
    selected = errors.copy()
    selected[selected[:, 0] >= end, 1:] = np.nan
    return metrics_from_grid(selected)


def calculate(results, collections=(), gt=None, allow_incomplete=False):
    """Serialize collection so concurrent method completions retain both results."""
    results = Path(results).resolve()
    results.mkdir(parents=True, exist_ok=True)
    with (results / '.statistics.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        return _calculate(results, collections, gt, allow_incomplete)


def write_empty_evaluation(work, name, grid):
    """Represent a failed method with no usable trajectory without inventing poses."""
    report = dict(name=name, cut_utc='18:09:40', end_utc='19:16:59',
        n_window=len(grid), n_solved=0, availability_pct=0.0,
        hor_lt_1m_pct_of_window=0.0, hor_lt_1p5m_pct_of_window=0.0,
        hor_rms_solved=None, hor_median_solved=None, hor_max_solved=None,
        ver_rms_solved=None, hor_lt_1m_pct_of_solved=None,
        hor_lt_1p5m_pct_of_solved=None, t_first=None, t_last=None)
    (work/'eval.json').write_text(json.dumps(report, indent=2)+'\n')
    np.savetxt(work/'hor_err_timeseries.csv',
        np.column_stack((grid, np.full((len(grid), 2), np.nan))), delimiter=',',
        header='utc_sec,hor_err_m(nan=no solution),ver_err_m', comments='')


def _calculate(results, collections=(), gt=None, allow_incomplete=False):
    results = Path(results).resolve()
    results.mkdir(parents=True, exist_ok=True)
    catalog_path = results / 'manifest.json'
    catalog = json.loads(catalog_path.read_text()) if catalog_path.exists() else {'methods': {}}
    ground_truth = Path(gt or default_ground_truth()).expanduser().resolve()
    gt_t, _ = load_gt(ground_truth)
    grid = gt_t[(gt_t >= 65380) & (gt_t <= 69419)]
    if not np.array_equal(grid, np.arange(65380, 69420)):
        raise ValueError('Expected the inclusive 4,040-epoch TEX-CUP evaluation grid')
    for specification in collections:
        name, directory = specification.split('=', 1)
        if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
            raise ValueError(f'Invalid method filename: {name}')
        source = Path(directory).resolve()
        status_path = source / 'status.json'
        status = json.loads(status_path.read_text()) if status_path.exists() else {}
        original_phase = status.get('phase', 'completed')
        incomplete = original_phase != 'completed'
        finalization_path = source/'finalization.json'
        finalization = json.loads(finalization_path.read_text()) if finalization_path.exists() else {}
        if (original_phase == 'finalizing'
                and finalization.get('all_processes_exited') is True
                and finalization.get('run_status') == 'completed'
                and finalization.get('ready_for_collection') is True):
            incomplete = False
        if incomplete and not allow_incomplete:
            raise ValueError(f'Run is not completed: {source}')
        if incomplete and original_phase not in ('error', 'failed', 'stopped', 'interrupted', 'cancelled', 'aborted'):
            if not finalization.get('all_processes_exited'):
                raise ValueError(f'Incomplete run has no evidence that all processes exited: {source}')
        trajectory = source / 'est.csv'
        if incomplete and finalization.get('trajectory_file'):
            trajectory = (source/finalization['trajectory_file']).resolve()
            if not trajectory.is_relative_to(source):
                raise ValueError(f'Recovered trajectory is outside the run directory: {source}')
        n = validate_trajectory(trajectory, allow_empty=incomplete and allow_incomplete)
        run_status = finalization.get('run_status', original_phase) if (incomplete or original_phase == 'finalizing') else original_phase
        result_status = 'completed' if not incomplete else f'{run_status} ({"partial" if n else "no output"})'
        target = results / f'{name}.est.csv'
        if trajectory != target:
            shutil.copy2(trajectory, target)
        provenance = {'method': name, 'source_run': str(source), 'trajectory_rows': n,
                      'reference_point': 'ALT1 antenna',
                      'source_trajectory': str(trajectory), 'run_status': run_status,
                      'original_phase': original_phase, 'result_status': result_status,
                      'incomplete_collection': incomplete,
                      'source_artifacts': {}}
        for filename in ('run_manifest.json', 'verification.json', 'status.json',
                         'input_validation.json', 'preparation.json', 'finalization.json'):
            path = source / filename
            if path.exists():
                provenance['source_artifacts'][filename] = json.loads(path.read_text())
        for filename in ('gici_rtk.yaml', 'gici_rrr.yaml', 'gvins_texcup.yaml', 'texcup_rtk_demo5.conf'):
            path = source / filename
            if path.exists():
                provenance['source_artifacts'][filename] = path.read_text()
        provenance['display_name'] = provenance['source_artifacts'].get(
            'run_manifest.json', {}).get('display_name', name)
        provenance = portable_metadata(provenance)
        (results / f'{name}.source.json').write_text(json.dumps(provenance, indent=2)+'\n')
        catalog['methods'][name] = {k: provenance[k] for k in ('source_run', 'trajectory_rows',
            'run_status', 'result_status', 'incomplete_collection', 'display_name')}

    for source_metadata in results.glob('*.source.json'):
        normalize_file(source_metadata)
    trajectories = sorted(results.glob('*.est.csv'))
    if not trajectories:
        raise ValueError('No METHOD.est.csv trajectories found; use --collect METHOD=RUN_DIR first')
    comparison_path = results/'comparison.json'
    comparison = compact_metadata(json.loads(comparison_path.read_text())) if comparison_path.exists() else None
    display_names = {}
    if comparison is not None:
        methods = comparison['methods']
        ids = [entry['id'] for entry in methods]
        available = {p.name.removesuffix('.est.csv'): p for p in trajectories}
        if not ids or len(ids) != len(set(ids)) or set(ids) - available.keys():
            raise ValueError('comparison.json requires unique method IDs with saved trajectories')
        trajectories = [available[name] for name in ids]
        display_names = {entry['id']: entry['label'] for entry in methods}
    selection_path = results/'table_selections.json'
    selections = compact_metadata(json.loads(selection_path.read_text())) if selection_path.exists() else {}
    available = {p.name.removesuffix('.est.csv'): p for p in trajectories}
    for name, selection in selections.items():
        if name not in available:
            raise ValueError(f'{name}: table selection requires a saved trajectory; review table_selections.json')
    summaries = {}
    table_statuses = {}
    for trajectory in trajectories:
        name = trajectory.name.removesuffix('.est.csv')
        entry = catalog['methods'].get(name, {})
        display_name = display_names.setdefault(name, entry.get('display_name', name))
        rows = validate_trajectory(trajectory, allow_empty=entry.get('incomplete_collection', False))
        with tempfile.TemporaryDirectory(prefix=f'{name}-statistics-') as temporary:
            work = Path(temporary)
            shutil.copy2(trajectory, work/'est.csv')
            if rows:
                completed = subprocess.run([sys.executable, str(HERE/'evaluate.py'),
                    '--est', str(work/'est.csv'), '--out', str(work), '--gt', str(ground_truth),
                    '--name', display_name, '--cut', '18:09:40', '--end', '19:16:59'],
                    capture_output=True, text=True)
                if completed.returncode:
                    raise RuntimeError(f'{name} evaluation failed:\n{completed.stderr}\n{completed.stdout}')
            else:
                write_empty_evaluation(work, display_name, grid)
            report = summarize(work)
            errors = np.loadtxt(work/'hor_err_timeseries.csv', delimiter=',', skiprows=1)
            if not np.array_equal(errors[:, 0], grid):
                raise ValueError(f'{name}: evaluation grid differs from common GT grid')
            for source_name, target_suffix in [('eval.json', 'eval.json'),
                    ('hor_err_timeseries.csv', 'errors.csv'), ('error.png', 'error.png')]:
                if (work/source_name).exists():
                    shutil.copy2(work/source_name, results/f'{name}.{target_suffix}')
                elif target_suffix == 'error.png':
                    (results/f'{name}.{target_suffix}').unlink(missing_ok=True)
            report['run_dir'] = str(results)
            report['run_status'] = entry.get('run_status', 'completed')
            report['result_status'] = entry.get('result_status', 'completed')
            report['incomplete_collection'] = entry.get('incomplete_collection', False)
            report = portable_metadata(report)
            (results/f'{name}.statistics.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            summaries[name] = report['metrics']
            table_statuses[name] = report['result_status']
            if name in selections:
                summaries[name] = select_table_metrics(errors, selections[name])
                table_statuses[name] = f'{report["run_status"]} (pre-divergence partial)'
            detail = f' (before {selections[name]["end_utc_exclusive"]} UTC)' if name in selections else ''
            print(f'{display_name}: {summaries[name]["n_solved"]}/4040 solved in comparison{detail}', flush=True)

    protocol = dict(date_utc='2019-05-09', start_utc='18:09:40', end_utc='19:16:59',
        n_window=4040, reference='ALT1 antenna', match_tolerance_s=0.5,
        percentages='All 4,040 epochs; missing estimates fail thresholds',
        distance_statistics='Solved epochs; no zero filling or outlier removal',
        p95='NumPy linear percentile', error_3d='hypot(horizontal, vertical)')
    columns = [('Method', None), ('Status', None), ('Total epochs', 'n_window'), ('Solved epochs', 'n_solved'),
        ('Avail. (%)', 'availability_pct'), ('Hor. RMSE (m)', 'hor_rmse_solved_m'),
        ('Hor. MAX (m)', 'hor_max_solved_m'), ('Hor. <1.0 m (%)', 'hor_lt_1m_pct_of_window'),
        ('Hor. <1.5 m (%)', 'hor_lt_1p5m_pct_of_window'), ('Hor. P95 (m)', 'hor_p95_solved_m'),
        ('3D RMSE (m)', '3d_rmse_solved_m'), ('3D MAX (m)', '3d_max_solved_m'),
        ('3D P95 (m)', '3d_p95_solved_m')]
    with (results/'statistics.csv').open('w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow([label for label, _ in columns])
        for name, m in summaries.items():
            writer.writerow([display_names[name], table_statuses[name]]
                            + [m[key] for _, key in columns[2:]])
    lines = ['# Final benchmark statistics', '',
             '2019-05-09 **18:09:40–19:16:59 UTC**, inclusive: **4,040 epochs**.', '',
             'Percentages use all epochs. RMSE, MAX and P95 use solved epochs; '
             'missing errors are undefined and all finite outliers within the stated reporting interval are retained.', '',
             'Status distinguishes completed processing from explicitly collected failed or partial runs. '
             'Source provenance identifies imported results and local replays.', '',
             '| ' + ' | '.join(label for label, _ in columns) + ' |',
             '|---|' + '---:|'*(len(columns)-1)]
    for name, m in summaries.items():
        values = [display_names[name], table_statuses[name]]
        for _, key in columns[2:]:
            v = m[key]
            values.append('undefined' if v is None else str(v) if key.startswith('n_') else f'{v:.2f}')
        lines.append('| ' + ' | '.join(values) + ' |')
    for name, selection in selections.items():
        lines += ['', f'**{name} partial result:** scoring epochs from 18:09:40 UTC up to '
                  f'{selection["end_utc_exclusive"]} UTC (exclusive). '
                  f'{summaries[name]["n_solved"]:,} retained solved epochs; all percentages still use '
                  '**4,040 epochs**. Later epochs count as unavailable in this table. '
                  + selection['reason'] + f' Full saved output: [{name}.statistics.json]({name}.statistics.json).']
    if selections:
        lines += ['', 'The time selections are saved in [table_selections.json](table_selections.json) '
                  'and apply to the combined Markdown, CSV and JSON tables. '
                  'The original trajectories, error grids and per-method statistics remain untruncated. '
                  'Partial distance metrics describe the selected interval, not full-route accuracy.']
    lines += ['', 'Download ground truth following [data instructions](../../data/README.md), '
              'then recalculate every method from its saved antenna trajectory:', '', '```bash',
              'export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"',
              'python common/calculate_statistics.py --results results/final \\',
              '  --gt "$TEXCUP_DATA/ground_truth.log"', '```', '']
    (results/'statistics.md').write_text('\n'.join(lines))
    (results/'statistics.json').write_text(json.dumps({'protocol': protocol, 'methods': summaries,
        'display_names': display_names, 'comparison': comparison,
        'statuses': table_statuses, 'table_selections': selections}, indent=2, allow_nan=False)+'\n')
    catalog.update(updated_utc=datetime.now(timezone.utc).isoformat(), protocol=protocol)
    catalog = portable_metadata(catalog)
    catalog_path.write_text(json.dumps(catalog, indent=2)+'\n')
    (results/'README.md').write_text('''# Final benchmark results

See [STATISTICS.md](../../STATISTICS.md) for the scoring environment, commands,
evaluation protocol, and recovery of stopped runs.

This folder holds antenna trajectories and statistics for completed methods and
explicitly collected failed or partial runs, identified by the Status column.
It contains real file copies, so recalculation does not depend on the
per-run `latest` links. Ground truth is TEX-CUP antenna 2 / ALT1.
Download ground truth from the publisher using [data instructions](../../data/README.md).
It stays in the local dataset directory and is not copied into this results folder.

- `statistics.md`, `statistics.csv`, `statistics.json`: combined results.
- `comparison.json`: methods, display labels and row order for this comparison.
  Only these methods enter the generated tables; pending methods are added here
  after their trajectories and provenance have been collected.
- `table_selections.json`, when present: explicit time prefixes for the combined tables;
  later epochs count as unavailable while the full 4,040-epoch denominator is retained.
  These selections do not alter the per-method files below.
- `METHOD.est.csv`: antenna trajectory in UTC seconds of day, ECEF or LLA.
- `METHOD.errors.csv`: all 4,040 evaluation epochs, including NaN gaps.
- `METHOD.eval.json`, `METHOD.statistics.json`, `METHOD.error.png`: per-method results.
- `METHOD.source.json`, `manifest.json`: original run and calculation provenance.
  Paths are repository-relative; symbolic environment references and original
  run identifiers are metadata, not prerequisites for recalculating statistics.
  Input metadata, code revisions and saved configuration values are retained.

From the repository root, with NumPy and Matplotlib installed:

```bash
export TEXCUP_DATA="${TEXCUP_DATA:-$PWD/data/tex_cup}"
python common/calculate_statistics.py --results results/final \\
  --gt "$TEXCUP_DATA/ground_truth.log"
```

To add or replace a completed method, then recalculate all methods:

```bash
python common/calculate_statistics.py --results results/final \\
  --gt "$TEXCUP_DATA/ground_truth.log" \\
  --collect METHOD=results/METHOD/latest
```

Availability and threshold percentages use all 4,040 epochs, including missing
estimates. Distance statistics use solved epochs; no missing error is replaced
by zero. Runtime logs and ROS bags are generated when methods are replayed;
the original run directories recorded in provenance need not be distributed.

Failed runs require explicit `--allow-incomplete` when collecting. Their original
status and recovery exclusions remain in `METHOD.source.json`. A failed method
with no usable output has zero availability and undefined distance statistics.
''')
    return summaries


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', default=str(DEFAULT_RESULTS))
    parser.add_argument('--collect', action='append', default=[], metavar='METHOD=RUN_DIR')
    parser.add_argument('--gt', help='Downloaded ground truth; defaults to TEXCUP_DATA or repository data/tex_cup')
    parser.add_argument('--allow-incomplete', action='store_true',
                        help='Explicitly collect stopped failed/partial runs, preserving their status')
    args = parser.parse_args()
    try:
        calculate(args.results, args.collect, args.gt, args.allow_incomplete)
    except FileNotFoundError as error:
        parser.error(str(error))
