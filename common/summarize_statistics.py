#!/usr/bin/env python3
"""Summarize the saved 1 Hz evaluator grid, preserving missing epochs."""
import argparse
import json
from pathlib import Path

import numpy as np
from provenance import portable_metadata


def metrics_from_grid(a):
    """Compute metrics from a complete epoch grid, including unavailable rows."""
    if (a.ndim != 2 or a.shape[1] != 3 or not len(a) or
            not np.isfinite(a[:, 0]).all() or np.any(np.diff(a[:, 0]) <= 0) or
            np.isinf(a[:, 1:]).any() or
            not np.array_equal(np.isfinite(a[:, 1]), np.isfinite(a[:, 2]))):
        raise ValueError('Invalid evaluator grid or inconsistent horizontal/vertical support')
    solved = np.isfinite(a[:, 1:]).all(axis=1)
    h, v = a[solved, 1], a[solved, 2]
    n, ns = len(a), int(solved.sum())
    if np.any(h < 0) or np.any(v < 0):
        raise ValueError('Invalid errors or epoch counts')
    m = dict(n_window=n, n_solved=ns, n_missing=n-ns,
             availability_pct=100*ns/n,
             hor_lt_1m_count=int(np.sum(h < 1)),
             hor_lt_1p5m_count=int(np.sum(h < 1.5)),
             hor_lt_1m_pct_of_window=float(100*np.sum(h < 1)/n),
             hor_lt_1p5m_pct_of_window=float(100*np.sum(h < 1.5)/n))
    for name, errors in [('hor', h), ('3d', np.hypot(h, v))]:
        m[f'{name}_rmse_solved_m'] = float(np.sqrt(np.mean(errors**2))) if ns else None
        m[f'{name}_max_solved_m'] = float(errors.max()) if ns else None
        m[f'{name}_p95_solved_m'] = float(np.percentile(errors, 95, method='linear')) if ns else None
    return m


def summarize(run):
    run = Path(run).resolve()
    previous = json.loads((run / 'eval.json').read_text())
    a = np.loadtxt(run / 'hor_err_timeseries.csv', delimiter=',', skiprows=1,
                   ndmin=2)
    m = metrics_from_grid(a)
    n, ns = m['n_window'], m['n_solved']
    if n != previous['n_window'] or ns != previous['n_solved']:
        raise ValueError('Invalid errors or epoch counts')
    for key in ('availability_pct', 'hor_lt_1m_pct_of_window', 'hor_lt_1p5m_pct_of_window'):
        if not np.isclose(m[key], previous[key], rtol=1e-12):
            raise ValueError(f'Grid and eval.json disagree: {key}')
    if ns and not np.isclose(m['3d_rmse_solved_m']**2,
                            previous['hor_rms_solved']**2 + previous['ver_rms_solved']**2,
                            rtol=1e-12):
        raise ValueError('Grid and eval.json disagree: 3D RMSE identity')
    report = dict(name=previous['name'], run_dir=str(run),
                  cut_utc=previous['cut_utc'], end_utc=previous['end_utc'],
                  metrics=m,
                  protocol=dict(percentage_denominator='n_window; missing epochs fail thresholds',
                                distance_population='n_solved; missing errors are undefined, not zero',
                                p95='NumPy linear percentile of solved errors',
                                error_3d='hypot(horizontal, vertical)', outlier_filter='None'))
    report = portable_metadata(report)
    (run/'statistics.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    rows = [('Avail. (%)', 'availability_pct'), ('Horizontal RMSE (m)', 'hor_rmse_solved_m'),
            ('Horizontal MAX (m)', 'hor_max_solved_m'),
            ('Horizontal < 1.0 m (%)', 'hor_lt_1m_pct_of_window'),
            ('Horizontal < 1.5 m (%)', 'hor_lt_1p5m_pct_of_window'),
            ('Horizontal P95 (m)', 'hor_p95_solved_m'),
            ('3D RMSE (m)', '3d_rmse_solved_m'), ('3D MAX (m)', '3d_max_solved_m'),
            ('3D P95 (m)', '3d_p95_solved_m')]
    lines = [f'# {previous["name"]} statistics', '',
             f'Window: {previous["cut_utc"]}–{previous["end_utc"]} UTC, inclusive.', '',
             f'**{n:,} total epochs; {ns:,} solved; {n-ns:,} missing.**', '',
             '| Metric | Value |', '|---|---:|']
    for label, key in rows:
        value = f'{m[key]:,.2f}' if m[key] is not None else 'undefined'
        lines.append(f'| {label} | {value} |')
    lines += ['', f'All percentages use all **{n:,} epochs**. Missing estimates fail both thresholds.', '',
              f'RMSE, MAX and P95 use the **{ns:,} solved epochs**, with no outlier removal. '
              'Missing errors are undefined and are never replaced by zero. '
              'Full-window distance statistics require an explicit missing-error policy.', '',
              '3D error is `hypot(horizontal, vertical)`; P95 uses NumPy linear interpolation. '
              'Association follows the common evaluator: nearest estimate within ±0.5 s, '
              'with GT interpolated at the selected estimate timestamp.', '',
              'Full precision and scoring protocol: [statistics.json](statistics.json).', '',
              'Reproduce from the repository root with Python/NumPy:', '', '```bash',
              'python common/summarize_statistics.py --results results/METHOD/latest', '```', '']
    (run/'statistics.md').write_text('\n'.join(lines))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', required=True)
    args = parser.parse_args()
    print(json.dumps(summarize(args.results), indent=2))
