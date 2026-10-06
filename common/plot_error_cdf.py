#!/usr/bin/env python3
"""Plot current TEX-CUP error grids without re-association or outlier removal.

The upper panel shows the selected SeA-RVINS variants over time. The lower
panel shows unconditional horizontal-error CDFs for selected current methods.
All percentages use the complete 4,040-epoch window, including unavailable
epochs. Only METHOD.errors.csv from the shared final folder supplies errors;
comparison.json defines the comparison and table_selections.json supplies any
explicit time-prefix selections, matching the combined statistics table.

Examples:
  python common/plot_error_cdf.py
  python common/plot_error_cdf.py --results results/final --out-prefix figures/fig_error_cdf
  python common/plot_error_cdf.py --cdf-only
  python common/plot_error_cdf.py --method RTKLIB --method SeA-RVINS-latent-robust
"""
import argparse
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import LogLocator, MultipleLocator, NullFormatter
from matplotlib.transforms import Bbox
import numpy as np
from provenance import portable_metadata


BENCH = Path(__file__).resolve().parents[1]
CUT = 18*3600 + 9*60 + 40
END = 19*3600 + 16*60 + 59
GRID = np.arange(CUT, END+1, dtype=float)
N_EPOCHS = len(GRID)
SEA_METHODS = ('SeA-RVINS-latent-robust', 'SeA-RVINS-scalar-robust',
               'SeA-RVINS-batch-robust')
CDF_ONLY_METHODS = ('RTKLIB', 'GICI-RRR', 'VINS-Fusion', *SEA_METHODS)
METHOD_ORDER = ('RTKLIB', 'GVINS', 'GICI-RTK', 'GICI-RRR', 'VINS-Fusion',
                'IC-GVINS', *SEA_METHODS)
LABELS = {'SeA-RVINS-latent-robust': 'SeA-RVINS (latent)',
          'SeA-RVINS-scalar-robust': 'SeA-RVINS (scalar)',
          'SeA-RVINS-batch-robust': 'SeA-RVINS (batch)'}
COLORS = {'RTKLIB': '#D55E00', 'GVINS': '#0072B2', 'GICI-RTK': '#E69F00',
          'GICI-RRR': '#56B4E9', 'VINS-Fusion': '#CC79A7', 'IC-GVINS': '#333333',
          'SeA-RVINS-latent-robust': '#009E73', 'SeA-RVINS-scalar-robust': '#AA8F00',
          'SeA-RVINS-batch-robust': '#6F4E9C'}
STYLES = {'GVINS': '--', 'GICI-RTK': '-.', 'IC-GVINS': ':',
          'SeA-RVINS-scalar-robust': '--', 'SeA-RVINS-batch-robust': '-.'}


def is_no_vision(method):
    compact = re.sub(r'[^a-z0-9]', '', method.lower())
    return 'novision' in compact or 'withoutvision' in compact


def comparison_settings(results):
    path = Path(results)/'comparison.json'
    if not path.exists():
        return list(METHOD_ORDER), LABELS.copy(), None
    comparison = json.loads(path.read_text())
    entries = comparison['methods']
    order = [entry['id'] for entry in entries]
    if not order or len(order) != len(set(order)):
        raise ValueError('comparison.json requires a nonempty list of unique method IDs')
    labels = LABELS.copy()
    labels.update({entry['id']: entry['label'] for entry in entries})
    return order, labels, dict(file=path.name)


def select_methods(results, requested=None, require_sea=True):
    """Discover canonical result grids, excluding the retired no-vision method."""
    available = {p.name[:-len('.errors.csv')] for p in Path(results).glob('*.errors.csv')}
    excluded = sorted(m for m in available if is_no_vision(m))
    order, _, comparison = comparison_settings(results)
    if requested is not None:
        candidates = set(requested)
    else:
        candidates = set(order) if comparison else available
    missing = candidates - available
    if missing:
        raise ValueError('Missing method error grids: '+', '.join(sorted(missing)))
    candidates = {m for m in candidates if not is_no_vision(m)}
    ordered = [m for m in order if m in candidates]
    ordered += sorted(candidates-set(order))
    if not ordered:
        raise ValueError('No eligible METHOD.errors.csv files found')
    if require_sea and not any(m in SEA_METHODS for m in ordered):
        raise ValueError('Select at least one SeA-RVINS variant for the upper panel')
    return ordered, excluded


def load_error_grid(results, method):
    path = Path(results)/(method+'.errors.csv')
    values = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    if values.shape != (N_EPOCHS, 3):
        raise ValueError(f'{method}: require {N_EPOCHS} rows and three error-grid columns')
    if not np.allclose(values[:, 0], GRID, atol=1e-6, rtol=0):
        raise ValueError(f'{method}: expected 18:09:40-19:16:59 UTC at 1 Hz')
    errors = values[:, 1:]
    if (np.isinf(errors).any() or
            not np.array_equal(np.isfinite(errors[:, 0]), np.isfinite(errors[:, 1])) or
            np.any(errors[np.isfinite(errors)] < 0)):
        raise ValueError(f'{method}: invalid errors or inconsistent horizontal/vertical support')
    n_solved = int(np.isfinite(errors[:, 0]).sum())
    metadata_path = Path(results)/(method+'.eval.json')
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text())
        if metadata.get('n_window') != N_EPOCHS or metadata.get('n_solved') != n_solved:
            raise ValueError(f'{method}: eval.json disagrees with the error grid')
    return values


def select_time_prefix(results, method, values, selection):
    """Mask an explicit time prefix on a copy, preserving the complete grid."""
    trajectory = Path(results)/(method+'.est.csv')
    if not trajectory.is_file():
        raise ValueError(f'{method}: table selection requires a saved trajectory; review table_selections.json')
    cutoff = selection['end_utc_exclusive']
    if not re.fullmatch(r'(?:[01]\d|2[0-3]):[0-5]\d:[0-5]\d', cutoff):
        raise ValueError(f'Invalid exclusive table cutoff: {cutoff}')
    hour, minute, second = map(int, cutoff.split(':'))
    end = hour*3600 + minute*60 + second
    if not CUT < end <= END+1:
        raise ValueError('Table cutoff must fall within the evaluation window')
    selected = values.copy()
    selected[selected[:, 0] >= end, 1:] = np.nan
    return selected


def cdf_at(errors, thresholds):
    """P(error <= threshold), using all input epochs as the denominator."""
    errors = np.asarray(errors, dtype=float)
    if errors.ndim != 1 or not len(errors):
        raise ValueError('CDF needs a nonempty one-dimensional epoch grid')
    solved = np.sort(errors[np.isfinite(errors)])
    return 100.0*np.searchsorted(solved, thresholds, side='right')/len(errors)


def error_limits(grids):
    """Include every positive finite error, including extreme outliers."""
    solved = np.concatenate([v[np.isfinite(v[:, 1]), 1] for v in grids.values()])
    positive = solved[solved > 0]
    if not len(positive):
        return 0.01, 10.0
    low = min(0.01, float(10.0**np.floor(np.log10(positive.min()))))
    high = max(10.0, float(10.0**np.ceil(np.log10(positive.max()))))
    return low, high


def render(results, out_prefix, requested=None, *, cdf_only=False, swap_sea_colors=False,
           trim_top_left=False):
    results, out_prefix = Path(results).resolve(), Path(out_prefix).resolve()
    if cdf_only and requested is None:
        requested = CDF_ONLY_METHODS
    methods, excluded = select_methods(results, requested, require_sea=not cdf_only)
    _, labels, comparison = comparison_settings(results)
    grids = {m: load_error_grid(results, m) for m in methods}
    full_solved = {m: int(np.isfinite(values[:, 1]).sum()) for m, values in grids.items()}
    selection_path = results/'table_selections.json'
    selections = json.loads(selection_path.read_text()) if selection_path.exists() else {}
    applied_selections = {m: selections[m] for m in methods if m in selections}
    for method, selection in applied_selections.items():
        grids[method] = select_time_prefix(results, method, grids[method], selection)
    sea = [m for m in methods if m in SEA_METHODS]
    xmin, xmax = (1e-2, 1e3) if cdf_only else error_limits(grids)
    report = dict(results_directory=str(results),
        methods=methods, labels={m: labels.get(m, m) for m in methods},
        comparison=comparison, table_selections=applied_selections,
        time_panel_methods=[] if cdf_only else sea, excluded_no_vision_methods=excluded,
        window=dict(date='2019-05-09', start_utc='18:09:40', end_utc='19:16:59',
                    start_utc_sec=CUT, end_utc_sec=END, interval_s=1, epochs=N_EPOCHS),
        cdf=dict(definition='100 * count(finite horizontal error <= threshold) / 4040',
                 denominator=N_EPOCHS, missing_epochs='Unavailable; remain in denominator',
                 x_scale='log', x_limits_m=[xmin, xmax],
                 reference_lines_m=[1.0, 1.5],
                 zero_error_policy='Included in CDF at all positive thresholds',
                 distance_filter='None; every finite error within the reported time selection is retained'),
        source_files={}, dependencies=dict(numpy=np.__version__, matplotlib=matplotlib.__version__))
    colors = COLORS.copy()
    if swap_sea_colors:
        latent, scalar = SEA_METHODS[:2]
        colors[latent], colors[scalar] = colors[scalar], colors[latent]
    styles = {m: dict(color=colors.get(m, plt.get_cmap('tab20')(i % 20)),
                     linestyle=STYLES.get(m, '-'), label=labels.get(m, m))
              for i, m in enumerate(methods)}
    report['styles'] = styles
    if applied_selections:
        report['table_selection_source'] = dict(file=selection_path.name)
    with plt.rc_context({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.titlesize': 11, 'axes.labelsize': 10,
                         'axes.linewidth': .7, 'pdf.fonttype': 42, 'ps.fonttype': 42,
                         'savefig.facecolor': 'white'}):
        fig, (time_ax, cdf_ax) = plt.subplots(2, 1, figsize=(8.3, 7.2),
                                            gridspec_kw={'height_ratios': [1, 1.2]})
        fig.subplots_adjust(left=.105, right=.985, top=.965, bottom=.23, hspace=.45)
        original_height = fig.get_figheight()
        cdf_bounds = cdf_ax.get_position().frozen()
        if cdf_only:
            # Preserve height, margins and width per decade of the six-decade CDF.
            time_ax.remove()
            axes_height = cdf_bounds.height * original_height
            original_width = fig.get_figwidth()
            left_margin = cdf_bounds.x0 * original_width
            right_margin = (1-cdf_bounds.x1) * original_width
            axes_width = cdf_bounds.width * original_width * np.log10(xmax/xmin) / 6
            bottom_margin, top_margin = .54, .15
            fig.set_size_inches(left_margin + axes_width + right_margin,
                                axes_height + bottom_margin + top_margin)
            cdf_ax.set_position([left_margin / fig.get_figwidth(), bottom_margin / fig.get_figheight(),
                                 axes_width / fig.get_figwidth(), axes_height / fig.get_figheight()])
        minutes = (GRID-CUT)/60.0
        top_max = 0.
        if not cdf_only:
            for method in sea:
                errors = grids[method][:, 1]
                finite = errors[np.isfinite(errors)]
                if len(finite): top_max = max(top_max, float(finite.max()))
                time_ax.plot(minutes, errors, linewidth=1.25, **styles[method])
            time_ax.set(xlabel='Time since 18:09:40 UTC (min)', ylabel='Horizontal error (m)',
                        xlim=(0, minutes[-1]), ylim=(0, max(1., top_max*1.08)))
            time_ax.set_title('(a) SeA-RVINS horizontal error', loc='left')
            time_ax.grid(alpha=.25, linewidth=.5)
            time_ax.legend(loc='upper left', frameon=False, ncol=len(sea), fontsize=9)
        handles = []
        for method in methods:
            errors = grids[method][:, 1]
            finite = errors[np.isfinite(errors)]
            # Every observed positive error is a step, so no sampled x-grid
            # can miss a small cluster or omit a very large outlier.
            xs = np.unique(np.concatenate(([xmin, xmax], finite[finite > 0])))
            ys = cdf_at(errors, xs)
            line, = cdf_ax.step(xs, ys, where='post', linewidth=1.65, **styles[method])
            handles.append(line)
            path = results/(method+'.errors.csv')
            report['source_files'][method] = dict(file=path.name,
                total_epochs=N_EPOCHS, solved_epochs=len(finite),
                full_source_solved_epochs=full_solved[method],
                unavailable_epochs=N_EPOCHS-len(finite),
                cdf_endpoint_pct=float(100*len(finite)/N_EPOCHS),
                zero_errors=int(np.count_nonzero(finite == 0)),
                maximum_horizontal_error_m=float(finite.max()) if len(finite) else None)
        cdf_ax.set_xscale('log')
        cdf_ax.set(xlim=(xmin, xmax), ylim=(0, 102),
                   xlabel='Horizontal error (m)', ylabel='Fraction of all epochs (%)')
        if not cdf_only:
            cdf_ax.set_title('(b) Horizontal-error CDF', loc='left')
        cdf_ax.yaxis.set_major_locator(MultipleLocator(20))
        cdf_ax.xaxis.set_major_locator(LogLocator(base=10, numticks=12))
        cdf_ax.xaxis.set_minor_locator(LogLocator(base=10, subs=(2, 5), numticks=36))
        cdf_ax.xaxis.set_minor_formatter(NullFormatter())
        cdf_ax.grid(alpha=.25, linewidth=.5, which='major')
        cdf_ax.grid(alpha=.12, linewidth=.4, which='minor')
        for value in (1., 1.5):
            cdf_ax.axvline(value, color='.6', linestyle=(0, (3, 3)), linewidth=.7, zorder=0)
        cdf_ax.text(1., 4, '1.0 m', ha='right', va='bottom', fontsize=8, color='.4')
        cdf_ax.text(1.5, 4, '1.5 m', ha='left', va='bottom', fontsize=8, color='.4')
        if cdf_only:
            cdf_ax.legend(handles=handles, loc='lower right', frameon=False,
                          fontsize=9.5, handlelength=2.5)
        else:
            fig.legend(handles=handles, loc='lower center', ncol=3 if len(handles) > 8 else 2, frameon=False,
                       fontsize=9.5, handlelength=2.5, columnspacing=2.8,
                       bbox_to_anchor=(.545, .035))
            fig.text(.545, .012, 'All 4,040 epochs in denominator; missing epochs unavailable; no outlier removal.',
                     ha='center', fontsize=8, color='.35')
        bounds = cdf_ax.get_position()
        report['layout'] = dict(mode='cdf_only' if cdf_only else 'two_panel',
            figure_size_inches=fig.get_size_inches().tolist(),
            cdf_axes_width_inches=bounds.width * fig.get_figwidth(),
            cdf_axes_height_inches=bounds.height * fig.get_figheight(),
            legend_location='inside lower right' if cdf_only else 'below axes',
            explanatory_annotations=not cdf_only)
        export_bbox = None
        if trim_top_left:
            fig.set_dpi(300)
            fig.canvas.draw()
            tight = fig.get_tightbbox(fig.canvas.get_renderer())
            width, height = fig.get_size_inches()
            padding = .02
            left = max(0., tight.x0 - padding)
            top = min(height, tight.y1 + padding)
            export_bbox = Bbox.from_extents(left, 0., width, top)
            report['layout']['uncropped_figure_size_inches'] = [width, height]
            report['layout']['figure_size_inches'] = [export_bbox.width, export_bbox.height]
            report['layout']['trim_top_left'] = dict(
                padding_inches=padding, left_removed_inches=left,
                top_removed_inches=height-top)
        out_prefix.parent.mkdir(parents=True, exist_ok=True)
        paths = {extension: Path(str(out_prefix)+'.'+extension) for extension in ('png', 'pdf')}
        fig.savefig(paths['png'], dpi=300, bbox_inches=export_bbox)
        fig.savefig(paths['pdf'], bbox_inches=export_bbox)
        plt.close(fig)
    report['time_panel'] = None if cdf_only else dict(
        y_scale='linear', y_limits_m=[0., max(1., top_max*1.08)],
        missing_epochs='NaN gaps; no interpolation')
    report['outputs'] = {extension: dict(file=str(path))
                         for extension, path in paths.items()}
    report = portable_metadata(report)
    provenance = Path(str(out_prefix)+'.provenance.json')
    provenance.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, default=BENCH/'results/final')
    parser.add_argument('--out-prefix', type=Path,
                        help='output filename without extension; defaults to figures/fig_error_cdf[_only]')
    parser.add_argument('--cdf-only', action='store_true',
                        help='export RTKLIB, GICI-RRR, VINS-Fusion and all three SeA-RVINS variants; '
                             'crop to 0.01–1000 m and shorten width while preserving height')
    parser.add_argument('--method', action='append', help='method identifier; repeat to select a subset')
    parser.add_argument('--swap-sea-colors', action='store_true',
                        help='exchange latent and scalar colors while preserving line styles')
    parser.add_argument('--trim-top-left', action='store_true',
                        help='crop top and left white margins with 0.02 inch padding around labels')
    args = parser.parse_args()
    try:
        prefix = args.out_prefix or BENCH/'figures'/('fig_error_cdf_only' if args.cdf_only else 'fig_error_cdf')
        report = render(args.results, prefix, args.method, cdf_only=args.cdf_only,
                        swap_sea_colors=args.swap_sea_colors, trim_top_left=args.trim_top_left)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__': main()
