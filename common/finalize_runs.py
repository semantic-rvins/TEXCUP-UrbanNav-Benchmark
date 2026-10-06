#!/usr/bin/env python3
"""Collect stopped VINS-Fusion/IC-GVINS runs, including explicitly marked failures.

Native files and status.json are never rewritten. Recovery uses the existing
antenna converters on a validated copy of native observations, and records every
exclusion without filtering finite positioning errors against ground truth.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np

from calculate_statistics import calculate, validate_trajectory

ROOT = Path(__file__).resolve().parents[1]
HEADER = 'utc_sec,ecef_x,ecef_y,ecef_z,qw,qx,qy,qz\n'


def load_converter(method):
    path = ROOT/'projects'/method/'loaders/extract_est.py'
    spec = importlib.util.spec_from_file_location('recovery_'+method.replace('-', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return path, module


def process_exists(pid):
    try:
        stat = Path(f'/proc/{int(pid)}/stat').read_text()
        return stat.rsplit(')', 1)[1].split()[0] not in ('Z', 'X')
    except (FileNotFoundError, ProcessLookupError):
        return False


def stopped_status(run):
    status = json.loads((run/'status.json').read_text())
    active = []
    if status.get('supervisor_pid') and process_exists(status['supervisor_pid']):
        active.append('supervisor')
    for name, process in status.get('processes', {}).items():
        if process.get('returncode') is None and process.get('pid') and process_exists(process['pid']):
            active.append(name)
    if active:
        raise RuntimeError(f'Refusing to finalize active run {run}: {", ".join(active)}')
    phase = status.get('phase', 'unknown')
    terminal = ('completed', 'error', 'failed', 'stopped', 'interrupted', 'cancelled', 'aborted')
    return status, phase if phase in terminal else 'supervisor_exited'


def convert(command, work):
    result = subprocess.run(command, capture_output=True, text=True)
    (work/'conversion.log').write_text(result.stdout+'\n'+result.stderr)
    if result.returncode:
        raise RuntimeError(f'Native output conversion failed; see {work}/conversion.log')


def recover_ic(run, work):
    converter_path, converter = load_converter('IC-GVINS')
    native = run/'gvins_output/gvins.nav'
    counts = Counter()
    report = {'native_source': str(native), 'converter': str(converter_path)}
    filtered = work/'usable_gvins.nav'
    last_time = -math.inf
    with filtered.open('w') as output:
        if native.exists():
            with native.open(errors='replace') as source:
                for line in source:
                    if not line.strip():
                        continue
                    counts['native_records_read'] += 1
                    fields = line.split()
                    if len(fields) < 11:
                        counts['malformed_record'] += 1
                        continue
                    try:
                        sow, lat, lon, height, roll, pitch, yaw = map(float,
                            [fields[i] for i in (1, 2, 3, 4, 8, 9, 10)])
                    except ValueError:
                        counts['malformed_record'] += 1
                        continue
                    if not np.isfinite([sow, lat, lon, height, roll, pitch, yaw]).all():
                        counts['nonfinite_required_values'] += 1
                        continue
                    if not -90 <= lat <= 90:
                        counts['invalid_geodetic_latitude'] += 1
                        continue
                    utc = float(f'{sow-converter.DOW_THURSDAY-converter.LEAP:.3f}')
                    if utc <= last_time:
                        counts['nonincreasing_output_timestamp'] += 1
                        continue
                    rotation = converter.euler2R(*map(math.radians, (roll, pitch, yaw)))
                    antenna = converter.lla2ecef(lat, lon, height) + converter.R_ecef_ned(lat, lon) @ (
                        rotation @ converter.ANTLEVER_FRD)
                    if not np.isfinite(antenna).all():
                        counts['nonfinite_converted_position'] += 1
                        continue
                    output.write(line if line.endswith('\n') else line+'\n')
                    last_time = utc
                    counts['usable_records'] += 1
        else:
            report['native_read_error'] = 'Native navigation file is absent'
    trajectory = work/'est.csv'
    if counts['usable_records']:
        convert([sys.executable, str(converter_path), '--nav', str(filtered), '--out', str(trajectory)], work)
    else:
        trajectory.write_text(HEADER)
    report['counts'] = dict(counts)
    return report


def open_output_bag(native, work, report):
    import rosbag
    try:
        return rosbag.Bag(str(native))
    except rosbag.ROSBagUnindexedException:
        # The recorder may have been interrupted before its index was written.
        # Reindex only a copy; never repair or rename the original recording.
        recovery = work/'reindexed_native.bag'
        shutil.copy2(native, recovery)
        report['reindexed_copy'] = str(recovery)
        try:
            with rosbag.Bag(str(recovery), 'a', allow_unindexed=True) as bag:
                for _ in bag.reindex():
                    pass
        except (rosbag.ROSBagException, OSError, EOFError) as error:
            report['reindex_error'] = str(error)
        return rosbag.Bag(str(recovery))


def recover_vins(run, work):
    import rosbag
    converter_path, converter = load_converter('VINS-Fusion')
    native = run/'vinsfusion_out.bag'
    if not native.exists() and (run/'vinsfusion_out.bag.active').exists():
        native = run/'vinsfusion_out.bag.active'
    log = run/'logs/global_fusion.log'
    report = {'native_source': str(native), 'converter': str(converter_path)}
    counts = Counter()
    anchor = None
    first_usable = None
    if log.exists():
        try:
            anchor = converter.read_anchor(log)
            if not np.isfinite(anchor).all() or not -90 <= anchor[0] <= 90:
                anchor = None
        except (RuntimeError, ValueError):
            pass
        for match in re.finditer(r'BENCHMARK_GLOBAL_OPTIMIZED ([\d.]+) (\d+) (\d+) (\d+)',
                                 log.read_text(errors='replace')):
            if int(match[4]) == 1:
                first_usable = float(match[1])
                break
    report['anchor_lla'] = anchor
    report['first_usable_global_optimization_utc'] = first_usable
    report['quaternion_norm_tolerance'] = 1e-4  # Existing converter contract.
    filtered = work/'usable_global_odometry.bag'
    last_time = -math.inf
    with rosbag.Bag(str(filtered), 'w') as output:
        if native.exists():
            try:
                with open_output_bag(native, work, report) as source:
                    for topic, message, stamp in source.read_messages(topics=['/globalEstimator/global_odometry']):
                        counts['native_records_read'] += 1
                        if anchor is None or first_usable is None:
                            counts['missing_global_reference'] += 1
                            continue
                        q, p = message.pose.pose.orientation, message.pose.pose.position
                        timestamp = message.header.stamp.to_sec()
                        values = [timestamp, p.x, p.y, p.z, q.w, q.x, q.y, q.z]
                        if not np.isfinite(values).all():
                            counts['nonfinite_required_values'] += 1
                            continue
                        if timestamp <= first_usable:
                            counts['before_global_alignment'] += 1
                            continue
                        norm = math.hypot(q.w, q.x, q.y, q.z)
                        if abs(norm-1) > 1e-4:
                            counts['invalid_rotation'] += 1
                            continue
                        if timestamp <= last_time:
                            counts['nonincreasing_output_timestamp'] += 1
                            continue
                        rotation = converter.quat_to_rot(q.w, q.x, q.y, q.z)
                        antenna = converter.lla2ecef(*anchor) + converter.r_ecef_from_enu(*anchor[:2]) @ (
                            np.array([p.x, p.y, p.z]) + rotation @ converter.T_IMU_TO_ANT_IN_B)
                        if not np.isfinite(antenna).all():
                            counts['nonfinite_converted_position'] += 1
                            continue
                        output.write(topic, message, stamp)
                        last_time = timestamp
                        counts['usable_records'] += 1
            except (rosbag.ROSBagException, OSError, EOFError) as error:
                report['native_read_error'] = str(error)
                report['unreadable_tail_record_count'] = None
        else:
            report['native_read_error'] = 'Native output bag is absent'
    trajectory = work/'est.csv'
    if counts['usable_records']:
        convert([sys.executable, str(converter_path), '--bag', str(filtered), '--anchor-log', str(log),
                 '--after-utc', str(first_usable), '--out', str(trajectory)], work)
    else:
        trajectory.write_text(HEADER)
    report['counts'] = dict(counts)
    return report


def finalize_one(method, run):
    if method not in ('VINS-Fusion', 'IC-GVINS'):
        raise ValueError(f'Unsupported recovery method: {method}')
    run = Path(run).resolve()
    status, run_status = stopped_status(run)
    original_status = (run/'status.json').read_bytes()
    report = dict(method=method, source_run=str(run),
        finalized_utc=datetime.now(timezone.utc).isoformat(), original_phase=status.get('phase'),
        run_status=run_status, all_processes_exited=True,
        scoring='All 4040 epochs; missing or unconvertible output is unavailable; finite outliers retained')
    if run_status == 'completed':
        trajectory = run/'est.csv'
        report['trajectory_rows'] = validate_trajectory(trajectory)
        report['recovery'] = 'Completed-run antenna trajectory retained unchanged'
    else:
        work = run/'finalization'
        work.mkdir(exist_ok=True)
        recovery = recover_vins(run, work) if method == 'VINS-Fusion' else recover_ic(run, work)
        report['recovery'] = recovery
        trajectory = work/'est.csv'
        report['trajectory_rows'] = validate_trajectory(trajectory, allow_empty=True)
    report['trajectory_file'] = str(trajectory.relative_to(run))
    if (run/'status.json').read_bytes() != original_status:
        raise RuntimeError(f'Run status changed during finalization: {run}')
    temporary = run/'finalization.json.tmp'
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    temporary.replace(run/'finalization.json')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--run', action='append', required=True, metavar='METHOD=RUN_DIR')
    parser.add_argument('--gt', type=Path)
    args = parser.parse_args()
    runs = [specification.split('=', 1) for specification in args.run]
    if len({method for method, _ in runs}) != len(runs):
        parser.error('Each method may be finalized only once')
    # Check every run before writing any recovery output.
    for _, run in runs:
        stopped_status(Path(run).resolve())
    reports = [finalize_one(method, run) for method, run in runs]
    calculate(args.results, args.run, args.gt, allow_incomplete=True)
    print(json.dumps({'runs': reports, 'results': str(args.results.resolve())}, indent=2))


if __name__ == '__main__':
    main()
