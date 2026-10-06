#!/usr/bin/env python3
"""Run TEX-CUP GNSS-only GICI-RTK, supervise EOF, extract and score results.

Activate benchmark_gici first. All paths are resolved into a new run directory.
Upstream stays alive after post-file EOF: terminate it only after every input
file has been read and the solution has remained unchanged for 30 seconds.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parent
BENCH = PROJECT.parents[1]


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def observation_summary(path):
    epochs, header = [], []
    in_header = True
    with path.open() as stream:
        for line in stream:
            if in_header:
                header.append(line)
                if 'END OF HEADER' in line:
                    in_header = False
            elif line.startswith('>'):
                p = line.split()
                epochs.append(int(p[4])*3600 + int(p[5])*60 + float(p[6]))
    if not epochs or np.any(np.diff(epochs) <= 0):
        raise ValueError(f'Empty or non-increasing observations: {path}')
    return dict(path=str(path), n_epochs=len(epochs),
                first_gpst_sec=epochs[0], last_gpst_sec=epochs[-1],
                first_utc_sec=epochs[0]-18, last_utc_sec=epochs[-1]-18,
                observation_types=[s.strip() for s in header if 'SYS / # / OBS TYPES' in s],
                base_position_header=[s.strip() for s in header if 'APPROX POSITION XYZ' in s])


def input_read_positions(pid, paths):
    positions = {}
    for fd in Path(f'/proc/{pid}/fd').iterdir():
        try:
            target = fd.resolve()
            if target in paths:
                info = Path(f'/proc/{pid}/fdinfo/{fd.name}').read_text()
                pos = int(re.search(r'^pos:\s*(\d+)', info, re.M)[1])
                positions[str(target)] = dict(position=pos, size=target.stat().st_size)
        except (OSError, TypeError):
            continue
    return positions


def solution_summary(path):
    quality = Counter()
    times = []
    if path.exists():
        for line in path.read_text(errors='replace').splitlines():
            if not line.startswith('$GPGGA,') or '*' not in line:
                continue
            fields = line.split(',')
            if fields[1] and fields[6]:
                s = fields[1]
                times.append(int(s[:2])*3600 + int(s[2:4])*60 + float(s[4:]))
                quality[fields[6]] += 1
    return dict(n_gga=len(times), quality_counts=dict(quality),
                last_solution_utc_sec=times[-1] if times else None,
                last_solution_utc=(f'{int(times[-1]//3600):02d}:'
                                   f'{int(times[-1]%3600//60):02d}:{times[-1]%60:06.3f}') if times else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--data-dir', default=os.environ.get('TEXCUP_DATA', str(BENCH/'data/tex_cup')))
    parser.add_argument('--binary', default=str(PROJECT/'upstream/build-benchmark/gici_main'))
    args = parser.parse_args()
    out, data, binary = Path(args.out).resolve(), Path(args.data_dir).resolve(), Path(args.binary).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out/'run_manifest.json').exists() or (out/'solution.nmea').exists():
        raise SystemExit('Use a new output directory; an existing run will not be overwritten')
    logs, inputs = out/'logs', out/'inputs'
    logs.mkdir(exist_ok=True)
    inputs.mkdir(exist_ok=True)
    status = dict(phase='preparing', supervisor_pid=os.getpid(), started_utc=utc_now(), run_dir=str(out))
    manifest = dict(name='GICI-RTK', estimator_type='rtk', sensors=['rover GNSS', 'base GNSS', 'broadcast nav'],
                    started_utc=status['started_utc'], evaluation_window_utc=['18:09:40', '19:16:59'],
                    ground_truth_reference='ALT1 antenna', lever_arm_applied=False, commands=[])
    process = None

    def update(phase, **fields):
        status.update(phase=phase, updated_utc=utc_now(), **fields)
        write_json(out/'status.json', status)

    def command(argv, log_name):
        manifest['commands'].append(list(map(str, argv)))
        with (logs/log_name).open('w') as log:
            subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, check=True, cwd=BENCH)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    update('preparing')
    try:
        if not binary.is_file():
            raise FileNotFoundError(f'Build GICI first: {binary}')
        manifest['source_revision'] = subprocess.check_output(
            ['git', '-C', str(PROJECT/'upstream'), 'rev-parse', 'HEAD'], text=True).strip()
        (out/'upstream.diff').write_bytes(subprocess.check_output(
            ['git', '-C', str(PROJECT/'upstream'), 'diff', 'HEAD']))
        command(['ldd', str(binary)], 'shared_libraries.log')
        manifest['inputs'] = {}
        for original, cut in [('asterx4_rover.obs', 'rover_cut.obs'), ('asterx4_base_1hz.obs', 'base_cut.obs')]:
            manifest['inputs'][original] = observation_summary(data/original)
            command([sys.executable, str(PROJECT/'loaders/cut_rinex.py'), '--in', str(data/original),
                     '--out', str(inputs/cut), '--cut-gpst', '2019-05-09 18:09:58'], f'cut_{cut}.log')
            meta = observation_summary(inputs/cut)
            if meta['first_gpst_sec'] != 65398 or meta['observation_types'] != manifest['inputs'][original]['observation_types']:
                raise ValueError(f'Unexpected RINEX time cut or header: {cut}')
            manifest['inputs'][cut] = meta
        for name in ('brdm1290.19p', 'ground_truth.log'):
            manifest['inputs'][name] = dict(path=str(data/name))
        config = yaml.safe_load((PROJECT/'config/gici_rtk_dualfreq_post.yaml').read_text())
        paths = {'str_gnss_rov_file': inputs/'rover_cut.obs', 'str_gnss_ref_file': inputs/'base_cut.obs',
                 'str_gnss_eph_file': data/'brdm1290.19p', 'str_rtk_solution_file': out/'solution.nmea'}
        for entry in config['stream']['streamers']:
            s = entry['streamer']
            s['path'] = str(paths[s['tag']])
        estimator = config['estimate'][0]['estimator']
        assert estimator['type'] == 'rtk' and len(estimator['input_tags']) == 3
        assert estimator['gnss_estimator_base_options']['gnss_common']['system_exclude'] == ['R']
        estimator['estimator_base_options'].update(log_intermediate_data=True,
                                                   log_intermediate_data_directory=str(logs))
        config['logging']['file_directory'] = str(logs)
        config_path = out/'gici_rtk.yaml'
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        env = os.environ.copy()
        env.update(OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', PYTHONNOUSERSITE='1')
        manifest['thread_environment'] = {key: env[key] for key in
                                          ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')}
        argv = [str(binary), str(config_path)]
        manifest['commands'].append(argv)
        write_json(out/'run_manifest.json', manifest)
        with (logs/'gici.log').open('w') as log:
            process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            previous_size, last_change = -1, time.monotonic()
            tracked_inputs = {paths[tag] for tag in ('str_gnss_rov_file', 'str_gnss_ref_file', 'str_gnss_eph_file')}
            update('running', gici_pid=process.pid)
            while True:
                time.sleep(5)
                if process.poll() is not None:
                    raise RuntimeError(f'GICI exited before supervised EOF: {process.returncode}; see logs/gici.log')
                sol = out/'solution.nmea'
                size = sol.stat().st_size if sol.exists() else 0
                if size != previous_size:
                    previous_size, last_change = size, time.monotonic()
                idle = time.monotonic()-last_change
                positions = input_read_positions(process.pid, tracked_inputs)
                eof = len(positions) == len(tracked_inputs) and all(p['position'] >= p['size'] for p in positions.values())
                progress = solution_summary(sol)
                update('running', **progress, solution_bytes=size, idle_seconds=round(idle, 1),
                       all_inputs_read_to_eof=eof, input_read_positions=positions)
                if eof and size and idle >= 30:
                    manifest['termination_reason'] = 'All input file descriptors at EOF; solution unchanged for 30 seconds'
                    manifest['final_replay_progress'] = dict(progress, input_read_positions=positions)
                    update('finalizing', message=manifest['termination_reason'])
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
                    manifest['gici_returncode'] = process.returncode
                    break
                if idle > 600:
                    raise RuntimeError('Replay stalled for 10 minutes before verified EOF')
        text = (logs/'gici.log').read_text(errors='replace')
        for name, pattern in [('warnings', r'^W\d{4}.*$'), ('errors', r'^[EF]\d{4}.*$')]:
            matches = re.findall(pattern, text, re.M)
            (logs/f'{name}.log').write_text('\n'.join(matches) + ('\n' if matches else ''))
            manifest[name+'_count'] = len(matches)
        command([sys.executable, str(PROJECT/'loaders/gici_nmea_to_csv.py'), '--nmea', str(out/'solution.nmea'),
                 '--out', str(out/'est.csv')], 'extract.log')
        exported = np.genfromtxt(out/'est.csv', delimiter=',', names=True)
        for name in ('utc_sec', 'lat_deg', 'lon_deg', 'h_ell'):
            if not np.isfinite(exported[name]).all():
                raise ValueError(f'Nonfinite exported {name}')
        if np.any(np.diff(exported['utc_sec']) <= 0):
            raise ValueError('Non-increasing output timestamps')
        command([sys.executable, str(BENCH/'common/evaluate.py'), '--est', str(out/'est.csv'), '--out', str(out),
                 '--gt', str(data/'ground_truth.log'), '--name', 'GICI-RTK', '--cut', '18:09:40', '--end', '19:16:59'], 'evaluate.log')
        command([sys.executable, str(BENCH/'common/summarize_statistics.py'), '--results', str(out)], 'statistics.log')
        metrics = json.loads((out/'eval.json').read_text())
        if metrics['n_window'] != 4040:
            raise ValueError('Expected 4,040 GT epochs')
        manifest['finished_utc'] = utc_now()
        write_json(out/'run_manifest.json', manifest)
        update('completed', message='Replay EOF verified; extraction, evaluation and statistics completed',
               n_window=metrics['n_window'], n_solved=metrics['n_solved'],
               availability_pct=metrics['availability_pct'], gici_returncode=manifest['gici_returncode'],
               warnings_count=manifest['warnings_count'], errors_count=manifest['errors_count'])
        print(json.dumps(json.loads((out/'statistics.json').read_text())['metrics'], indent=2))
    except BaseException as exc:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        manifest.update(finished_utc=utc_now(), error=str(exc))
        write_json(out/'run_manifest.json', manifest)
        update('stopped' if isinstance(exc, KeyboardInterrupt) else 'error', message=str(exc))
        raise


if __name__ == '__main__':
    main()
