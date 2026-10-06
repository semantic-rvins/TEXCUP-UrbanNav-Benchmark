#!/usr/bin/env python3
"""Run the prepared TEX-CUP RRR profile and publish final antenna results.

Activate benchmark_gici. Build upstream-rrr with both saved patches and run
loaders/verify_rrr_inputs.py first. See RUN_INSTRUCTIONS.md for exact commands.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

from run_rtk import PROJECT, BENCH, utc_now, write_json, observation_summary, input_read_positions, solution_summary

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--data-dir', default=os.environ.get('TEXCUP_DATA', str(BENCH/'data/tex_cup')))
    parser.add_argument('--preparation', default=str(BENCH/'results/GICI-RRR/preparation.json'))
    parser.add_argument('--final-results', default=str(BENCH/'results/final'))
    args = parser.parse_args()
    out, data = Path(args.out).resolve(), Path(args.data_dir).resolve()
    source = PROJECT/'upstream-rrr'
    binary = source/'build/gici_main'
    out.mkdir(parents=True, exist_ok=True)
    if (out/'run_manifest.json').exists() or (out/'solution.nmea').exists():
        raise SystemExit('Use a new output directory')
    logs, inputs = out/'logs', out/'inputs'
    logs.mkdir(exist_ok=True)
    inputs.mkdir(exist_ok=True)
    status = dict(phase='preparing', started_utc=utc_now(), supervisor_pid=os.getpid(), run_dir=str(out))
    manifest = dict(name='GICI-RRR', estimator_type='rtk_imu_camera_rrr', started_utc=status['started_utc'],
        sensors=['rover GNSS', 'base GNSS', 'broadcast nav', 'RFU Lord IMU', '2048x732 port mono camera'],
        ground_truth_reference='ALT1 antenna', lever_arm_rfu_m=[-0.610, -0.052, 0.010],
        evaluation_window_utc=['18:09:40', '19:16:59'], commands=[])
    process = None
    warnings = errors = 0

    def update(phase, **fields):
        status.update(phase=phase, updated_utc=utc_now(), **fields)
        write_json(out/'status.json', status)

    def command(argv, filename):
        manifest['commands'].append(list(map(str, argv)))
        write_json(out/'run_manifest.json', manifest)
        with (logs/filename).open('w') as log:
            subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, check=True, cwd=BENCH)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    update('preparing')
    try:
        preparation = json.loads(Path(args.preparation).read_text())
        for key in ('imu', 'images'):
            info = preparation['files'][key]
            path = Path(info['path'])
            if (path.stat().st_size, path.stat().st_mtime_ns) != (info['bytes'], info['mtime_ns']):
                raise ValueError(f'{key} changed since input validation; validate again')
        write_json(out/'preparation.json', preparation)
        for filename in ('environment_explicit.txt',):
            path = BENCH/'results/GICI-RRR'/filename
            if path.exists(): shutil.copy2(path, out/filename)
        for filename in ('build.log', 'configure.log', 'input_validation.log', 'converter_tests.log'):
            path = BENCH/'results/GICI-RRR/logs'/filename
            if path.exists(): shutil.copy2(path, logs/filename)
        manifest['source_revision'] = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
        (out/'upstream.diff').write_bytes(subprocess.check_output(['git', '-C', str(source), 'diff', 'HEAD']))
        manifest['inputs'] = preparation['files'].copy()
        # GNSS must stop while IMU still brackets its final epoch. Keep one second
        # of IMU padding after the common scoring endpoint; it is not scored.
        manifest['input_window'] = dict(gnss_start_utc='18:09:40', gnss_end_utc='19:16:59',
                                       imu_end_utc='19:17:00', imu_end_padding_s=1.0)
        imu_path = inputs/'imu_rfu.txt'
        imu_times = []
        with Path(preparation['files']['imu']['path']).open() as source_imu, imu_path.open('w') as target_imu:
            target_imu.write(next(source_imu))
            for line in source_imu:
                timestamp = float(line.split()[0])
                if timestamp > 1557429420.0: break
                target_imu.write(line)
                imu_times.append(timestamp)
        assert imu_times and imu_times[-1] > 1557429419.0
        manifest['inputs']['run_imu'] = dict(path=str(imu_path),
                                            n_epochs=len(imu_times), first_utc_unix=imu_times[0],
                                            last_utc_unix=imu_times[-1])
        for original, cut in [('asterx4_rover.obs','rover_cut.obs'), ('asterx4_base_1hz.obs','base_cut.obs')]:
            manifest['inputs'][original] = observation_summary(data/original)
            command([sys.executable, str(PROJECT/'loaders/cut_rinex.py'), '--in', str(data/original),
                     '--out', str(inputs/cut), '--cut-gpst', '2019-05-09 18:09:58',
                     '--end-gpst', '2019-05-09 19:17:17'], f'cut_{cut}.log')
            manifest['inputs'][cut] = observation_summary(inputs/cut)
            assert manifest['inputs'][cut]['first_utc_sec'] == 65380
            assert manifest['inputs'][cut]['last_utc_sec'] == 69419
            assert manifest['inputs'][cut]['n_epochs'] == 4040
        for filename in ('brdm1290.19p', 'ground_truth.log'):
            manifest['inputs'][filename] = dict(path=str(data/filename))
        config = yaml.safe_load((PROJECT/'config/gici_rrr_mono_dualfreq_post.yaml').read_text())
        paths = {'str_gnss_rov_file':inputs/'rover_cut.obs', 'str_gnss_ref_file':inputs/'base_cut.obs',
                 'str_gnss_eph_file':data/'brdm1290.19p', 'str_imu_file':imu_path,
                 'str_camera_file':Path(preparation['files']['images']['path']), 'str_rrr_solution_file':out/'solution.nmea'}
        for entry in config['stream']['streamers']:
            entry['streamer']['path'] = str(paths[entry['streamer']['tag']])
        estimator = config['estimate'][0]['estimator']
        assert estimator['type']=='rtk_imu_camera_rrr'
        assert estimator['gnss_estimator_base_options']['gnss_common']['system_exclude']==['R']
        assert estimator['gnss_imu_initializer_options']['gnss_extrinsics']==[-0.610,-0.052,0.010]
        estimator['estimator_base_options'].update(log_intermediate_data=True, log_intermediate_data_directory=str(logs))
        config['logging']['file_directory'] = str(logs)
        (out/'gici_rrr.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        env = os.environ.copy()
        env.update(OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', PYTHONNOUSERSITE='1')
        manifest['thread_environment'] = {k:env[k] for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS')}
        command(['ldd', str(binary)], 'shared_libraries.log')
        argv = [str(binary), str(out/'gici_rrr.yaml')]
        manifest['commands'].append(argv)
        write_json(out/'run_manifest.json', manifest)
        eof, origins, warnings, errors = False, [], 0, 0
        last_size, last_input = -1, None
        last_change = last_input_change = time.monotonic()
        with (logs/'gici.log').open('w') as stdout, (logs/'gici.log').open() as reader, \
                (logs/'warnings.log').open('w') as warning_log, (logs/'errors.log').open('w') as error_log:
            process = subprocess.Popen(argv, stdout=stdout, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            update('running', gici_pid=process.pid)
            partial_line = ''
            while True:
                time.sleep(5)
                chunk = partial_line + reader.read()
                lines = chunk.split('\n')
                partial_line = lines.pop()
                for line in lines:
                    if re.match(r'^W\d{4}', line):
                        warnings += 1; warning_log.write(line+'\n')
                    if re.match(r'^[EF]\d{4}', line):
                        errors += 1; error_log.write(line+'\n')
                    if 'BENCHMARK_POST_FILES_EOF' in line: eof = True
                    if 'BENCHMARK_ENU_ORIGIN ' in line:
                        values = list(map(float, line.split('BENCHMARK_ENU_ORIGIN ')[1].split()))
                        assert len(values)==4
                        origins.append(dict(utc_unix=values[0], ecef_m=values[1:]))
                warning_log.flush(); error_log.flush()
                if process.poll() is not None:
                    raise RuntimeError(f'GICI exited before supervised EOF: {process.returncode}; see logs/gici.log')
                sol = out/'solution.nmea'
                size = sol.stat().st_size if sol.exists() else 0
                if size != last_size: last_size, last_change = size, time.monotonic()
                positions = input_read_positions(process.pid, set(paths.values())-{sol})
                current_input = {p:v['position'] for p,v in positions.items()}
                if current_input != last_input: last_input, last_input_change = current_input, time.monotonic()
                progress = solution_summary(sol)
                idle = time.monotonic()-last_change
                update('running', **progress, solution_bytes=size, idle_seconds=round(idle,1),
                       warnings_count=warnings, errors_count=errors, replay_eof=eof,
                       input_read_positions=positions, enu_origins=origins)
                if eof and size and idle>=30:
                    manifest.update(termination_reason='Logged post-file EOF and 30 seconds unchanged output',
                                    final_replay_progress=progress, enu_origins=origins,
                                    warnings_count=warnings, errors_count=errors)
                    update('finalizing', message=manifest['termination_reason'])
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
                    manifest['gici_returncode'] = process.returncode
                    break
                if min(idle, time.monotonic()-last_input_change)>600:
                    raise RuntimeError('No input or output progress for 10 minutes before logged EOF')
        if not origins or any(not np.allclose(o['ecef_m'], origins[0]['ecef_m'], atol=1e-6, rtol=0) for o in origins):
            raise ValueError('Missing or changing ENU origin; cannot apply one fixed lever rotation')
        spec=importlib.util.spec_from_file_location('gici_nmea', PROJECT/'loaders/gici_nmea_to_csv.py')
        converter=importlib.util.module_from_spec(spec); spec.loader.exec_module(converter)
        lla=converter.ecef2lla(origins[0]['ecef_m'])
        manifest['enu_origin_lla_deg_m'] = list(lla)
        write_json(out/'enu_origin.json', dict(source='GICI initial SPP origin log', **origins[0], lla_deg_m=list(lla)))
        command([sys.executable, str(PROJECT/'loaders/gici_nmea_to_csv.py'), '--nmea', str(out/'solution.nmea'),
                 '--out', str(out/'est.csv'), '--lever-arm=-0.610,-0.052,0.010',
                 '--enu-origin-lla='+','.join(format(v,'.17g') for v in lla)], 'extract.log')
        command([sys.executable, str(BENCH/'common/evaluate.py'), '--est', str(out/'est.csv'), '--out', str(out),
                 '--gt', str(data/'ground_truth.log'), '--name', 'GICI-RRR', '--cut', '18:09:40', '--end', '19:16:59'], 'evaluate.log')
        command([sys.executable, str(BENCH/'common/summarize_statistics.py'), '--results', str(out)], 'statistics.log')
        manifest['finished_utc'] = utc_now()
        write_json(out/'run_manifest.json', manifest)
        update('completed', message='Replay, antenna extraction and statistics completed; collecting final results')
        command([sys.executable, str(BENCH/'common/calculate_statistics.py'), '--results', args.final_results,
                 '--gt', str(data/'ground_truth.log'), '--collect', f'GICI-RRR={out}'], 'collect_final.log')
        update('completed', message='Replay, antenna extraction, statistics and results/final collection completed')
        print((out/'statistics.json').read_text())
    except BaseException as exc:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        manifest.update(error=str(exc), finished_utc=utc_now(), warnings_count=warnings,
                        errors_count=errors, gici_returncode=process.returncode if process else None)
        write_json(out/'run_manifest.json', manifest)
        update('stopped' if isinstance(exc, KeyboardInterrupt) else 'error', message=str(exc),
               warnings_count=warnings, errors_count=errors)
        raise


if __name__=='__main__':
    main()
