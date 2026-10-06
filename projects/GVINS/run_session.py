#!/usr/bin/env python3
"""Supervise one headless GVINS replay; record progress, failures and evaluation.

Invoke through run_gvins.sh after building and preparing the input bag.
Each run needs its own output directory and ROS master port.
"""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import signal
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback
import xmlrpc.client


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--port', type=int, default=11951)
    ap.add_argument('--data-dir', type=Path,
                    default=Path(os.environ.get('TEXCUP_DATA', str(Path(__file__).resolve().parents[2]/'data/tex_cup'))))
    args = ap.parse_args()
    if args.rate <= 0:
        ap.error('--rate must be positive')
    project = Path(__file__).resolve().parent
    bench = project.parents[1]
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / 'status.json').exists():
        ap.error('output directory already contains a run; choose a new directory')
    if not args.bag.is_file():
        ap.error(f'input bag not found: {args.bag}')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', args.port)) == 0:
            ap.error(f'ROS master port {args.port} is already occupied')
    for name in ('logs', 'gvins_output', 'ros_home'):
        (out / name).mkdir(exist_ok=True)
    os.environ.update(ROS_MASTER_URI=f'http://localhost:{args.port}',
                      ROS_HOSTNAME='localhost', ROS_HOME=str(out / 'ros_home'),
                      ROS_LOG_DIR=str(out / 'logs/ros'))
    config_text = (project / 'config/gvins_texcup.yaml').read_text()
    config_text, replaced = re.subn(r'^output_dir:.*$',
        'output_dir: ' + json.dumps(str(out / 'gvins_output')),
        config_text, flags=re.MULTILINE)
    assert replaced == 1
    config = out / 'gvins_texcup.yaml'
    config.write_text(config_text)
    state = dict(phase='starting', supervisor_pid=os.getpid(), started_utc=now(),
                 run_dir=str(out), replay_rate=args.rate, ros_port=args.port,
                 warning_count=0, error_count=0, feature_count=0, pose_count=0,
                 fused_count=0, last_imu_utc=None, last_feature_utc=None,
                 last_pose_utc=None, last_fused_utc=None)
    lock = threading.RLock()
    children = {}
    logs = []
    warnings = (out / 'logs/warnings.log').open('a', buffering=1)
    errors = (out / 'logs/errors.log').open('a', buffering=1)
    events = (out / 'logs/progress.log').open('a', buffering=1)
    manifest = dict(started_utc=now(), bag=str(args.bag.resolve()),
        bag_bytes=args.bag.stat().st_size, replay_rate=args.rate,
        python=sys.executable, commands=[],
        evaluation=dict(cut_utc='18:09:40', end_utc='19:16:59', reference='ALT1'),
        patches=sorted(p.name for p in (project / 'patches').glob('*.patch')),
        source_revisions={name: subprocess.check_output(
            ['git', '-C', str(project / name), 'rev-parse', 'HEAD'], text=True).strip()
            for name in ('upstream', 'gnss_comm')})
    for name in ('input_validation.json', 'environment_explicit.txt'):
        original = bench / 'results/GVINS' / name
        if original.exists():
            shutil.copy2(original, out / name)
    manifest['thread_environment'] = {name: os.environ.get(name) for name in
        ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')}

    def save(phase=None, message=None):
        with lock:
            if phase:
                state['phase'] = phase
            if message:
                state['message'] = message
                events.write(f'{now()} [{state["phase"]}] {message}\n')
                print(message, flush=True)
            state['updated_utc'] = now()
            if state['last_feature_utc'] is not None and state['last_pose_utc'] is not None:
                state['feature_to_pose_lag_seconds'] = state['last_feature_utc'] - state['last_pose_utc']
            state['processes'] = {name: {'pid': p.pid, 'returncode': p.poll()}
                                  for name, p in children.items()}
            temp = out / 'status.json.tmp'
            temp.write_text(json.dumps(state, indent=2) + '\n')
            temp.replace(out / 'status.json')
            (out / 'run_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    def spawn(name, cmd):
        handle = (out / f'logs/{name}.log').open('w')
        logs.append(handle)
        manifest['commands'].append(dict(name=name, argv=cmd, started_utc=now()))
        child = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        children[name] = child
        save(message=f'Started {name}, PID {child.pid}')
        return child

    def stop(name):
        child = children.get(name)
        if child is None or child.poll() is not None:
            return
        for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            try:
                os.killpg(child.pid, sig)
                child.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                return

    # Bound XML-RPC health checks so a dead node cannot hang the supervisor.
    class Transport(xmlrpc.client.Transport):
        def make_connection(self, host):
            conn = super().make_connection(host)
            conn.timeout = 3
            return conn

    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'], transport=Transport())

    def node_alive(name):
        try:
            code, _, uri = master.lookupNode('/benchmark_supervisor', name)
            if code != 1:
                return False
            node = xmlrpc.client.ServerProxy(uri, transport=Transport())
            return node.getPid('/benchmark_supervisor')[0] == 1
        except (OSError, xmlrpc.client.Error):
            return False

    def healthy(names):
        for name in names:
            if children[name].poll() is not None:
                raise RuntimeError(f'{name} exited unexpectedly: {children[name].returncode}')
        for node in ('/gvins', '/gvins_feature_tracker'):
            if not node_alive(node):
                raise RuntimeError(f'Estimator node is no longer responding: {node}')

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f'signal {signum}')

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    rc = 1
    try:
        save(message='Starting isolated ROS master and recording')
        spawn('roscore', ['roscore', '-p', str(args.port)])
        deadline = time.monotonic() + 60
        while not node_alive('/rosout'):
            if time.monotonic() > deadline or children['roscore'].poll() is not None:
                raise RuntimeError('ROS master did not become ready')
            time.sleep(0.5)

        import rospy
        from sensor_msgs.msg import Imu, NavSatFix, PointCloud
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Log
        rospy.init_node('benchmark_monitor', disable_signals=True)

        def roslog(msg):
            line = f'{now()} [{msg.name}] {msg.msg}\n'
            with lock:
                if msg.level >= Log.ERROR:
                    errors.write(line)
                    state['error_count'] += 1
                elif msg.level >= Log.WARN:
                    warnings.write(line)
                    state['warning_count'] += 1

        def stamp(msg, key, count=None, offset=0):
            with lock:
                state[key] = msg.header.stamp.to_sec() - offset
                if count:
                    state[count] += 1

        subscriptions = [
            rospy.Subscriber('/rosout_agg', Log, roslog, queue_size=1000),
            rospy.Subscriber('/imu0', Imu, lambda m: stamp(m, 'last_imu_utc'), queue_size=10),
            rospy.Subscriber('/gvins_feature_tracker/feature', PointCloud,
                lambda m: stamp(m, 'last_feature_utc', 'feature_count'), queue_size=100),
            rospy.Subscriber('/gvins/odometry', Odometry,
                lambda m: stamp(m, 'last_pose_utc', 'pose_count'), queue_size=100),
            rospy.Subscriber('/gvins/gnss_fused_lla', NavSatFix,
                lambda m: stamp(m, 'last_fused_utc', 'fused_count', 18), queue_size=100)]
        spawn('record', ['rosbag', 'record', '--lz4', '--buffsize=256',
            '-O', str(out / 'gvins_out.bag'), '/gvins/gnss_fused_lla',
            '/gvins/gnss_anchor_lla', '/gvins/enu_pose', '/gvins/odometry',
            '/gvins/gnss_enu_path', '/gvins_feature_tracker/restart',
            '__name:=gvins_rec'])
        spawn('gvins', ['roslaunch', '--screen', str(project / 'config/texcup.launch'),
            f'config_path:={config}', f'gvins_path:={project / "upstream"}/'])
        deadline = time.monotonic() + 90
        while not all(node_alive(n) for n in ('/gvins', '/gvins_feature_tracker', '/gvins_rec')):
            if time.monotonic() > deadline or children['gvins'].poll() is not None:
                raise RuntimeError('GVINS nodes or recorder did not become ready')
            time.sleep(0.5)
        time.sleep(3)
        player = spawn('play', ['rosbag', 'play', str(args.bag.resolve()),
            '--rate', str(args.rate), '--queue=1000', '-d', '3', '--quiet'])
        save('running', 'Playback started; progress and ROS warnings/errors are monitored')
        while player.poll() is None:
            healthy(('roscore', 'gvins', 'record'))
            save()
            time.sleep(5)
        if player.returncode:
            raise RuntimeError(f'rosbag play failed: exit {player.returncode}')
        save('draining', 'Playback finished; waiting for estimator queues to drain')
        deadline = time.monotonic() + 120
        previous, stable = None, 0
        while time.monotonic() < deadline:
            healthy(('roscore', 'gvins', 'record'))
            time.sleep(5)
            current = (state['feature_count'], state['pose_count'], state['fused_count'])
            stable = stable + 1 if current == previous else 0
            previous = current
            save()
            if stable >= 3:
                break
        else:
            raise RuntimeError('Estimator is still processing 120 seconds after playback EOF')
        stop('record')
        stop('gvins')
        rospy.signal_shutdown('replay finished')
        for subscription in subscriptions:
            subscription.unregister()
        stop('roscore')
        if not state['fused_count']:
            raise RuntimeError('Replay produced no GNSS-fused positions')
        save('extracting', 'Converting antenna positions and evaluating the common window')
        extract = spawn('extract', [sys.executable, str(project / 'loaders/extract_est.py'),
            '--bag', str(out / 'gvins_out.bag'), '--out', str(out / 'est.csv')])
        if extract.wait():
            raise RuntimeError('Output extraction failed; see logs/extract.log')
        import numpy as np
        values = np.genfromtxt(out / 'est.csv', delimiter=',', names=True)
        if not values.size or not all(np.isfinite(values[k]).all() for k in values.dtype.names):
            raise RuntimeError('Extracted CSV contains no positions or non-finite values')
        evaluate = spawn('evaluate', [sys.executable, str(bench / 'common/evaluate.py'),
            '--est', str(out / 'est.csv'), '--out', str(out), '--name', 'GVINS',
            '--gt', str(args.data_dir.resolve() / 'ground_truth.log'),
            '--cut', '18:09:40', '--end', '19:16:59'])
        if evaluate.wait():
            raise RuntimeError('Evaluation failed; see logs/evaluate.log')
        manifest['finished_utc'] = now()
        save('completed', 'Replay, extraction and evaluation completed; review eval.json and logs')
        rc = 0
    except KeyboardInterrupt as ex:
        save('stopped', f'Stop requested: {ex}')
        rc = 130
    except Exception as ex:
        errors.write(f'{now()} [supervisor] {ex}\n{traceback.format_exc()}\n')
        with lock:
            state['error_count'] += 1
        save('error', str(ex))
    finally:
        for name in ('play', 'record', 'gvins', 'roscore', 'extract', 'evaluate'):
            stop(name)
        save()
        if 'rospy' in locals() and not rospy.is_shutdown():
            rospy.signal_shutdown('supervisor stopping')
        if 'subscriptions' in locals():
            for subscription in subscriptions:
                subscription.unregister()
        for handle in logs + [warnings, errors, events]:
            handle.close()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
