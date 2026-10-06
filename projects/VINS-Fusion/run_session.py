#!/usr/bin/env python3
"""Run stereo VINS-Fusion with RTKLIB global fusion and automatic verification/scoring."""
import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
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
    ap.add_argument('--gps-bag', type=Path, help='GPS sidecar prepared for upstream alternate-frame admission')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rate', type=float, default=1)
    ap.add_argument('--port', type=int, default=11952)
    ap.add_argument('--data-dir', type=Path,
                    default=Path(os.environ.get('TEXCUP_DATA', str(Path(__file__).resolve().parents[2]/'data/tex_cup'))))
    ap.add_argument('--duration', type=float, help='diagnostic replay duration; never collected')
    a = ap.parse_args()
    project = Path(__file__).resolve().parent
    bench, out = project.parents[1], a.out.resolve()
    if a.rate <= 0 or (a.duration is not None and a.duration <= 0): ap.error('rate/duration must be positive')
    if (out/'status.json').exists(): ap.error('choose a fresh output directory')
    prep = json.loads(Path(str(a.bag)+'.preparation.json').read_text())
    validation = json.loads((bench/'results/VINS-Fusion/input_validation.json').read_text())
    assert str(a.bag.resolve()) == validation['bag']
    assert a.bag.stat().st_size == validation['bag_bytes'] == prep['bag_bytes']
    assert a.bag.stat().st_mtime_ns == validation['bag_mtime_ns'] == prep['bag_mtime_ns']
    assert prep['downscale'] == 1 and all(validation['checks'].values())
    a.gps_bag = (a.gps_bag or project/'bags/texcup_gps_upstream.bag').resolve()
    gps_prep = json.loads(Path(str(a.gps_bag)+'.preparation.json').read_text())
    assert gps_prep['source_bag'] == str(a.bag.resolve())
    assert gps_prep['source_bag_bytes'] == a.bag.stat().st_size
    assert gps_prep['source_bag_mtime_ns'] == a.bag.stat().st_mtime_ns
    assert gps_prep['bag_bytes'] == a.gps_bag.stat().st_size
    assert gps_prep['estimator_image_stride'] == 2
    assert gps_prep['all_gps_on_estimator_image_timestamps']
    patch_names = (project/'patches/series').read_text().splitlines()
    assert patch_names and all(n and '/' not in n and n.endswith('.patch') for n in patch_names)
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', a.port)) == 0: ap.error('ROS port is occupied')
    for folder in ('logs','vins_output','ros_home'): (out/folder).mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable,str(project/'tests/verify_patch_series.py'),
                    '--out',str(out/'source_verification.json')],check=True)
    (out/'supervisor.pid').write_text(str(os.getpid())+'\n')
    os.environ.update(ROS_MASTER_URI=f'http://localhost:{a.port}', ROS_HOSTNAME='localhost',
                      ROS_HOME=str(out/'ros_home'), ROS_LOG_DIR=str(out/'logs/ros'))
    text, n = re.subn(r'^output_path:.*$', 'output_path: '+json.dumps(str(out/'vins_output')+'/'),
                     (project/'config/texcup_stereo_config.yaml').read_text(), flags=re.M)
    assert n == 1
    config = out/'texcup_stereo_config.yaml'
    config.write_text(text)
    for name in ('texcup_cam0.yaml','texcup_cam1.yaml'): shutil.copy2(project/'config'/name, out/name)
    for name in ('input_validation.json','environment_explicit.txt'):
        shutil.copy2(bench/'results/VINS-Fusion'/name, out/name)
    (out/'preparation.json').write_text(json.dumps(prep, indent=2)+'\n')
    (out/'gps_preparation.json').write_text(json.dumps(gps_prep, indent=2)+'\n')
    for name in ('build.log','rotation_regression.log','extraction_tests.log','patch_verification.json'):
        original = bench/'results/VINS-Fusion/baseline_preparation'/name
        if original.is_file(): shutil.copy2(original, out/'logs'/('baseline_'+name))
    smoke_evidence = bench/'results/VINS-Fusion/baseline_preparation/smoke_120s/verification.json'
    if smoke_evidence.is_file():
        shutil.copy2(smoke_evidence,out/'logs/baseline_startup_verification.json')
    (out/'upstream.diff').write_bytes(subprocess.check_output(['git','-C',str(project/'upstream'),'diff']))
    manifest = dict(started_utc=now(), source_revision=subprocess.check_output(
        ['git','-C',str(project/'upstream'),'rev-parse','HEAD'], text=True).strip(),
        bag=str(a.bag.resolve()), bag_bytes=prep['bag_bytes'],
        replay_rate=a.rate, diagnostic_duration=a.duration, mode='stereo + IMU + global_fusion(RTKLIB)', commands=[],
        source_scope='upstream algorithm settings with build, diagnostics, and unit-rotation software fixes',
        gps_bag=str(a.gps_bag),
        estimator_image_stride=2, estimator_first_image_index=1,
        evaluation=dict(cut_utc='18:09:40',end_utc='19:16:59',reference='ALT1'),
        patch_series=patch_names,
        thread_environment={k:os.environ.get(k) for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','OPENCV_FOR_THREADS_NUM']})
    expected_pairs = prep['stereo_pairs']
    if a.duration is not None:
        import rosbag
        with rosbag.Bag(str(a.bag)) as bag:
            end = bag.get_start_time()+a.duration
            expected_pairs = sum(e.time.to_sec()<end for index in
                bag._get_indexes(bag._get_connections(['/cam0/image_raw'])) for e in index)
    state = dict(phase='starting', started_utc=now(), supervisor_pid=os.getpid(), run_dir=str(out),
        replay_rate=a.rate, ros_port=a.port, warning_count=0, error_count=0, image_pairs_processed=0,
        vio_count=0, global_count=0, gps_accepted=0, reset_count=0, expected_image_pairs=expected_pairs,
        expected_estimator_image_candidates=expected_pairs//2,
        estimator_cadence='tracks every pair; admits every second pair before initialization losses',
        invalid_rotation_count=0, max_vio_quaternion_norm_error=0., max_global_quaternion_norm_error=0.,
        last_imu_utc=None,last_image_utc=None,last_vio_utc=None,last_global_utc=None)
    lock, children, handles = threading.RLock(), {}, []
    warnings = (out/'logs/warnings.log').open('a',buffering=1)
    errors = (out/'logs/errors.log').open('a',buffering=1)
    events = (out/'logs/progress.log').open('a',buffering=1)
    measurements = (out/'logs/measurements.log').open('a',buffering=1)

    def save(phase=None,message=None):
        with lock:
            if phase: state['phase'] = phase
            if message:
                state['message'] = message
                events.write(f'{now()} [{state["phase"]}] {message}\n')
                print(message,flush=True)
            state['updated_utc'] = now()
            state['processes'] = {n:dict(pid=p.pid,returncode=p.poll()) for n,p in children.items()}
            if state['last_vio_utc'] is not None and state['last_image_utc'] is not None:
                state['image_to_vio_lag_s'] = state['last_image_utc']-state['last_vio_utc']
            p = out/'status.json.tmp'
            p.write_text(json.dumps(state,indent=2)+'\n')
            p.replace(out/'status.json')
            (out/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

    def spawn(name,cmd):
        handle = (out/f'logs/{name}.log').open('w')
        handles.append(handle)
        manifest['commands'].append(dict(name=name,argv=cmd,started_utc=now()))
        child = subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        children[name] = child
        save(message=f'Started {name}, PID {child.pid}')
        return child

    def stop(name):
        p = children.get(name)
        if p is None or p.poll() is not None: return
        # vins_node leaves its sync thread joinable when ros::spin returns.
        # After draining and closing the recorder, terminate that node directly.
        signals = ((signal.SIGTERM,5),(signal.SIGKILL,5)) if name=='vins' else ((signal.SIGINT,20),(signal.SIGTERM,5),(signal.SIGKILL,5))
        for sig,wait in signals:
            try:
                os.killpg(p.pid,sig)
                p.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired: pass
            except ProcessLookupError: return

    class Transport(xmlrpc.client.Transport):
        def make_connection(self,host):
            conn = super().make_connection(host)
            conn.timeout = 3
            return conn
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'],transport=Transport())

    def node_alive(name):
        try:
            code,_,uri = master.lookupNode('/benchmark_monitor',name)
            return code == 1 and xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid('/benchmark_monitor')[0] == 1
        except (OSError,xmlrpc.client.Error): return False

    def healthy():
        if state['invalid_rotation_count']:
            raise RuntimeError('Invalid output rotation detected; see logs/errors.log')
        for name in ('roscore','vins','global_fusion','record'):
            if children[name].poll() is not None: raise RuntimeError(f'{name} exited unexpectedly: {children[name].returncode}')
        for name in ('/vins_estimator','/globalEstimator'):
            if not node_alive(name): raise RuntimeError(f'{name} stopped responding')

    def interrupted(signum,_): raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    rc = 1
    try:
        save(message='Starting isolated ROS master')
        spawn('roscore',['roscore','-p',str(a.port)])
        deadline = time.monotonic()+60
        while not node_alive('/rosout'):
            if time.monotonic()>deadline or children['roscore'].poll() is not None: raise RuntimeError('ROS master startup failed')
            time.sleep(.5)
        import rospy
        from sensor_msgs.msg import Imu
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Log
        rospy.init_node('benchmark_monitor',disable_signals=True)

        def roslog(msg):
            line = f'{now()} [{msg.name}] {msg.msg}\n'
            with lock:
                if msg.msg.startswith('BENCHMARK_'): measurements.write(line)
                if msg.level >= Log.ERROR:
                    errors.write(line)
                    state['error_count'] += 1
                elif msg.level >= Log.WARN:
                    warnings.write(line)
                    state['warning_count'] += 1
                if msg.msg.startswith('BENCHMARK_VINS_IMAGE '):
                    _,t,features,stereo = msg.msg.split()
                    state['image_pairs_processed'] += 1
                    state.update(last_image_utc=float(t),tracked_features=int(features),stereo_features=int(stereo))
                elif msg.msg.startswith('BENCHMARK_GPS_ACCEPTED '): state['gps_accepted'] += 1
                elif 'failure detection!' in msg.msg: state['reset_count'] += 1

        def stamp(msg,key,count=None):
            with lock:
                state[key] = msg.header.stamp.to_sec()
                if count: state[count] += 1
                if count in ('vio_count','global_count'):
                    q = msg.pose.pose.orientation
                    error = abs(math.sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z)-1.)
                    field = 'max_vio_quaternion_norm_error' if count=='vio_count' else 'max_global_quaternion_norm_error'
                    state[field] = max(state[field],error) if math.isfinite(error) and state[field] is not None else None
                    if not math.isfinite(error) or error>1e-5:
                        state['invalid_rotation_count'] += 1
                        if state['invalid_rotation_count']==1:
                            errors.write(f'{now()} [rotation] {count} UTC {msg.header.stamp.to_sec()} quaternion norm error {error}\n')
        subscriptions = [rospy.Subscriber('/rosout_agg',Log,roslog,queue_size=2000),
            rospy.Subscriber('/imu0',Imu,lambda m:stamp(m,'last_imu_utc'),queue_size=10),
            rospy.Subscriber('/vins_estimator/odometry',Odometry,lambda m:stamp(m,'last_vio_utc','vio_count'),queue_size=1000),
            rospy.Subscriber('/globalEstimator/global_odometry',Odometry,lambda m:stamp(m,'last_global_utc','global_count'),queue_size=1000)]
        spawn('record',['rosbag','record','--lz4','--buffsize=256','-O',str(out/'vinsfusion_out.bag'),
            '/vins_estimator/odometry','/globalEstimator/global_odometry','__name:=vins_rec'])
        spawn('vins',['stdbuf','-oL','-eL','rosrun','vins','vins_node',str(config)])
        spawn('global_fusion',['stdbuf','-oL','-eL','rosrun','global_fusion','global_fusion_node',f'_output_csv:={out / "vio_global.csv"}'])
        deadline = time.monotonic()+90
        while not all(node_alive(n) for n in ('/vins_estimator','/globalEstimator','/vins_rec')):
            if time.monotonic()>deadline or any(children[n].poll() is not None for n in ('vins','global_fusion','record')):
                raise RuntimeError('Estimator/recorder startup failed')
            time.sleep(.5)
        time.sleep(3)
        cmd = ['rosbag','play',str(a.bag.resolve()),str(a.gps_bag),
               '--rate',str(a.rate),'--queue=1000','-d','3','--quiet',
               '--topics','/imu0','/cam0/image_raw','/cam1/image_raw','/gps_baseline',
               '/gps_baseline:=/gps']
        if a.duration is not None: cmd += ['--duration',str(a.duration)]
        player = spawn('play',cmd)
        save('running','Playback started; tracked stereo pairs and ROS errors are monitored')
        previous,progressed = None,time.monotonic()
        while player.poll() is None:
            healthy()
            current = state['image_pairs_processed']
            if current != previous: previous,progressed = current,time.monotonic()
            if time.monotonic()-progressed>120: raise RuntimeError('No image-processing progress for 120 seconds')
            save()
            time.sleep(5)
        if player.returncode: raise RuntimeError(f'rosbag play failed: {player.returncode}')
        save('draining','Playback finished; waiting for VIO and global fusion to drain')
        deadline,previous,stable = time.monotonic()+600,None,0
        while time.monotonic()<deadline:
            healthy()
            time.sleep(5)
            current = (state['image_pairs_processed'],state['vio_count'],state['global_count'])
            stable = stable+1 if current == previous else 0
            previous = current
            save()
            if stable>=6: break
        else: raise RuntimeError('Estimator queues did not drain in 600 seconds')
        stop('record')
        stop('vins')
        stop('global_fusion')
        rospy.signal_shutdown('replay finished')
        for sub in subscriptions: sub.unregister()
        stop('roscore')
        if not state['global_count']: raise RuntimeError('No fused output')
        save('verifying','Checking actual image processing, GPS association and output support')
        if spawn('verify',[sys.executable,str(project/'loaders/verify_run.py'),'--run',str(out)]).wait():
            raise RuntimeError('Runtime verification failed; see logs/verify.log')
        verification = json.loads((out/'verification.json').read_text())
        state['image_pairs_processed'] = verification['image_pairs_processed']
        save('extracting','Converting globally aligned body poses to ALT1 antenna ECEF')
        cmd = [sys.executable,str(project/'loaders/extract_est.py'),'--bag',str(out/'vinsfusion_out.bag'),
               '--anchor-log',str(out/'logs/global_fusion.log'),'--after-utc',str(verification['first_global_optimization_utc']),
               '--out',str(out/'est.csv')]
        if spawn('extract',cmd).wait(): raise RuntimeError('Extraction failed')
        cmd = [sys.executable,str(bench/'common/evaluate.py'),'--est',str(out/'est.csv'),'--out',str(out),
               '--name','VINS-Fusion','--gt',str(a.data_dir.resolve()/'ground_truth.log'),'--cut','18:09:40','--end','19:16:59']
        if spawn('evaluate',cmd).wait(): raise RuntimeError('Evaluation failed')
        if spawn('statistics',[sys.executable,str(bench/'common/summarize_statistics.py'),'--results',str(out)]).wait():
            raise RuntimeError('Statistics failed')
        manifest['finished_utc'] = now()
        save('completed','Replay, verification and statistics completed')
        if a.duration is None:
            cmd = [sys.executable,str(bench/'common/calculate_statistics.py'),'--results',str(bench/'results/final'),
                   '--gt',str(a.data_dir.resolve()/'ground_truth.log'),'--collect',f'VINS-Fusion={out}']
            if spawn('collect_final',cmd).wait(): raise RuntimeError('Final results collection failed')
            save('completed','Replay and statistics completed; collected into results/final')
        rc = 0
    except KeyboardInterrupt as ex:
        save('stopped',str(ex))
        rc = 130
    except Exception as ex:
        errors.write(f'{now()} [supervisor] {ex}\n{traceback.format_exc()}\n')
        state['error_count'] += 1
        save('error',str(ex))
    finally:
        for name in reversed(list(children)): stop(name)
        save()
        if 'rospy' in locals() and not rospy.is_shutdown(): rospy.signal_shutdown('supervisor stopping')
        for handle in handles+[warnings,errors,events,measurements]: handle.close()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
