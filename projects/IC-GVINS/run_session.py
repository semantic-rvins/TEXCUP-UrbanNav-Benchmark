#!/usr/bin/env python3
"""Run isolated IC-GVINS, monitor native tracking/navigation, verify and collect results."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
import traceback
import xmlrpc.client

GPS_WEEK_UNIX=315964800+2052*604800-18


def now():return datetime.now(timezone.utc).isoformat()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--data-dir',type=Path,
                    default=Path(os.environ.get('TEXCUP_DATA',str(Path(__file__).resolve().parents[2]/'data/tex_cup'))))
    ap.add_argument('--rate',type=float,default=1)
    ap.add_argument('--port',type=int,default=11961)
    ap.add_argument('--duration',type=float,help='diagnostic replay duration; never collected')
    a=ap.parse_args()
    project=Path(__file__).resolve().parent
    bench,out=project.parents[1],a.out.resolve()
    if a.rate<=0 or (a.duration is not None and a.duration<=0):ap.error('rate/duration must be positive')
    if (out/'status.json').exists():ap.error('choose a fresh output directory')
    prep=json.loads(Path(str(a.bag)+'.preparation.json').read_text())
    val=json.loads((bench/'results/IC-GVINS/input_validation.json').read_text())
    assert str(a.bag.resolve())==val['bag']
    assert a.bag.stat().st_size==val['bag_bytes']==prep['bag_bytes']
    assert a.bag.stat().st_mtime_ns==val['bag_mtime_ns']==prep['bag_mtime_ns']
    assert all(val['checks'].values())
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',a.port))==0:ap.error('ROS port is occupied')
    for f in ['logs','gvins_output','ros_home']:(out/f).mkdir(parents=True,exist_ok=True)
    subprocess.run([sys.executable,str(project/'loaders/verify_source.py'),
                    '--out',str(out/'source_verification.json')],check=True)
    (out/'supervisor.pid').write_text(str(os.getpid())+'\n')
    os.environ.update(ROS_MASTER_URI=f'http://localhost:{a.port}',ROS_HOSTNAME='localhost',
                      ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'logs/ros'))
    import yaml
    config_data=yaml.safe_load((project/'config/icgvins_texcup.yaml').read_text())
    config_data['outputpath']=str(out/'gvins_output')
    config=out/'icgvins_texcup.yaml'
    config.write_text(yaml.safe_dump(config_data,sort_keys=False))
    for f in ['input_validation.json','environment_explicit.txt']:
        shutil.copy2(bench/'results/IC-GVINS'/f,out/f)
    for f in (bench/'results/IC-GVINS/logs').glob('*'):
        if f.name in ['build.log','build_retry.log','configure.log','shared_libraries.log','converter_tests.log']:
            shutil.copy2(f,out/'logs'/f.name)
    for f in (bench/'results/IC-GVINS/time_bounds_validation').glob('*'):
        if f.name in ['build.log','patched.log','original.log','patch_verification.json']:
            shutil.copy2(f,out/'logs'/('time_bounds_'+f.name))
    for f in (bench/'results/IC-GVINS/baseline_preparation').glob('*'):
        if f.name in ['build.log','bounds_tests.log','extraction_tests.log','gnss_state_tests.log',
                      'prior_pointer_tests.log','feature_grid_tests.log','feature_velocity_tests.log']:
            shutil.copy2(f,out/'logs'/('baseline_'+f.name))
    (out/'preparation.json').write_text(json.dumps(prep,indent=2)+'\n')
    (out/'upstream.diff').write_bytes(subprocess.check_output(['git','-C',str(project/'upstream'),'diff']))
    cpus=sorted(os.sched_getaffinity(0))[-4:]
    manifest=dict(started_utc=now(),bag=str(a.bag.resolve()),bag_bytes=prep['bag_bytes'],
        replay_rate=a.rate,diagnostic_duration=a.duration,allowed_node_cpus=cpus,
        source_revision=subprocess.check_output(['git','-C',str(project/'upstream'),'rev-parse','HEAD'],text=True).strip(),
        patches=sorted(p.name for p in (project/'patches').glob('*.patch')),
        implementation_profile='Upstream initialization and GNSS weighting; diagnostics, time-window bounds, state/prior lifetime, feature-grid bounds, and Eigen temporary-lifetime fixes',
        archived_algorithm_patches_applied=False,
        evaluation=dict(cut_utc='18:09:40',end_utc='19:16:59',reference='ALT1'),commands=[],
        thread_environment={k:os.environ.get(k) for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','OPENCV_FOR_THREADS_NUM']})
    expected=prep['counts']['img']
    if a.duration is not None:
        import rosbag
        with rosbag.Bag(str(a.bag)) as bag:
            end=bag.get_start_time()+a.duration
            expected=sum(e.time.to_sec()<end for index in bag._get_indexes(bag._get_connections(['/cam0/image_raw'])) for e in index)
    state=dict(phase='starting',started_utc=now(),supervisor_pid=os.getpid(),run_dir=str(out),ros_port=a.port,
        replay_rate=a.rate,expected_images=expected,images_received=0,images_queued=0,images_initializing=0,
        images_tracked=0,nav_count=0,error_count=0,warning_count=0,tracking_lost_count=0,keyframes_skipped=0,
        gnss_states_retained=0,state_lookup_failure_count=0,
        last_image_utc=None,last_tracked_utc=None,last_nav_utc=None)
    children,handles,readers,pending={},{},{},{}
    errors=(out/'logs/errors.log').open('a',buffering=1)
    warnings=(out/'logs/warnings.log').open('a',buffering=1)
    progress=(out/'logs/progress.log').open('a',buffering=1)

    def save(phase=None,message=None):
        if phase:state['phase']=phase
        if message:
            state['message']=message
            progress.write(f'{now()} [{state["phase"]}] {message}\n')
            print(message,flush=True)
        state['updated_utc']=now()
        state['processes']={name:dict(pid=p.pid,returncode=p.poll()) for name,p in children.items()}
        if state['last_image_utc'] is not None and state['last_tracked_utc'] is not None:
            state['image_to_tracking_lag_s']=state['last_image_utc']-state['last_tracked_utc']
        p=out/'status.json.tmp'
        p.write_text(json.dumps(state,indent=2)+'\n')
        p.replace(out/'status.json')
        (out/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

    def spawn(name,cmd):
        h=(out/f'logs/{name}.log').open('w')
        handles[name]=h
        manifest['commands'].append(dict(name=name,argv=cmd,started_utc=now()))
        p=subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
        children[name]=p
        save(message=f'Started {name}, PID {p.pid}')
        return p

    def stop(name):
        p=children.get(name)
        if p is None or p.poll() is not None:return
        for sig,timeout in [(signal.SIGINT,45),(signal.SIGTERM,5),(signal.SIGKILL,5)]:
            try:
                os.killpg(p.pid,sig)
                p.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:pass
            except ProcessLookupError:return

    class Transport(xmlrpc.client.Transport):
        def make_connection(self,host):
            conn=super().make_connection(host)
            conn.timeout=3
            return conn
    master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'],transport=Transport())

    def node_alive(name):
        try:
            code,_,uri=master.lookupNode('/benchmark_supervisor',name)
            return code==1 and xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid('/benchmark_supervisor')[0]==1
        except (OSError,xmlrpc.client.Error):return False

    def healthy():
        for name in ['roscore','icgvins']:
            if children[name].poll() is not None:raise RuntimeError(f'{name} exited unexpectedly: {children[name].returncode}')
        if not node_alive('/ic_gvins_node'):raise RuntimeError('IC-GVINS node stopped responding')

    def poll_native():
        for name,path in [('icgvins',out/'logs/icgvins.log'),('nav',out/'gvins_output/gvins.nav')]:
            if not path.exists():continue
            if name not in readers:
                readers[name]=path.open(errors='replace')
                pending[name]=''
            parts=(pending[name]+readers[name].read()).split('\n')
            pending[name]=parts.pop()
            for line in parts:
                line=re.sub(r'\x1b\[[0-9;]*m','',line)
                if name=='nav':
                    row=line.split()
                    if len(row)>=11:
                        state['nav_count']+=1
                        state['last_nav_utc']=float(row[1])+GPS_WEEK_UNIX
                    continue
                if re.match(r'^[EF]\d',line) or '[ERROR]' in line or '[FATAL]' in line:
                    errors.write(line+'\n');state['error_count']+=1
                elif re.match(r'^W\d',line) or '[WARN]' in line:
                    warnings.write(line+'\n');state['warning_count']+=1
                if 'Tracking lost at ' in line:state['tracking_lost_count']+=1
                if 'Skip keyframe outside the ordered IMU state window at ' in line:state['keyframes_skipped']+=1
                if 'Retain GNSS-supported time node ' in line:state['gnss_states_retained']+=1
                if 'Wrong matching time node ' in line:state['state_lookup_failure_count']+=1
                m=re.search(r'BENCHMARK_IC_(IMAGE_INPUT|FRAME_QUEUED|FRAME_INITIALIZING|TRACKED) ([\d.]+)(.*)',line)
                if m:
                    kind,t,extra=m.groups()
                    count={'IMAGE_INPUT':'images_received','FRAME_QUEUED':'images_queued','FRAME_INITIALIZING':'images_initializing','TRACKED':'images_tracked'}[kind]
                    state[count]+=1
                    if kind=='IMAGE_INPUT':state['last_image_utc']=float(t)+GPS_WEEK_UNIX
                    if kind=='TRACKED':
                        state['last_tracked_utc']=float(t)+GPS_WEEK_UNIX
                        state['tracked_features']=int(extra.split()[0])

    def interrupted(sig,_):raise KeyboardInterrupt(f'signal {sig}')
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    rc=1
    try:
        save(message='Starting isolated ROS master')
        spawn('roscore',['roscore','-p',str(a.port)])
        deadline=time.monotonic()+60
        while not node_alive('/rosout'):
            if time.monotonic()>deadline:raise RuntimeError('ROS master startup failed')
            time.sleep(.5)
        spawn('icgvins',['taskset','-c',','.join(map(str,cpus)),'stdbuf','-oL','-eL','rosrun','ic_gvins','ic_gvins_ros',
            '__name:=ic_gvins_node',f'_configfile:={config}','_imu_topic:=/imu0','_gnss_topic:=/gnss0','_image_topic:=/cam0/image_raw'])
        deadline=time.monotonic()+90
        while not node_alive('/ic_gvins_node'):
            if time.monotonic()>deadline or children['icgvins'].poll() is not None:raise RuntimeError('IC-GVINS startup failed')
            time.sleep(.5)
        time.sleep(3)
        cmd=['rosbag','play',str(a.bag.resolve()),'--rate',str(a.rate),'--queue=1000','-d','3','--quiet']
        if a.duration is not None:cmd+=['--duration',str(a.duration)]
        player=spawn('play',cmd)
        save('running','Playback started; actual image tracking and native errors are monitored')
        previous,last_change=None,time.monotonic()
        while player.poll() is None:
            healthy();poll_native()
            current=(state['images_received'],state['images_tracked'],state['nav_count'])
            if current!=previous:previous,last_change=current,time.monotonic()
            if time.monotonic()-last_change>120:raise RuntimeError('No input or estimator progress for 120 seconds')
            save();time.sleep(5)
        if player.returncode:raise RuntimeError(f'rosbag play failed: {player.returncode}')
        save('draining','Playback ended; waiting for tracking and navigation queues to drain')
        previous,stable,deadline=None,0,time.monotonic()+600
        while time.monotonic()<deadline:
            healthy();time.sleep(5);poll_native()
            current=(state['images_received'],state['images_tracked'],state['nav_count'])
            stable=stable+1 if current==previous else 0
            previous=current;save()
            if stable>=6:break
        else:raise RuntimeError('Processing did not drain in 600 seconds')
        if state['images_received']!=expected or state['images_tracked']!=state['images_queued']:
            raise RuntimeError('Image stream or tracking queue incomplete at EOF')
        stop('icgvins');poll_native();stop('roscore')
        if children['icgvins'].returncode!=0:raise RuntimeError(f'IC-GVINS shutdown failed: {children["icgvins"].returncode}')
        save('verifying','Verifying frame coverage and complete navigation output')
        if spawn('verify',[sys.executable,str(project/'loaders/verify_run.py'),'--run',str(out)]).wait():raise RuntimeError('Runtime verification failed')
        save('extracting','Converting IMU navigation to ALT1 antenna ECEF and calculating statistics')
        if spawn('extract',[sys.executable,str(project/'loaders/extract_est.py'),'--nav',str(out/'gvins_output/gvins.nav'),'--out',str(out/'est.csv')]).wait():raise RuntimeError('Extraction failed')
        if spawn('evaluate',[sys.executable,str(bench/'common/evaluate.py'),'--est',str(out/'est.csv'),'--out',str(out),'--name','IC-GVINS',
            '--gt',str(a.data_dir.resolve()/'ground_truth.log'),'--cut','18:09:40','--end','19:16:59']).wait():raise RuntimeError('Evaluation failed')
        if spawn('statistics',[sys.executable,str(bench/'common/summarize_statistics.py'),'--results',str(out)]).wait():raise RuntimeError('Statistics failed')
        manifest['finished_utc']=now()
        save('completed','Replay, verification and statistics completed')
        if a.duration is None:
            if spawn('collect_final',[sys.executable,str(bench/'common/calculate_statistics.py'),'--results',str(bench/'results/final'),
                '--gt',str(a.data_dir.resolve()/'ground_truth.log'),'--collect',f'IC-GVINS={out}']).wait():raise RuntimeError('Final collection failed')
            save('completed','Replay and statistics completed; collected into results/final')
        rc=0
    except KeyboardInterrupt as ex:
        save('stopped',str(ex));rc=130
    except Exception as ex:
        errors.write(f'{now()} [supervisor] {ex}\n{traceback.format_exc()}\n')
        state['error_count']+=1
        save('error',str(ex))
    finally:
        for name in reversed(list(children)):stop(name)
        poll_native();save()
        for handle in list(handles.values())+list(readers.values())+[errors,warnings,progress]:handle.close()
    return rc


if __name__=='__main__':raise SystemExit(main())
