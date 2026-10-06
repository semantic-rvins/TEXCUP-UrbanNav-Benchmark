#!/usr/bin/env python3
"""Verify actual tracked stereo frames, camera/GPS association and pose output."""
import argparse
import json
from pathlib import Path
import re

import numpy as np
import rosbag


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True)
    a = ap.parse_args()
    run = a.run.resolve()
    manifest = json.loads((run/'run_manifest.json').read_text())
    duration = manifest['diagnostic_duration']
    vlog = (run/'logs/vins.log').read_text(errors='replace')
    glog = (run/'logs/global_fusion.log').read_text(errors='replace')
    mpath = run/'logs/measurements.log'
    telemetry = mpath.read_text() if mpath.exists() else vlog+'\n'+glog
    frames = np.array([list(map(float,m)) for m in re.findall(
        r'BENCHMARK_VINS_IMAGE ([\d.]+) (\d+) (\d+)',telemetry)])
    assert len(frames) and np.all(np.diff(frames[:,0])>0), 'missing/repeated frame telemetry'
    with rosbag.Bag(manifest['bag']) as bag:
        indexes = bag._get_indexes(bag._get_connections(['/cam0/image_raw']))
        expected = np.array(sorted(e.time.to_sec() for index in indexes for e in index))
        if duration is not None: expected = expected[expected < bag.get_start_time()+duration]
    admitted = expected[manifest['estimator_first_image_index']::manifest['estimator_image_stride']]
    with rosbag.Bag(manifest['gps_bag']) as bag:
        gps_indices = bag._get_indexes(bag._get_connections(['/gps_baseline']))
        gps_stamps = np.array(sorted(e.time.to_sec() for index in gps_indices for e in index))
    assert len(frames)==len(expected), f'processed {len(frames)} of {len(expected)} image pairs'
    assert np.allclose(frames[:,0],expected,atol=1e-6,rtol=0), 'tracked image timestamps do not match input'
    assert np.count_nonzero(frames[:,2]>0) > .9*len(frames), 'stereo matches missing in most frames'
    gps = np.array([list(map(float,m)) for m in re.findall(r'BENCHMARK_GPS_ACCEPTED ([\d.]+) ([\d.]+)',telemetry)])
    assert len(gps) and np.all(abs(gps[:,0]-gps[:,1])<=.01)
    assert np.all(np.diff(gps[:,0])>0)
    assert all(np.min(abs(gps_stamps-t))<1e-6 for t in gps[:,1])
    assert all(np.min(abs(admitted-t))<1e-6 for t in gps[:,0]), 'GPS was associated to an unadmitted image'
    optimizations = [list(map(float,m)) for m in re.findall(
        r'BENCHMARK_GLOBAL_OPTIMIZED ([\d.]+) (\d+) (\d+) (\d+)',glog)]
    assert optimizations and optimizations[0][3]==1, 'global alignment did not initialize'
    topics = {}
    with rosbag.Bag(str(run/'vinsfusion_out.bag')) as bag:
        for topic in ['/vins_estimator/odometry','/globalEstimator/global_odometry']:
            rows = []
            for _,msg,_ in bag.read_messages(topics=[topic]):
                p,q = msg.pose.pose.position,msg.pose.pose.orientation
                rows.append([msg.header.stamp.to_sec(),p.x,p.y,p.z,q.w,q.x,q.y,q.z])
            rows = np.asarray(rows)
            assert len(rows) and np.isfinite(rows).all() and np.all(np.diff(rows[:,0])>0)
            norm_error = float(np.max(abs(np.linalg.norm(rows[:,4:8],axis=1)-1)))
            assert norm_error<1e-5, f'{topic}: maximum quaternion norm error {norm_error}; global/body rotations must have unit norm'
            assert all(np.min(abs(admitted-t))<1e-6 for t in rows[:,0]), 'pose timestamp was not admitted by upstream frame cadence'
            assert abs(rows[-1,0]-admitted[-1])<1e-6, 'output did not reach the final estimator-admitted image'
            topics[topic] = dict(count=len(rows),first_utc=float(rows[0,0]),last_utc=float(rows[-1,0]),
                                 max_quaternion_norm_error=norm_error)
    report = dict(image_pairs_processed=len(frames),expected_image_pairs=len(expected),
        estimator_image_candidates=len(admitted), estimator_image_stride=manifest['estimator_image_stride'],
        all_pose_times_match_estimator_admission=True,
        all_expected_pairs_tracked=True,first_image_utc=float(frames[0,0]),last_image_utc=float(frames[-1,0]),
        median_features=float(np.median(frames[:,1])),median_stereo_features=float(np.median(frames[:,2])),
        pairs_with_stereo_features=int(np.count_nonzero(frames[:,2])),gps_accepted=len(gps),
        global_optimization_cycles=len(optimizations),unusable_optimization_cycles=sum(m[3]!=1 for m in optimizations),
        first_global_optimization_utc=optimizations[0][0],
        global_initialization_exclusion='Only odometry strictly after the first completed global alignment is georeferenced.',
        resets=vlog.count('failure detection!'),topics=topics,
        note='Checks establish data processing and output support; they do not establish position accuracy.')
    (run/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
