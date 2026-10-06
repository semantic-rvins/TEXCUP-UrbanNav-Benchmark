#!/usr/bin/env python3
"""Verify received images, initialization exclusion, tracking and flushed navigation."""
import argparse
import json
from pathlib import Path
import re

import numpy as np
import rosbag

GPS_WEEK_UNIX=315964800+2052*604800-18


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    a=ap.parse_args()
    run=a.run.resolve()
    manifest=json.loads((run/'run_manifest.json').read_text())
    text=(run/'logs/icgvins.log').read_text(errors='replace')
    def times(kind):
        return np.array(list(map(float,re.findall(r'BENCHMARK_IC_'+kind+r' ([\d.]+)',text))))+GPS_WEEK_UNIX
    received,queued,initial,tracked=[times(k) for k in ['IMAGE_INPUT','FRAME_QUEUED','FRAME_INITIALIZING','TRACKED']]
    with rosbag.Bag(manifest['bag']) as bag:
        end=bag.get_end_time()+1 if manifest['diagnostic_duration'] is None else bag.get_start_time()+manifest['diagnostic_duration']
        def expected(topic):
            return np.array(sorted(e.time.to_sec() for index in bag._get_indexes(bag._get_connections([topic])) for e in index if e.time.to_sec()<end))
        images,imus=expected('/cam0/image_raw'),expected('/imu0')
    assert len(received)==len(images) and np.allclose(received,images,atol=2e-6,rtol=0),'image delivery incomplete'
    admitted=np.sort(np.concatenate([queued,initial]))
    assert len(admitted)==len(images) and np.allclose(admitted,images,atol=2e-6,rtol=0),'unaccounted input image'
    assert len(tracked)>0 and len(tracked)==len(queued) and np.allclose(tracked,queued,atol=2e-6,rtol=0),'tracking queue incomplete'
    assert np.all(np.diff(received)>0) and np.all(np.diff(tracked)>0)
    nav=np.atleast_2d(np.loadtxt(run/'gvins_output/gvins.nav'))
    assert nav.shape[1]>=11 and np.isfinite(nav).all() and np.all(np.diff(nav[:,1])>0)
    assert abs(nav[-1,1]+GPS_WEEK_UNIX-imus[-1])<.2,'navigation did not reach final IMU'
    assert 'GVINS has finished processing' in text and 'ROS node has been shutdown' in text
    features=np.array(list(map(int,re.findall(r'BENCHMARK_IC_TRACKED [\d.]+ (\d+)',text))))
    report=dict(images_received=len(received),expected_images=len(images),
        images_skipped_during_initialization=len(initial),images_queued=len(queued),images_tracked=len(tracked),
        all_input_images_accounted_for=True,all_queued_images_tracked=True,
        first_image_utc=float(received[0]),last_image_utc=float(received[-1]),first_tracked_utc=float(tracked[0]),
        last_tracked_utc=float(tracked[-1]),median_tracked_features=float(np.median(features)),
        tracking_lost_events=text.count('Tracking lost at '),navigation_rows=len(nav),
        gnss_states_retained=text.count('Retain GNSS-supported time node '),
        state_lookup_failure_count=text.count('Wrong matching time node '),
        first_nav_utc=float(nav[0,1]+GPS_WEEK_UNIX),last_nav_utc=float(nav[-1,1]+GPS_WEEK_UNIX),
        shutdown_complete=True,note='Coverage checks do not establish trajectory accuracy; all errors remain in the score.')
    (run/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
