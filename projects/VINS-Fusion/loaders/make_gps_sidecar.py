#!/usr/bin/env python3
"""Create RTKLIB GPS input on upstream VINS' every-second-image timestamps.

The existing large bag supplies only images and IMU during replay. This small
sidecar supplies /gps_baseline, remapped to /gps by the same rosbag player.
Position interpolation, reference conversion and uncertainty conventions are
shared with make_rosbag_vins.py; no camera or IMU data are rewritten.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
import rospy

import make_rosbag_vins as adapter


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rtklib', type=Path, default=adapter.RTKLIB_CSV)
    a = ap.parse_args()
    if a.out.exists(): ap.error('output already exists; choose a new path')
    prep = json.loads(Path(str(a.bag)+'.preparation.json').read_text())
    assert a.bag.stat().st_size == prep['bag_bytes']
    assert a.bag.stat().st_mtime_ns == prep['bag_mtime_ns']
    assert prep['downscale'] == 1
    adapter.RTKLIB_CSV = a.rtklib.resolve()
    with rosbag.Bag(str(a.bag)) as bag:
        indices = bag._get_indexes(bag._get_connections(['/cam0/image_raw']))
        camera = np.array(sorted(e.time.to_sec() for index in indices for e in index))
    assert len(camera) == prep['stereo_pairs'] and np.all(np.diff(camera)>0)
    admitted = camera[1::2]  # inputImageCnt increments before upstream modulo test
    messages = adapter.build_gps_msgs(admitted, prep['start_utc'], prep['end_utc'])
    a.out.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(a.out), 'w', compression=rosbag.Compression.LZ4) as bag:
        for t, _, msg in messages:
            bag.write('/gps_baseline', msg, rospy.Time.from_sec(t))
    times = np.array([t for t, _, _ in messages])
    assert np.all(np.diff(times)>0)
    assert all(np.min(abs(admitted-t))<1e-7 for t in times)
    with rosbag.Bag(str(a.out)) as bag:
        assert set(bag.get_type_and_topic_info().topics) == {'/gps_baseline'}
        loaded = list(bag.read_messages(topics=['/gps_baseline']))
        assert len(loaded) == len(messages)
        for (_, got, record), (t, _, expected) in zip(loaded, messages):
            assert abs(record.to_sec()-t)<1e-7
            assert abs(got.header.stamp.to_sec()-t)<1e-7
            assert (got.latitude,got.longitude,got.altitude) == (expected.latitude,expected.longitude,expected.altitude)
            assert np.array_equal(got.position_covariance,expected.position_covariance)
    report = dict(bag=str(a.out.resolve()),
        bag_bytes=a.out.stat().st_size, source_bag=str(a.bag.resolve()),
        source_bag_bytes=a.bag.stat().st_size,
        source_bag_mtime_ns=a.bag.stat().st_mtime_ns,
        rtklib=str(a.rtklib.resolve()),
        topic='/gps_baseline', remap='/gps_baseline:=/gps',
        estimator_image_stride=2, estimator_first_image_index=1,
        stereo_pairs=len(camera), estimator_image_candidates=len(admitted),
        gps_messages=len(messages), first_gps_utc=float(times[0]), last_gps_utc=float(times[-1]),
        all_gps_on_estimator_image_timestamps=True,
        original_bag_gps_excluded_from_playback=True,
        note='GPS positions are interpolated to the admitted image time; images and IMU retain their original UTC stamps.')
    Path(str(a.out)+'.preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__': main()
