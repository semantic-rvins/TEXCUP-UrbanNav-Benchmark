#!/usr/bin/env python3
"""Check stereo coverage, native pixels, UTC/RFU samples and RTKLIB GPS inputs."""
import argparse
import json
from pathlib import Path

import cv2
import h5py
import numpy as np
import rosbag
import rospy
import yaml

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    prep = json.loads(Path(str(a.bag)+'.preparation.json').read_text())
    assert a.bag.stat().st_size == prep['bag_bytes']
    assert a.bag.stat().st_mtime_ns == prep['bag_mtime_ns']
    data = Path(prep['data_dir'])
    report = dict(bag=str(a.bag.resolve()), bag_bytes=a.bag.stat().st_size,
                  bag_mtime_ns=a.bag.stat().st_mtime_ns, topics={}, checks={})
    stamps, samples = {}, {}
    with rosbag.Bag(str(a.bag)) as bag:
        for topic, meta in bag.get_type_and_topic_info().topics.items():
            indices = bag._get_indexes(bag._get_connections([topic]))
            ts = np.array(sorted(e.time.to_sec() for index in indices for e in index))
            assert len(ts) == meta.message_count == prep['counts'][topic]
            assert np.isfinite(ts).all() and np.all(np.diff(ts) > 0)
            stamps[topic] = ts
            report['topics'][topic] = dict(count=len(ts), first_utc=float(ts[0]), last_utc=float(ts[-1]))
            samples[topic] = []
            for t in ts[[0, len(ts)//2, -1]]:
                _, msg, record = next(bag.read_messages(topics=[topic], start_time=rospy.Time.from_sec(float(t)-1e-6)))
                assert abs(record.to_sec()-t) < 1e-6 and abs(msg.header.stamp.to_sec()-t) < 1e-6
                samples[topic].append((t, msg))
        assert set(stamps) == {'/imu0', '/cam0/image_raw', '/cam1/image_raw', '/gps'}
        assert np.array_equal(stamps['/cam0/image_raw'], stamps['/cam1/image_raw'])
        with h5py.File(data/'camera_data/images.h5', 'r') as h5:
            keys = [k for k in sorted(h5['port']) if prep['start_utc'] <= float(h5['port'][k].attrs['tstamp']) <= prep['end_utc']]
            ts = np.array([float(h5['port'][k].attrs['tstamp']) for k in keys])
            star_ts = np.array([float(h5['star'][k].attrs['tstamp']) for k in keys])
            assert len(keys) == 40372 and np.allclose(ts, stamps['/cam0/image_raw'], atol=1e-6, rtol=0)
            assert max(abs(ts-star_ts)) < .003
            report['max_raw_stereo_skew_s'] = float(max(abs(ts-star_ts)))
            for topic, group in [('/cam0/image_raw', 'port'), ('/cam1/image_raw', 'star')]:
                for t, msg in samples[topic]:
                    i = int(np.argmin(abs(ts-t)))
                    assert (msg.width, msg.height, msg.step, msg.encoding) == (2048, 732, 2048, 'mono8')
                    assert np.array_equal(np.frombuffer(msg.data, np.uint8).reshape(732, 2048), h5[group][keys[i]][()])
        # Every GPS message must be on an actual paired-image timestamp.
        for _, msg, record in bag.read_messages(topics=['/gps']):
            t = record.to_sec()
            assert abs(t-msg.header.stamp.to_sec()) < 1e-6
            assert np.min(abs(stamps['/cam0/image_raw']-t)) < 1e-6
            assert np.isfinite([msg.latitude, msg.longitude, msg.altitude]).all()
            assert msg.position_covariance[0] in (.05, .75)
            assert msg.position_covariance == (msg.position_covariance[0], 0, 0, 0, msg.position_covariance[0], 0, 0, 0, msg.position_covariance[0])
    raw = np.loadtxt(data/'lord_imu.log', delimiter=',', comments='#')
    raw_t = 315964800 + raw[:, 0]*604800 + raw[:, 1] + raw[:, 2] - 18
    keep = (raw_t >= prep['start_utc']-5) & (raw_t <= prep['end_utc'])
    assert np.allclose(stamps['/imu0'], raw_t[keep], atol=1e-6, rtol=0)
    for t, msg in samples['/imu0']:
        r = raw[np.argmin(abs(raw_t-t))]
        got = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
               msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        assert np.allclose(got, r[3:9]*[-1, 1, -1, -1, 1, -1], atol=1e-12, rtol=0)
    project = Path(__file__).resolve().parents[1]
    config = cv2.FileStorage(str(project/'config/texcup_stereo_config.yaml'), cv2.FILE_STORAGE_READ)
    transforms = [config.getNode('body_T_cam'+str(i)).mat() for i in range(2)]
    assert config.getNode('image_width').real() == 2048 and config.getNode('image_height').real() == 732
    assert np.allclose(transforms[0][:3, :3], [[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    assert np.allclose(transforms[0][:3, 3], [-.303, .011, .033])
    for matrix in transforms:
        assert np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-10)
    calibration = yaml.safe_load((data/'camera-calibration-stereo/texcup_stereo-camchain_ovins.yaml').read_text())
    assert np.allclose(transforms[1], transforms[0]@np.linalg.inv(np.array(calibration['cam1']['T_cn_cnm1'])), atol=1e-10)
    for i in range(2):
        camera = cv2.FileStorage(str(project/f'config/texcup_cam{i}.yaml'), cv2.FILE_STORAGE_READ)
        assert camera.getNode('image_width').real()==2048 and camera.getNode('image_height').real()==732
        intr = camera.getNode('projection_parameters')
        dist = camera.getNode('distortion_parameters')
        assert np.allclose([intr.getNode(k).real() for k in ['fx','fy','cx','cy']], calibration[f'cam{i}']['intrinsics'], atol=1e-10)
        assert np.allclose([dist.getNode(k).real() for k in ['k1','k2','p1','p2']], calibration[f'cam{i}']['distortion_coeffs'], atol=1e-10)
    report['stereo_baseline_m'] = float(np.linalg.norm(transforms[1][:3, 3]-transforms[0][:3, 3]))
    assert .497 < report['stereo_baseline_m'] < .499
    report['checks'] = dict(all_topic_record_times_monotonic=True, all_camera_times_match_hdf5=True,
                           all_imu_times_match_raw_utc=True, sampled_imu_axes_rfu=True,
                           first_middle_last_pixels_exact_both_cameras=True,
                           all_gps_stamps_match_camera_pairs=True, gps_covariance_uses_consumer_sigma_convention=True,
                           native_dimensions_and_camera_to_rfu_extrinsics=True)
    a.out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
