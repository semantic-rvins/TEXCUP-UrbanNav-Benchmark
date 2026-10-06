#!/usr/bin/env python3
"""Validate TEX-CUP GVINS bag timing, GNSS systems and sampled sensor payloads."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import h5py
import numpy as np
import rosbag
import rospy


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--gnss-bag', type=Path, required=True)
    ap.add_argument('--data-dir', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    start = 1557425380.0
    report = {'bag': str(args.bag.resolve()), 'bag_bytes': args.bag.stat().st_size,
              'topics': {}, 'checks': {}}
    samples = {}
    with rosbag.Bag(str(args.bag)) as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, metadata in info.items():
            indexes = list(bag._get_indexes(bag._get_connections([topic])))
            stamps = np.array(sorted(entry.time.to_sec() for index in indexes for entry in index))
            assert len(stamps) == metadata.message_count and np.isfinite(stamps).all()
            assert np.all(np.diff(stamps) > 0), topic
            report['topics'][topic] = dict(count=len(stamps), first_utc=float(stamps[0]),
                                          last_utc=float(stamps[-1]))
            if topic == '/cam0/image_raw':
                image_record_times = stamps
            if topic in ('/imu0', '/cam0/image_raw'):
                samples[topic] = []
                for t in stamps[[0, len(stamps)//2, -1]]:
                    _, msg, record = next(bag.read_messages(topics=[topic],
                        start_time=rospy.Time.from_sec(float(t)-1e-6)))
                    assert abs(record.to_sec()-t) < 1e-6
                    assert abs(msg.header.stamp.to_sec()-t) < 1e-6
                    samples[topic].append((t, msg))
        assert report['topics']['/ublox_driver/range_meas']['count'] == 4557
        assert report['topics']['/ublox_driver/range_meas']['first_utc'] == start
        assert not any('glo' in topic for topic in info)

    # Satellite-number ranges follow this pinned gnss_comm's satsys implementation.
    systems = Counter()
    frequencies = defaultdict(set)
    n_epochs = 0
    with rosbag.Bag(str(args.gnss_bag)) as bag:
        for _, msg, record in bag.read_messages(topics=['/ublox_driver/range_meas']):
            n_epochs += 1
            assert msg.meas
            for obs in msg.meas:
                gpst = 315964800 + obs.time.week*604800 + obs.time.tow
                assert abs(gpst-record.to_sec()-18) < 1e-6
                if 1 <= obs.sat <= 32:
                    system, allowed = 'GPS', {1575420000.0, 1227600000.0}
                elif 60 <= obs.sat <= 97:
                    system, allowed = 'Galileo', {1575420000.0, 1207140000.0}
                elif 98 <= obs.sat <= 160:
                    system, allowed = 'BeiDou', {1561098000.0}
                else:
                    raise AssertionError(f'Unexpected satellite system: {obs.sat}')
                assert set(obs.freqs) <= allowed
                for field in ('freqs', 'psr', 'cp', 'dopp', 'CN0', 'psr_std', 'dopp_std'):
                    values = getattr(obs, field)
                    assert len(values) == len(obs.freqs) and np.isfinite(values).all(), field
                assert np.allclose(obs.psr_std, 0.16) and np.allclose(obs.dopp_std, 0.256)
                systems[system] += 1
                frequencies[system].update(obs.freqs)
    assert n_epochs == 4557 and set(systems) == {'GPS', 'Galileo', 'BeiDou'}
    report['gnss_observations_by_system'] = dict(systems)
    report['gnss_frequencies_hz'] = {k: sorted(v) for k, v in frequencies.items()}
    report['checks']['gnss_gpst_payload_minus_utc_record_equals_18_seconds'] = True

    with h5py.File(args.data_dir / 'camera_data/images.h5', 'r') as source:
        group = source['port']
        frames = [(key, float(group[key].attrs['tstamp'])) for key in sorted(group)]
        frames = [(key, t) for key, t in frames if t >= start]
        assert len(frames) == report['topics']['/cam0/image_raw']['count']
        times = np.array([t for _, t in frames])
        assert np.all(np.diff(times) > 0)
        assert np.allclose(times, image_record_times, rtol=0, atol=1e-6)
        for t, msg in samples['/cam0/image_raw']:
            i = int(np.argmin(abs(times-t)))
            assert abs(times[i]-t) < 1e-6
            assert (msg.width, msg.height, msg.step, msg.encoding) == (2048, 732, 2048, 'mono8')
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(732, 2048)
            assert np.array_equal(image, group[frames[i][0]][()])
    report['checks']['first_middle_last_images_match_native_port_hdf5'] = True
    report['checks']['all_image_timestamps_covered_in_order'] = True

    targets = samples['/imu0']
    matched = 0
    with (args.data_dir / 'lord_imu.log').open() as source:
        for line in source:
            if line.startswith('#') or not line.strip():
                continue
            p = line.split(', ')
            if len(p) < 9:
                continue
            t = 315964800 + int(p[0])*604800 + float(p[1]) + float(p[2]) - 18
            for sample_t, msg in targets:
                if abs(sample_t-t) < 1e-6:
                    actual = [msg.linear_acceleration.x, msg.linear_acceleration.y,
                              msg.linear_acceleration.z, msg.angular_velocity.x,
                              msg.angular_velocity.y, msg.angular_velocity.z]
                    expected = [-float(p[3]), float(p[4]), -float(p[5]),
                                -float(p[6]), float(p[7]), -float(p[8])]
                    assert np.isfinite(actual).all() and np.allclose(actual, expected, rtol=0, atol=1e-12)
                    matched += 1
    assert matched == 3
    report['checks']['first_middle_last_imu_match_raw_to_rfu_and_utc'] = True
    report['input_files'] = {}
    for path in [args.data_dir / 'asterx4_rover.obs', args.data_dir / 'lord_imu.log',
                 args.data_dir / 'brdm1290_v304.19p',
                 args.data_dir / 'camera_data/images.h5']:
        item = {'path': str(path.resolve()), 'bytes': path.stat().st_size,
                'mtime_ns': path.stat().st_mtime_ns}
        report['input_files'][path.name] = item
    report['input_validation'] = 'Topic timing, observation fields and sampled sensor payloads checked against local inputs.'
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
