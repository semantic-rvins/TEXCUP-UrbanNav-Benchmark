#!/usr/bin/env python3
"""Validate TEX-CUP RRR timestamps, IMU axes and full-resolution image packing."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import struct

import h5py
import numpy as np


def file_info(path):
    return dict(path=str(path.resolve()), bytes=path.stat().st_size,
                mtime_ns=path.stat().st_mtime_ns)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', required=True)
    parser.add_argument('--imu', required=True)
    parser.add_argument('--images', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    data, imu_path, pack_path = Path(args.data_dir), Path(args.imu), Path(args.images)
    raw = np.loadtxt(data/'lord_imu.log', delimiter=',', comments='#')
    times = 315964800 + raw[:, 0]*604800 + raw[:, 1] + raw[:, 2] - 18
    keep = times >= 1557425380
    expected = raw[keep, 3:9] * np.array([-1, 1, -1, -1, 1, -1])
    imu = np.loadtxt(imu_path, skiprows=1)
    assert imu.shape == (int(keep.sum()), 7)
    assert np.isfinite(imu).all() and np.all(np.diff(imu[:, 0]) > 0)
    assert np.max(np.abs(imu[:, 0]-times[keep])) <= 0.000501
    np.testing.assert_allclose(imu[:, 1:], expected, atol=5.1e-9, rtol=0)
    report = dict(verified_utc=datetime.now(timezone.utc).isoformat(),
                  imu=dict(n_epochs=len(imu), first_utc_unix=float(imu[0, 0]),
                           last_utc_unix=float(imu[-1, 0]),
                           frame='RFU: [-raw_x, raw_y, -raw_z] for acceleration and gyro',
                           all_rows_verified=True))
    print(f'Validated {len(imu)} IMU samples, UTC timing and RFU axes', flush=True)
    frame_bytes = 2048*732 + 28
    with h5py.File(data/'camera_data/images.h5') as h5, pack_path.open('rb') as pack:
        frames = [(key, float(h5['port'][key].attrs['tstamp'])) for key in sorted(h5['port'])]
        frames = [(key, timestamp) for key, timestamp in frames if timestamp >= 1557425380]
        assert len(frames) == 40372
        assert pack_path.stat().st_size == len(frames)*frame_bytes
        assert np.all(np.diff([timestamp for _, timestamp in frames]) > 0)
        sample_indices = {0, len(frames)//2, len(frames)-1}
        for i, (key, timestamp) in enumerate(frames):
            pack.seek(i*frame_bytes)
            header = pack.read(22)
            assert header[:6] == bytes.fromhex('feca00ff00ff')
            assert int.from_bytes(header[6:9], 'big') == frame_bytes-9
            sec, nsec, width, height, step = struct.unpack('>IIHHB', header[9:])
            assert (width, height, step) == (2048, 732, 1)
            assert abs(sec+nsec*1e-9-timestamp) <= 2.5e-7
            if i in sample_indices:
                assert pack.read(2048*732) == np.ascontiguousarray(h5['port'][key][()], dtype=np.uint8).tobytes()
            pack.seek((i+1)*frame_bytes-6)
            assert pack.read(6) == bytes.fromhex('ccfeff00ff00')
        report['images'] = dict(n_frames=len(frames), width=2048, height=732, group='port',
                                first_utc_unix=frames[0][1], last_utc_unix=frames[-1][1],
                                all_timestamps_and_headers_verified=True,
                                pixel_verified_frame_indices=sorted(sample_indices))
    print('Validated all 40,372 image headers/timestamps and first/middle/last frame pixels', flush=True)
    report['files'] = {'imu': file_info(imu_path), 'images': file_info(pack_path),
                       'raw_imu': file_info(data/'lord_imu.log')}
    Path(args.out).write_text(json.dumps(report, indent=2)+'\n')
    print(f'Wrote {args.out}', flush=True)


if __name__ == '__main__':
    main()
