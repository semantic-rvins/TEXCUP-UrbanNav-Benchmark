#!/usr/bin/env python3
"""Convert TEX-CUP Lord IMU log to GICI imu-text format.

Input : lord_imu.log lines "week, wholesec, fracsec, accX, accY, accZ, gyroX, gyroY, gyroZ"
        (GPS time; acc m/s^2; gyro rad/s; raw Lord axes).
Output: GICI imu-text: header line + "timestamp acc(3) gyro(3)" whitespace-separated.
        timestamp = UTC unix seconds (GICI post-file readers put GNSS on the UTC
        timeline via gpst2utc, and use imu-text timestamps as-is -> must be UTC).

Axis convention: rotate the raw Lord axes by 180 deg about Y (x -> -x,
z -> -z) to obtain the calibrated RFU IMU frame (x-right, y-forward, z-up).
The lever arms and camera extrinsics in the root METHOD_SETUP_GUIDE.md and
the GICI configuration are expressed in this RFU frame.
"""
import argparse
import os
from pathlib import Path

GPS_UNIX_EPOCH = 315964800  # 1980-01-06 00:00:00 UTC
LEAP = 18                   # GPS-UTC leap seconds in 2019


def main():
    ap = argparse.ArgumentParser()
    data = Path(os.environ.get('TEXCUP_DATA', Path(__file__).resolve().parents[3]/'data/tex_cup'))
    ap.add_argument('--log', default=str(data/'lord_imu.log'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--cut-utc-unix', type=float, default=1557425380.0,
                    help='drop epochs before this UTC unix time (default 2019-05-09 18:09:40 UTC)')
    a = ap.parse_args()

    n_in = n_out = 0
    with open(a.log) as f, open(a.out, 'w') as g:
        g.write("Timestamp\tAcc-X\tAcc-Y\tAcc-Z\tGyro-X\tGyro-Y\tGyro-Z\t\r\n")
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            p = line.split(', ')
            if len(p) < 9:
                continue
            n_in += 1
            week = int(p[0]); tow = float(p[1]) + float(p[2])
            t_utc = GPS_UNIX_EPOCH + week * 604800 + tow - LEAP
            if t_utc < a.cut_utc_unix:
                continue
            ax, ay, az = -float(p[3]), float(p[4]), -float(p[5])   # z_up flip
            gx, gy, gz = -float(p[6]), float(p[7]), -float(p[8])
            g.write("%13.3f\t%12.8f\t%12.8f\t%12.8f\t%13.9f\t%13.9f\t%13.9f\r\n"
                    % (t_utc, ax, ay, az, gx, gy, gz))
            n_out += 1
    print(f"IMU: {n_in} epochs read, {n_out} written to {a.out}")


if __name__ == '__main__':
    main()
