#!/usr/bin/env python3
"""Extract InGVIO global estimates -> est.csv for common/evaluate.py.

Reads results/ingvio_out.bag:
  /ingvio_estimator/pose_ecef  (nav_msgs/Odometry, added by
      patches/02-publish-ecef-odometry.patch; stamp = UTC unix sec,
      pose.position = IMU position in ECEF,
      pose.orientation = R_imu->ecef)

Antenna position = IMU position + R_imu->ecef @ t_imu_to_ant_in_b with
t_imu_to_ant_in_b = [-0.610, -0.052, 0.01] m (TexCupLordImuParams; body frame
x-right, y-fwd, z-up, same convention as /imu0 in the bag).

Writes est.csv: utc_sec (sec of UTC day 2019-05-09), ecef_x/y/z (antenna),
qw/qx/qy/qz (body -> ECEF).
"""
import argparse
import csv
import math

import numpy as np
import rosbag

DAY0_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC
T_IMU_ANT = np.array([-0.610, -0.052, 0.01])


def quat2R(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="/ingvio_estimator/pose_ecef")
    args = ap.parse_args()

    rows = []
    with rosbag.Bag(args.bag) as b:
        for _topic, msg, _t in b.read_messages(topics=[args.topic]):
            t_utc = msg.header.stamp.to_sec()
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            R = quat2R(o.w, o.x, o.y, o.z)
            p_ant = np.array([p.x, p.y, p.z]) + R @ T_IMU_ANT
            rows.append(
                [t_utc - DAY0_UNIX, p_ant[0], p_ant[1], p_ant[2], o.w, o.x, o.y, o.z]
            )

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["utc_sec", "ecef_x", "ecef_y", "ecef_z", "qw", "qx", "qy", "qz"])
        w.writerows(rows)
    print(f"wrote {len(rows)} rows to {args.out}")
    if rows:
        print("first:", rows[0][0], "last:", rows[-1][0])


if __name__ == "__main__":
    main()
