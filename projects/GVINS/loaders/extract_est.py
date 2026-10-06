#!/usr/bin/env python3
"""Extract GVINS global estimates -> est.csv for common/evaluate.py.

Reads results/gvins_out.bag:
  /gvins/gnss_fused_lla  (NavSatFix, stamp = GPST unix sec, IMU position LLA)
  /gvins/enu_pose        (PoseStamped, stamp = local UTC unix sec,
                          orientation q_enu_sensor with sensor = camera-front:
                          R_w_sensor = Rs * ric * R_s_c^T, published in ENU)

Antenna position = IMU position + R_ecef_enu * R_enu_body * t_imu_to_ant_in_b,
with R_enu_body = R_enu_sensor * R_s_c * ric^T and
t_imu_to_ant_in_b = [-0.610, -0.052, 0.01] (TexCupLordImuParams).

Writes est.csv: utc_sec (sec of UTC day), ecef_x/y/z, qw/qx/qy/qz (body in ENU).
"""
import argparse
import csv
import math

import numpy as np
import rosbag

LEAP = 18.0
DAY0_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC
T_IMU_ANT = np.array([-0.610, -0.052, 0.01])
A = 6378137.0
E2 = 6.69437999014e-3

# camera-front convention used in visualization.cpp
R_S_C = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# Initial imu^R_cam from config/gvins_texcup.yaml, matching the active
# TexCupLordImuParams.rot_imu_from_opencv. This remains an approximation
# when GVINS refines its extrinsics; historical bags need their run's rotation.
RIC = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])


def lla2ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    N = A / math.sqrt(1 - E2 * math.sin(la) ** 2)
    return np.array(
        [
            (N + h) * math.cos(la) * math.cos(lo),
            (N + h) * math.cos(la) * math.sin(lo),
            (N * (1 - E2) + h) * math.sin(la),
        ]
    )


def R_ecef_enu(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    return np.array(
        [
            [-math.sin(lo), -math.sin(la) * math.cos(lo), math.cos(la) * math.cos(lo)],
            [math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la) * math.sin(lo)],
            [0.0, math.cos(la), math.sin(la)],
        ]
    )


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


def R2quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return np.array(
            [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        )
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    lla = {}   # utc_unix (rounded to ms) -> (lat, lon, h)
    pose = {}  # utc_unix -> quaternion
    with rosbag.Bag(args.bag) as b:
        for topic, msg, _t in b.read_messages(
            topics=["/gvins/gnss_fused_lla", "/gvins/enu_pose"]
        ):
            if topic == "/gvins/gnss_fused_lla":
                t_utc = msg.header.stamp.to_sec() - LEAP
                lla[round(t_utc * 1000)] = (msg.latitude, msg.longitude, msg.altitude)
            else:
                t_utc = msg.header.stamp.to_sec()
                o = msg.pose.orientation
                pose[round(t_utc * 1000)] = (o.w, o.x, o.y, o.z)

    keys = sorted(lla.keys())
    pose_keys = np.array(sorted(pose.keys()))
    rows = []
    for k in keys:
        lat, lon, h = lla[k]
        p_imu = lla2ecef(lat, lon, h)
        q = None
        if len(pose_keys):
            i = int(np.argmin(np.abs(pose_keys - k)))
            if abs(pose_keys[i] - k) <= 50:  # ms
                q = pose[int(pose_keys[i])]
        if q is not None:
            R_enu_sensor = quat2R(*q)
            R_enu_body = R_enu_sensor @ R_S_C @ RIC.T
        else:
            R_enu_body = np.eye(3)
        p_ant = p_imu + R_ecef_enu(lat, lon) @ (R_enu_body @ T_IMU_ANT)
        qout = R2quat(R_enu_body)
        rows.append(
            [
                k / 1000.0 - DAY0_UNIX,
                p_ant[0],
                p_ant[1],
                p_ant[2],
                qout[0],
                qout[1],
                qout[2],
                qout[3],
            ]
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
