#!/usr/bin/env python3
"""Convert IC-GVINS gvins.nav -> est.csv (ANTENNA position) for common/evaluate.py.

gvins.nav columns (MISC::writeNavResult, ~10 Hz):
  0 (week placeholder), sow (GPS seconds of week), lat_deg, lon_deg, h_ell,
  vn, ve, vd (m/s, NED), roll, pitch, yaw (deg).
Position is the IMU centre; attitude is R_n_b with n = local NED, b = IMU
front-right-down, ZYX euler (yaw around Z first when composing R = Rz Ry Rx).

Antenna position = p_imu_ecef + R_ecef_ned(lat,lon) @ R_nb @ antlever_frd,
antlever_frd = [-0.052, -0.610, -0.01] m (TexCupLordImuParams
t_imu_to_ant_in_b = [-0.610, -0.052, 0.01] RFU rotated to FRD).

Writes est.csv: utc_sec (seconds of UTC day 2019-05-09), ecef_x/y/z (antenna),
qw/qx/qy/qz (body FRD w.r.t. local NED).
"""
import argparse
import csv
import math

import numpy as np

LEAP = 18.0
DOW_THURSDAY = 4 * 86400  # 2019-05-09 was GPS day-of-week 4 (week 2052)
ANTLEVER_FRD = np.array([-0.052, -0.610, -0.01])
A = 6378137.0
E2 = 6.69437999014e-3


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


def R_ecef_ned(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    sl, cl = math.sin(la), math.cos(la)
    so, co = math.sin(lo), math.cos(lo)
    # columns: N, E, D expressed in ECEF
    return np.array(
        [
            [-sl * co, -so, -cl * co],
            [-sl * so, co, -cl * so],
            [cl, 0.0, -sl],
        ]
    )


def euler2R(roll, pitch, yaw):
    """ZYX: R_n_b = Rz(yaw) @ Ry(pitch) @ Rx(roll), angles in radians."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


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
    ap.add_argument("--nav", required=True, help="results/gvins_output/gvins.nav")
    ap.add_argument("--out", required=True)
    ap.add_argument(
        "--end-utc-sod",
        type=float,
        default=None,
        help="optional inclusive UTC seconds-of-day cutoff; default preserves all "
        "emitted navigation. common/evaluate.py applies the shared 19:16:59 window end.",
    )
    args = ap.parse_args()

    rows = []
    for line in open(args.nav):
        p = line.split()
        if len(p) < 11:
            continue
        sow = float(p[1])
        if args.end_utc_sod is not None and sow - DOW_THURSDAY - LEAP > args.end_utc_sod:
            break
        lat, lon, h = float(p[2]), float(p[3]), float(p[4])
        roll, pitch, yaw = (math.radians(float(v)) for v in p[8:11])
        utc_sod = sow - DOW_THURSDAY - LEAP  # GPST sow -> UTC seconds of day
        R_nb = euler2R(roll, pitch, yaw)
        p_imu = lla2ecef(lat, lon, h)
        p_ant = p_imu + R_ecef_ned(lat, lon) @ (R_nb @ ANTLEVER_FRD)
        q = R2quat(R_nb)
        rows.append([f"{utc_sod:.3f}"]
                    + [f"{v:.4f}" for v in p_ant]
                    + [f"{v:.7f}" for v in q])

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["utc_sec", "ecef_x", "ecef_y", "ecef_z", "qw", "qx", "qy", "qz"])
        w.writerows(rows)
    print(f"wrote {len(rows)} rows to {args.out}")
    if rows:
        print("first:", rows[0][0], "last:", rows[-1][0])


if __name__ == "__main__":
    main()
