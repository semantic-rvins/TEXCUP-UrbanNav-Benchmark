#!/usr/bin/env python3
"""Convert an OKVIS2-X global trajectory csv to the common est.csv format.

Input : okvis2-[mode]-global-final[(-ba)]_trajectory.csv with rows
        "timestamp(ns), p_GA_G_x, p_GA_G_y, p_GA_G_z"  -- ANTENNA position in the
        GNSS frame G (lever arm r_SA already applied by OKVIS2-X, see
        ViSlamBackend::writeGlobalCsvTrajectory). G = ENU anchored at the first
        ground-truth point (that is what loaders/convert_texcup.py fed as
        'cartesian' GPS measurements).
Output: est.csv with header utc_sec,ecef_x,ecef_y,ecef_z  (utc_sec = seconds of
        day 2019-05-09 UTC, positions = antenna ECEF), for common/evaluate.py.
"""
import argparse
import json
import math

import numpy as np

DAY0_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC
A = 6378137.0
E2 = 6.69437999014e-3


def lla2ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    N = A / math.sqrt(1 - E2 * math.sin(la) ** 2)
    return np.array([(N + h) * math.cos(la) * math.cos(lo),
                     (N + h) * math.cos(la) * math.sin(lo),
                     (N * (1 - E2) + h) * math.sin(la)])


def enu_rotation(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    return np.array([
        [-math.sin(lo), math.cos(lo), 0],
        [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
        [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", required=True)
    ap.add_argument("--anchor", required=True, help="gps0/anchor.json from the converter")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    ref = json.load(open(args.anchor))["ref_lla"]
    R = enu_rotation(ref[0], ref[1])  # ECEF -> ENU; transpose is ENU -> ECEF
    ref_ecef = lla2ecef(*ref)

    n = 0
    with open(args.traj) as f, open(args.out, "w") as o:
        o.write("utc_sec,ecef_x,ecef_y,ecef_z\n")
        header = f.readline()
        assert "p_GA_G" in header, f"unexpected header: {header}"
        for line in f:
            p = [s.strip() for s in line.split(",")]
            if len(p) < 4:
                continue
            t_ns = int(p[0])
            enu = np.array([float(p[1]), float(p[2]), float(p[3])])
            ecef = R.T @ enu + ref_ecef
            utc_sec = t_ns * 1e-9 - DAY0_UNIX
            o.write(f"{utc_sec:.3f},{ecef[0]:.4f},{ecef[1]:.4f},{ecef[2]:.4f}\n")
            n += 1
    print("est rows:", n)


if __name__ == "__main__":
    main()
