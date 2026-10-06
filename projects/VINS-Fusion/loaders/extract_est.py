#!/usr/bin/env python3
"""Convert VINS-Fusion output to the benchmark est.csv (antenna ECEF).

Modes:
  global (default): read /globalEstimator/global_odometry from the output bag.
    Frame: global_fusion optimizes in a GeographicLib LocalCartesian ENU frame
    anchored at the LLA of the FIRST fused NavSatFix (globalOpt.cpp GPS2XYZ
    Reset on first call). The anchor is printed by our patch as
    "GLOBAL_FUSION_ANCHOR_LLA lat lon alt" in the global_fusion log.
    Positions are the IMU/body; orientation q = body->ENU. Antenna position =
    p + R(q) @ t_imu_to_ant_in_b, then ENU->ECEF exactly via the anchor.

  vio: read results/vins_output/vio.csv (VIO-only, arbitrary gravity-aligned
    world frame). Aligned to the RTKLIB solution with a 4-DOF (yaw + 3D
    translation) least-squares fit over the first --align-window seconds of
    epochs both have; this represents "VIO with initial pose from GNSS".
    Lever arm applied with VIO attitude (rotated by the alignment yaw).

Usage:
  python extract_est.py --bag results/vinsfusion_out.bag \
      --anchor-log results/logs/global_fusion.log --out results/est.csv
  python extract_est.py --mode vio --vio-csv results/vins_output/vio.csv \
      --out results/est_vio.csv
"""
import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np

MIDNIGHT_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC
T_IMU_TO_ANT_IN_B = np.array([-0.610, -0.052, 0.01])  # TexCupLordImuParams
RTKLIB_CSV = Path(__file__).resolve().parents[3] / 'results/RTKLIB/est.csv'
A_WGS = 6378137.0
E2_WGS = 6.69437999014e-3


def lla2ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    N = A_WGS / math.sqrt(1 - E2_WGS * math.sin(la) ** 2)
    return np.array([(N + h) * math.cos(la) * math.cos(lo),
                     (N + h) * math.cos(la) * math.sin(lo),
                     (N * (1 - E2_WGS) + h) * math.sin(la)])


def r_ecef_from_enu(lat_deg, lon_deg):
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    return np.array([
        [-math.sin(lo), -math.sin(la) * math.cos(lo), math.cos(la) * math.cos(lo)],
        [math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la) * math.sin(lo)],
        [0.0, math.cos(la), math.sin(la)],
    ])


def quat_to_rot(qw, qx, qy, qz):
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def read_anchor(log_path):
    pat = re.compile(r"GLOBAL_FUSION_ANCHOR_LLA\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)")
    with open(log_path) as f:
        for line in f:
            m = pat.search(line)
            if m:
                return float(m.group(1)), float(m.group(2)), float(m.group(3))
    raise RuntimeError(f"no GLOBAL_FUSION_ANCHOR_LLA in {log_path}")


def write_est(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["utc_sec", "ecef_x", "ecef_y", "ecef_z",
                    "qw", "qx", "qy", "qz"])
        for r in rows:
            w.writerow([f"{r[0]:.3f}"] + [f"{v:.4f}" for v in r[1:4]]
                       + [f"{v:.7f}" for v in r[4:]])
    print(f"wrote {len(rows)} rows -> {path}")


def mode_global(args):
    lat0, lon0, alt0 = read_anchor(args.anchor_log)
    print(f"anchor LLA: {lat0:.9f} {lon0:.9f} {alt0:.4f}")
    ecef0 = lla2ecef(lat0, lon0, alt0)
    R0 = r_ecef_from_enu(lat0, lon0)
    poses = []
    if args.global_csv:  # patched globalOptNode csv (t_ns,x,y,z,qw,qx,qy,qz)
        raw = np.loadtxt(args.global_csv, delimiter=",", usecols=range(8))
        for r in raw:
            poses.append((r[0] * 1e-9, r[1:4], r[4], r[5], r[6], r[7]))
    else:
        import rosbag
        bag = rosbag.Bag(args.bag)
        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            q = msg.pose.pose.orientation
            poses.append((msg.header.stamp.to_sec(),
                          np.array([msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z]),
                          q.w, q.x, q.y, q.z))
        bag.close()
    rows = []
    for t, p, qw, qx, qy, qz in poses:
        if args.after_utc is not None and t <= args.after_utc:
            continue  # global alignment has not yet been applied to this output
        norm = math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)
        if not math.isfinite(norm) or abs(norm-1)>1e-4:
            raise ValueError(f'Invalid global rotation at UTC {t}: quaternion norm {norm}. '
                             'Rerun global fusion; output normalization cannot repair affected positions.')
        R_wb = quat_to_rot(qw, qx, qy, qz)
        p_ant = p + R_wb @ T_IMU_TO_ANT_IN_B
        ecef = ecef0 + R0 @ p_ant
        rows.append((t - MIDNIGHT_UNIX, ecef[0], ecef[1], ecef[2],
                     qw, qx, qy, qz))
    write_est(args.out, rows)


def mode_vio(args):
    # VIO trajectory (body in VIO world), antenna via lever arm
    raw = np.loadtxt(args.vio_csv, delimiter=",", usecols=range(11))
    t = raw[:, 0] * 1e-9
    p = raw[:, 1:4]
    q = raw[:, 4:8]  # w x y z
    p_ant = np.array([p[i] + quat_to_rot(*q[i]) @ T_IMU_TO_ANT_IN_B
                      for i in range(len(p))])

    # RTKLIB antenna in ENU anchored at its first epoch
    rows = list(csv.DictReader(open(args.rtklib)))
    tg = np.array([MIDNIGHT_UNIX + float(r["utc_sec"]) for r in rows])
    xyz = np.array([[float(r["ecef_x"]), float(r["ecef_y"]), float(r["ecef_z"])]
                    for r in rows])
    # anchor = first epoch LLA
    x, y, z = xyz[0]
    lon0 = math.degrees(math.atan2(y, x))
    rr = math.hypot(x, y)
    la = math.atan2(z, rr * (1 - E2_WGS))
    for _ in range(6):
        N = A_WGS / math.sqrt(1 - E2_WGS * math.sin(la) ** 2)
        h = rr / math.cos(la) - N
        la = math.atan2(z, rr * (1 - E2_WGS * N / (N + h)))
    lat0 = math.degrees(la)
    ecef0 = xyz[0].copy()
    R0 = r_ecef_from_enu(lat0, lon0)
    g_enu = (xyz - ecef0) @ R0  # == R0.T @ (xyz-ecef0) row-wise

    # matched pairs in the alignment window (interp VIO to RTKLIB times)
    t0 = max(t[0], tg[0])
    sel = (tg >= t0) & (tg <= t0 + args.align_window) & (tg <= t[-1])
    ta = tg[sel]
    va = np.stack([np.interp(ta, t, p_ant[:, k]) for k in range(3)], axis=1)
    ga = g_enu[sel]
    # 4-DOF fit: min || Rz(yaw) va + d - ga ||^2  (yaw from horizontal Procrustes)
    vc, gc = va - va.mean(0), ga - ga.mean(0)
    num = np.sum(vc[:, 0] * gc[:, 1] - vc[:, 1] * gc[:, 0])
    den = np.sum(vc[:, 0] * gc[:, 0] + vc[:, 1] * gc[:, 1])
    yaw = math.atan2(num, den)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    d = ga.mean(0) - (Rz @ va.T).T.mean(0)
    res = (Rz @ va.T).T + d - ga
    print(f"vio alignment: {len(ta)} pairs, yaw {math.degrees(yaw):.2f} deg, "
          f"rms {np.sqrt((res**2).sum(1).mean()):.2f} m")

    ecef = ecef0[None, :] + ((Rz @ p_ant.T).T + d) @ R0.T
    rows_out = []
    for i in range(len(t)):
        rows_out.append((t[i] - MIDNIGHT_UNIX, *ecef[i], *q[i]))
    write_est(args.out, rows_out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="global", choices=["global", "vio"])
    ap.add_argument("--bag")
    ap.add_argument("--global-csv", help="use the global_fusion csv instead of --bag")
    ap.add_argument("--topic", default="/globalEstimator/global_odometry")
    ap.add_argument("--anchor-log")
    ap.add_argument("--vio-csv")
    ap.add_argument("--align-window", type=float, default=120.0)
    ap.add_argument("--rtklib", type=Path, default=RTKLIB_CSV)
    ap.add_argument("--after-utc", type=float,
                    help="exclude output through the first completed global optimization timestamp")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    if args.mode == "global":
        mode_global(args)
    else:
        mode_vio(args)


if __name__ == "__main__":
    main()
