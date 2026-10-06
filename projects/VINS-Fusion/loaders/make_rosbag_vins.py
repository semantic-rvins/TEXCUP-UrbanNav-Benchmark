#!/usr/bin/env python3
"""Build the TEX-CUP VINS-Fusion input rosbag.

Adapted from projects/GVINS/loaders/make_rosbag.py (IMU/image handling kept
identical); GNSS raw topics replaced by a sensor_msgs/NavSatFix topic derived
from the RTKLIB RTK solution.

Topics (all timestamps UTC unix seconds):
  /imu0            sensor_msgs/Imu    lord_imu.log (GPS week/tow -> UTC), axes
                   flipped x,z: raw Lord x-left,y-fwd,z-down -> body x-right,
                   y-fwd,z-up (RFU; see the root METHOD_SETUP_GUIDE.md)
  /cam0/image_raw  sensor_msgs/Image  mono8 LEFT ('port'), native 2048x732
  /cam1/image_raw  sensor_msgs/Image  mono8 RIGHT ('star'), native 2048x732,
                   stamped with the SAME time as its port pair (same h5 key;
                     raw skew < 3 ms) so VINS' 3 ms stereo sync always pairs.
  /gps             sensor_msgs/NavSatFix from RTKLIB est_180940.csv:
                   - antenna ECEF -> IMU/body position by removing the lever
                     arm t_imu_to_ant_in_b = [-0.610, -0.052, 0.01] m rotated
                     by yaw-only attitude (heading = RTKLIB course-over-ground;
                     hold-last/backfill when speed < 1.5 m/s), because
                     global_fusion fuses the VIO body position against the
                     NavSatFix with no lever-arm model;
                   - re-timed to the nearest estimator-admitted port timestamp
                     (every second pair in upstream multiple_thread mode; linear
                     interpolation of body ECEF inside contiguous RTKLIB
                     segments) so global_fusion's +-10 ms GPS<->VIO matching
                     always succeeds; RTKLIB outage epochs simply have no msg;
                   - position_covariance[0] (which global_fusion divides the
                     residual by, i.e. treats as a std in metres) set from the
                     RTKLIB Q flag: Q=1 (fix) -> 0.05 m, Q=2 (float) -> 0.75 m.

Usage (inside the RoboStack `ros` env):
  python make_rosbag_vins.py --out bags/texcup_vins.bag \
      [--start-utc 1557425380.0] [--end-utc 1557429420.0] [--downscale 1]

If --downscale is changed, update both camera dimensions and intrinsics to match;
the benchmark configs use native resolution.
"""
import argparse
import os
import csv
import math
import json
from pathlib import Path

import cv2
import h5py
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu, NavSatFix

BENCH = Path(__file__).resolve().parents[3]
DATA_DIR = Path(os.environ.get("TEXCUP_DATA", BENCH / "data/tex_cup"))
IMU_LOG = DATA_DIR / "lord_imu.log"
H5_PATH = DATA_DIR / "camera_data/images.h5"
RTKLIB_CSV = BENCH / "results/RTKLIB/est.csv"
GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06 00:00:00 UTC
LEAP = 18.0
MIDNIGHT_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC
T_IMU_TO_ANT_IN_B = np.array([-0.610, -0.052, 0.01])  # TexCupLordImuParams
SIGMA_FIX = 0.05    # m, RTKLIB Q=1
SIGMA_FLOAT = 0.75  # m, RTKLIB Q=2
SEG_GAP = 1.5       # s, break RTKLIB stream into contiguous segments
SPEED_MIN = 1.5     # m/s, min horizontal speed for a valid course heading

A_WGS = 6378137.0
E2_WGS = 6.69437999014e-3


def ecef2lla(p):
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    lon = np.arctan2(y, x)
    r = np.hypot(x, y)
    lat = np.arctan2(z, r * (1 - E2_WGS))
    for _ in range(6):
        N = A_WGS / np.sqrt(1 - E2_WGS * np.sin(lat) ** 2)
        h = r / np.cos(lat) - N
        lat = np.arctan2(z, r * (1 - E2_WGS * N / (N + h)))
    N = A_WGS / np.sqrt(1 - E2_WGS * np.sin(lat) ** 2)
    h = r / np.cos(lat) - N
    return np.stack([np.degrees(lat), np.degrees(lon), h], axis=-1)


def r_ecef_from_enu(lat_deg, lon_deg):
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    return np.array([
        [-math.sin(lo), -math.sin(la) * math.cos(lo), math.cos(la) * math.cos(lo)],
        [math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la) * math.sin(lo)],
        [0.0, math.cos(la), math.sin(la)],
    ])


def imu_stream(start_utc, end_utc):
    with open(IMU_LOG) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            p = line.split(", ")
            if len(p) < 9:
                continue
            t_gps = int(p[0]) * 604800.0 + float(p[1]) + float(p[2])
            t_utc = GPS_EPOCH_UNIX + t_gps - LEAP
            if t_utc < start_utc - 5.0:
                continue
            if t_utc > end_utc:
                break
            ax, ay, az = -float(p[3]), float(p[4]), -float(p[5])
            gx, gy, gz = -float(p[6]), float(p[7]), -float(p[8])
            msg = Imu()
            msg.header.stamp = rospy.Time.from_sec(t_utc)
            msg.header.frame_id = "imu"
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.orientation.w = 1.0
            yield t_utc, "/imu0", msg


def scan_camera_times(start_utc, end_utc):
    """Return (keys, port_ts) for frames in window; assert port/star pairing."""
    f = h5py.File(H5_PATH, "r")
    gp, gs = f["port"], f["star"]
    keys = sorted(gp.keys())
    assert sorted(gs.keys()) == keys, "port/star key sets differ"
    sel_keys, sel_ts = [], []
    max_skew = 0.0
    for k in keys:
        tp = float(gp[k].attrs["tstamp"])
        if tp < start_utc or tp > end_utc:
            continue
        ts = float(gs[k].attrs["tstamp"])
        max_skew = max(max_skew, abs(ts - tp))
        sel_keys.append(k)
        sel_ts.append(tp)
    f.close()
    print(f"camera frames in window: {len(sel_keys)}, max port/star skew "
          f"{max_skew*1e3:.2f} ms", flush=True)
    assert max_skew < 0.005, "stereo pairs not synchronous enough for VINS"
    return sel_keys, np.array(sel_ts)


def image_stream(keys, downscale):
    f = h5py.File(H5_PATH, "r")
    gp, gs = f["port"], f["star"]
    for k in keys:
        t_utc = float(gp[k].attrs["tstamp"])
        for grp, topic, frame in ((gp, "/cam0/image_raw", "cam0"),
                                  (gs, "/cam1/image_raw", "cam1")):
            img = grp[k][()]
            if downscale > 1:
                img = cv2.resize(
                    img,
                    (img.shape[1] // downscale, img.shape[0] // downscale),
                    interpolation=cv2.INTER_AREA,
                )
            msg = Image()
            msg.header.stamp = rospy.Time.from_sec(t_utc)  # star uses port time
            msg.header.frame_id = frame
            msg.height, msg.width = img.shape
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = img.shape[1]
            msg.data = img.tobytes()
            yield t_utc, topic, msg


def build_gps_msgs(cam_ts, start_utc, end_utc):
    """NavSatFix list: RTKLIB antenna ECEF -> body ECEF, re-timed to camera."""
    rows = list(csv.DictReader(open(RTKLIB_CSV)))
    t = np.array([MIDNIGHT_UNIX + float(r["utc_sec"]) for r in rows])
    xyz = np.array([[float(r["ecef_x"]), float(r["ecef_y"]), float(r["ecef_z"])]
                    for r in rows])
    q = np.array([int(r["q"]) for r in rows])
    keep = (t >= start_utc - 1.0) & (t <= end_utc)
    t, xyz, q = t[keep], xyz[keep], q[keep]
    sigma = np.where(q == 1, SIGMA_FIX, SIGMA_FLOAT)
    lla = ecef2lla(xyz)

    # contiguous segments
    seg_bounds = [0] + list(np.where(np.diff(t) > SEG_GAP)[0] + 1) + [len(t)]
    segs = [(seg_bounds[i], seg_bounds[i + 1])
            for i in range(len(seg_bounds) - 1)]

    # heading from course-over-ground (ENU velocity), per segment
    heading = np.full(len(t), np.nan)
    for a, b in segs:
        if b - a < 2:
            continue
        for i in range(a, b):
            j0, j1 = max(a, i - 1), min(b - 1, i + 1)
            v_ecef = (xyz[j1] - xyz[j0]) / (t[j1] - t[j0])
            v_enu = r_ecef_from_enu(lla[i, 0], lla[i, 1]).T @ v_ecef
            if np.hypot(v_enu[0], v_enu[1]) >= SPEED_MIN:
                heading[i] = math.atan2(v_enu[0], v_enu[1])  # from North, cw
    # hold-last / backfill headings (vehicle keeps heading while stopped)
    valid = np.where(~np.isnan(heading))[0]
    if len(valid) == 0:
        raise RuntimeError("no valid course headings in RTKLIB solution")
    for i in range(len(t)):
        if np.isnan(heading[i]):
            j = valid[np.argmin(np.abs(valid - i))]
            heading[i] = heading[j]
    n_course = int(len(t) - len(valid))

    # antenna -> IMU/body position (yaw-only attitude, roll=pitch=0)
    body = np.empty_like(xyz)
    for i in range(len(t)):
        psi = heading[i]
        # body axes in ENU: x=right, y=fwd, z=up
        r_enu_b = np.array([[math.cos(psi), math.sin(psi), 0.0],
                            [-math.sin(psi), math.cos(psi), 0.0],
                            [0.0, 0.0, 1.0]])
        off = r_ecef_from_enu(lla[i, 0], lla[i, 1]) @ (r_enu_b @ T_IMU_TO_ANT_IN_B)
        body[i] = xyz[i] - off

    # re-time each epoch to the nearest camera frame
    msgs, n_skip, n_single = [], 0, 0
    used_cam = set()
    for a, b in segs:
        for i in range(a, b):
            ci = int(np.argmin(np.abs(cam_ts - t[i])))
            tc = float(cam_ts[ci])
            dt = tc - t[i]
            if abs(dt) > 0.6 or ci in used_cam:
                n_skip += 1
                continue
            if b - a == 1:
                if abs(dt) > 0.02:  # cannot interpolate a lone epoch
                    n_single += 1
                    continue
                pos, sig = body[i], sigma[i]
            else:
                # linear interp/extrapolation from the two nearest seg epochs
                if tc <= t[a]:
                    j0, j1 = a, a + 1
                elif tc >= t[b - 1]:
                    j0, j1 = b - 2, b - 1
                else:
                    j1 = int(np.searchsorted(t[a:b], tc) + a)
                    j0 = j1 - 1
                w = (tc - t[j0]) / (t[j1] - t[j0])
                pos = (1 - w) * body[j0] + w * body[j1]
                sig = max(sigma[j0], sigma[j1])
            used_cam.add(ci)
            plla = ecef2lla(pos)
            m = NavSatFix()
            m.header.stamp = rospy.Time.from_sec(tc)
            m.header.frame_id = "gps"
            m.status.status = 0
            m.status.service = 1
            m.latitude = float(plla[0])
            m.longitude = float(plla[1])
            m.altitude = float(plla[2])
            # global_fusion reads position_covariance[0] and treats it as a
            # std (residual = err/cov[0]); we store sigma in all 3 diagonals.
            m.position_covariance = [sig, 0, 0, 0, sig, 0, 0, 0, sig]
            m.position_covariance_type = 2
            msgs.append((tc, "/gps", m))
    msgs.sort(key=lambda x: x[0])
    print(f"gps msgs: {len(msgs)} (from {len(t)} rtklib epochs; "
          f"{n_skip} out of camera range, {n_single} lone epochs dropped, "
          f"{n_course} headings held from neighbours)", flush=True)
    return msgs


def merge(streams):
    import heapq

    heads = []
    for i, s in enumerate(streams):
        try:
            item = next(s)
            heads.append((item[0], i, item[1], item[2], s))
        except StopIteration:
            pass
    heapq.heapify(heads)
    while heads:
        t, i, topic, msg, s = heapq.heappop(heads)
        yield t, topic, msg
        try:
            nt, ntopic, nmsg = next(s)
            heapq.heappush(heads, (nt, i, ntopic, nmsg, s))
        except StopIteration:
            pass


def main():
    global IMU_LOG, H5_PATH, RTKLIB_CSV
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--data-dir", type=Path, default=DATA_DIR)
    ap.add_argument("--rtklib", type=Path, default=RTKLIB_CSV)
    ap.add_argument("--start-utc", type=float, default=1557425380.0)
    ap.add_argument("--end-utc", type=float, default=1557429420.0)
    ap.add_argument("--downscale", type=int, default=1,
                    help="image scale divisor; camera dimensions/intrinsics must match (default: native)")
    ap.add_argument("--compression", default="lz4", choices=["none", "lz4", "bz2"])
    args = ap.parse_args()
    if args.downscale < 1 or args.end_utc <= args.start_utc:
        ap.error("require downscale >= 1 and end > start")
    if Path(args.out).exists():
        ap.error("output bag already exists; choose a new path")
    IMU_LOG = args.data_dir.resolve() / "lord_imu.log"
    H5_PATH = args.data_dir.resolve() / "camera_data/images.h5"
    RTKLIB_CSV = args.rtklib.resolve()

    comp = {
        "none": rosbag.Compression.NONE,
        "lz4": rosbag.Compression.LZ4,
        "bz2": rosbag.Compression.BZ2,
    }[args.compression]

    keys, cam_ts = scan_camera_times(args.start_utc, args.end_utc)
    # Upstream tracks every image but admits inputImageCnt % 2 == 0 to VIO.
    gps_msgs = build_gps_msgs(cam_ts[1::2], args.start_utc, args.end_utc)

    out = rosbag.Bag(args.out, "w", compression=comp)
    n = {"/imu0": 0, "/cam0/image_raw": 0, "/cam1/image_raw": 0, "/gps": 0}
    try:
        for t, topic, msg in merge(
            [
                imu_stream(args.start_utc, args.end_utc),
                image_stream(keys, args.downscale),
                iter(gps_msgs),
            ]
        ):
            out.write(topic, msg, rospy.Time.from_sec(t))
            n[topic] += 1
            tot = sum(n.values())
            if tot % 50000 == 0:
                print(f"written {tot} msgs {n}", flush=True)
    finally:
        out.close()
    print("done:", n)
    report = dict(bag=str(Path(args.out).resolve()), counts=n,
                  start_utc=args.start_utc, end_utc=args.end_utc,
                  downscale=args.downscale, stereo_pairs=len(keys),
                  first_image_utc=float(cam_ts[0]), last_image_utc=float(cam_ts[-1]),
                  rtklib=str(RTKLIB_CSV), data_dir=str(args.data_dir.resolve()),
                  bag_bytes=Path(args.out).stat().st_size,
                  bag_mtime_ns=Path(args.out).stat().st_mtime_ns)
    Path(args.out + '.preparation.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == "__main__":
    main()
