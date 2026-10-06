#!/usr/bin/env python3
"""Build the TEX-CUP IC-GVINS input rosbag (extends GVINS/loaders/make_rosbag.py).

Merges, in time order (all timestamps UTC unix seconds):
  - /imu0            sensor_msgs/Imu    from lord_imu.log (GPS week/tow -> UTC).
                     IC-GVINS requires FRONT-RIGHT-DOWN axes; raw Lord log is
                     x-left, y-fwd, z-down, so (f,r,d) = (y_raw, -x_raw, z_raw).
                     Rates (rad/s, m/s^2), NOT increments: fusion_ros.cc
                     multiplies by dt itself.
  - /cam0/image_raw  sensor_msgs/Image  mono8, LEFT ('port') from images.h5,
                     at native 2048x732 resolution as for GVINS.
  - /gnss0           sensor_msgs/NavSatFix from the RTKLIB RTK solution
                     (results/RTKLIB/rtklib_demo5.pos, GPST tags):
                     ECEF -> lat/lon/ellipsoidal h, stamp GPST-18s (UTC).
                     position_covariance diag = [stdE^2, stdN^2, stdU^2]
                     (fusion_ros.cc reads cov[4]->N, cov[0]->E, cov[8]->D),
                     std from the Q flag: Q=1 (fix)   -> 0.05 m hor / 0.10 m ver
                                          Q=2 (float) -> 0.50 m hor / 1.00 m ver
                     Outage epochs are simply absent from the .pos file (~32%).

Run inside the RoboStack ros env (needs rosbag/rospy/h5py/cv2).

Usage:
  python make_icgvins_bag.py --out ../bags/texcup_icgvins.bag \
      [--start-utc 1557425380.0] [--downscale 1]

If --downscale is changed, update the camera dimensions and intrinsics to match;
the benchmark config uses native resolution.
"""
import argparse
import os
import heapq
import math
import json
from pathlib import Path

import cv2
import h5py
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu, NavSatFix

BENCH = Path(__file__).resolve().parents[3]
DATA_DIR = Path(os.environ.get('TEXCUP_DATA', BENCH / 'data/tex_cup'))
IMU_LOG = DATA_DIR / 'lord_imu.log'
H5_PATH = DATA_DIR / 'camera_data/images.h5'
POS_FILE = BENCH / 'results/RTKLIB/rtklib_demo5.pos'
GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06 00:00:00 UTC
LEAP = 18.0
DAY0_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC

A = 6378137.0
E2 = 6.69437999014e-3
# (horizontal, vertical) std metres by RTKLIB Q flag; overridable on the CLI
STD_BY_Q = {1: (0.05, 0.10), 2: (0.50, 1.00)}


def ecef2lla(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - E2))
    for _ in range(6):
        N = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - N
        lat = math.atan2(z, p * (1 - E2 * N / (N + h)))
    N = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), h


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
            ax_r, ay_r, az_r = float(p[3]), float(p[4]), float(p[5])
            gx_r, gy_r, gz_r = float(p[6]), float(p[7]), float(p[8])
            msg = Imu()
            msg.header.stamp = rospy.Time.from_sec(t_utc)
            msg.header.frame_id = "imu"
            # raw (x-left, y-fwd, z-down) -> FRD
            msg.linear_acceleration.x = ay_r
            msg.linear_acceleration.y = -ax_r
            msg.linear_acceleration.z = az_r
            msg.angular_velocity.x = gy_r
            msg.angular_velocity.y = -gx_r
            msg.angular_velocity.z = gz_r
            msg.orientation.w = 1.0
            yield t_utc, "/imu0", msg


def image_stream(start_utc, end_utc, downscale):
    f = h5py.File(H5_PATH, "r")
    g = f["port"]
    for k in sorted(g.keys()):
        d = g[k]
        t_utc = float(d.attrs["tstamp"])
        if t_utc < start_utc:
            continue
        if t_utc > end_utc:
            break
        img = d[()]
        if downscale > 1:
            img = cv2.resize(
                img,
                (img.shape[1] // downscale, img.shape[0] // downscale),
                interpolation=cv2.INTER_AREA,
            )
        msg = Image()
        msg.header.stamp = rospy.Time.from_sec(t_utc)
        msg.header.frame_id = "cam0"
        msg.height, msg.width = img.shape
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = img.shape[1]
        msg.data = img.tobytes()
        yield t_utc, "/cam0/image_raw", msg


def gnss_stream(start_utc, end_utc):
    import datetime

    for line in open(POS_FILE):
        if line.startswith("%") or not line.strip():
            continue
        p = line.split()
        if len(p) < 8:
            continue
        dt = datetime.datetime.strptime(
            p[0] + " " + p[1], "%Y/%m/%d %H:%M:%S.%f"
        ).replace(tzinfo=datetime.timezone.utc)
        t_utc = dt.timestamp() - LEAP  # tags are GPST
        if t_utc < start_utc or t_utc > end_utc:
            continue
        x, y, z, q = float(p[2]), float(p[3]), float(p[4]), int(p[5])
        if q not in STD_BY_Q:
            continue
        hstd, vstd = STD_BY_Q[q]
        lat, lon, h = ecef2lla(x, y, z)
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.from_sec(t_utc)
        msg.header.frame_id = "gnss"
        msg.status.status = 0
        msg.status.service = 1
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = h
        msg.position_covariance = [
            hstd * hstd, 0.0, 0.0,
            0.0, hstd * hstd, 0.0,
            0.0, 0.0, vstd * vstd,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        yield t_utc, "/gnss0", msg


def merge(streams):
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
    global IMU_LOG, H5_PATH, POS_FILE
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument('--data-dir', type=Path, default=DATA_DIR)
    ap.add_argument('--rtklib-pos', type=Path, default=POS_FILE)
    ap.add_argument("--start-utc", type=float, default=1557425380.0)
    ap.add_argument("--end-utc", type=float, default=1557429420.0)
    ap.add_argument('--gnss-end-utc', type=float, default=1557429419.0,
                    help='inclusive GNSS bound; IMU extends beyond this to support the last update')
    ap.add_argument("--downscale", type=int, default=1,
                    help="image scale divisor; camera dimensions/intrinsics must match (default: native)")
    ap.add_argument("--compression", default="lz4", choices=["none", "lz4", "bz2"])
    ap.add_argument("--std-fix", default=None, help="hor,ver std metres for Q=1")
    ap.add_argument("--std-float", default=None, help="hor,ver std metres for Q=2")
    args = ap.parse_args()
    if args.downscale < 1 or not args.start_utc <= args.gnss_end_utc < args.end_utc:
        ap.error('require downscale >= 1 and start <= GNSS end < IMU end')
    if Path(args.out).exists(): ap.error('output bag already exists; choose a fresh path')
    IMU_LOG = args.data_dir.resolve() / 'lord_imu.log'
    H5_PATH = args.data_dir.resolve() / 'camera_data/images.h5'
    POS_FILE = args.rtklib_pos.resolve()
    if args.std_fix:
        STD_BY_Q[1] = tuple(float(v) for v in args.std_fix.split(","))
    if args.std_float:
        STD_BY_Q[2] = tuple(float(v) for v in args.std_float.split(","))
    print("GNSS stds by Q:", STD_BY_Q)

    comp = {
        "none": rosbag.Compression.NONE,
        "lz4": rosbag.Compression.LZ4,
        "bz2": rosbag.Compression.BZ2,
    }[args.compression]

    out = rosbag.Bag(args.out, "w", compression=comp)
    n = {"imu": 0, "img": 0, "gnss": 0}
    try:
        for t, topic, msg in merge(
            [
                imu_stream(args.start_utc, args.end_utc),
                image_stream(args.start_utc, args.end_utc, args.downscale),
                gnss_stream(args.start_utc, args.gnss_end_utc),
            ]
        ):
            out.write(topic, msg, rospy.Time.from_sec(t))
            if topic == "/imu0":
                n["imu"] += 1
            elif topic == "/cam0/image_raw":
                n["img"] += 1
            else:
                n["gnss"] += 1
            tot = sum(n.values())
            if tot % 50000 == 0:
                print(f"written {tot} msgs {n}", flush=True)
    finally:
        out.close()
    print("done:", n)
    report = dict(bag=str(Path(args.out).resolve()), counts=n,
                  start_utc=args.start_utc, end_utc=args.end_utc, gnss_end_utc=args.gnss_end_utc,
                  downscale=args.downscale, data_dir=str(args.data_dir.resolve()), rtklib_pos=str(POS_FILE),
                  std_by_quality=STD_BY_Q, bag_bytes=Path(args.out).stat().st_size,
                  bag_mtime_ns=Path(args.out).stat().st_mtime_ns,
                  hdf5=dict(path=str(H5_PATH), bytes=H5_PATH.stat().st_size, mtime_ns=H5_PATH.stat().st_mtime_ns))
    Path(args.out+'.preparation.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == "__main__":
    main()
