#!/usr/bin/env python3
"""Build the TEX-CUP GVINS input rosbag.

Merges, in time order, into one bag (all timestamps UTC unix seconds):
  - /imu0            sensor_msgs/Imu    from lord_imu.log (GPS week/tow -> UTC,
                     axes flipped x,z: raw Lord x-left,y-fwd,z-down ->
                     body x-right,y-fwd,z-up, matching TexCupLordImuParams)
  - /cam0/image_raw  sensor_msgs/Image  mono8, LEFT ('port') images from
                     images.h5 at native 2048x732 resolution by default
  - GNSS topics copied from the bag produced by rinex2bag
    (/ublox_driver/range_meas, /ublox_driver/ephem, /ublox_driver/iono_params)

Run inside the RoboStack env after sourcing catkin devel/setup.bash
(needed for gnss_comm python message classes).

Usage:
  python make_rosbag.py --gnss-bag gnss.bag --out texcup_gvins.bag \
      [--start-utc 1557425380.0] [--downscale 1] [--data-dir PATH]

If --downscale is changed, update the consuming camera dimensions and intrinsics
to match; the benchmark configs use native resolution.
"""
import argparse
import os
import sys
from pathlib import Path

import cv2
import h5py
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu

DATA_DIR = Path(os.environ.get("TEXCUP_DATA", Path(__file__).resolve().parents[3] / "data/tex_cup"))
IMU_LOG = str(DATA_DIR / "lord_imu.log")
H5_PATH = str(DATA_DIR / "camera_data/images.h5")
GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06 00:00:00 UTC
LEAP = 18.0


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


def image_stream(start_utc, end_utc, downscale):
    f = h5py.File(H5_PATH, "r")
    g = f["port"]
    keys = sorted(g.keys())
    for k in keys:
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


def gnss_stream(gnss_bag, end_utc):
    bag = rosbag.Bag(gnss_bag)
    for topic, msg, t in bag.read_messages():
        if t.to_sec() > end_utc:
            break
        yield t.to_sec(), topic, msg


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
    global IMU_LOG, H5_PATH
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", type=Path, default=DATA_DIR,
                    help="TEX-CUP directory (default: TEXCUP_DATA or repository data/tex_cup)")
    ap.add_argument("--gnss-bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--start-utc", type=float, default=1557425380.0)
    ap.add_argument("--end-utc", type=float, default=1e18)
    ap.add_argument("--downscale", type=int, default=1,
                    help="image scale divisor; camera dimensions/intrinsics must match (default: native)")
    ap.add_argument("--compression", default="lz4", choices=["none", "lz4", "bz2"])
    args = ap.parse_args()
    IMU_LOG = str(args.data_dir / "lord_imu.log")
    H5_PATH = str(args.data_dir / "camera_data/images.h5")

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
                gnss_stream(args.gnss_bag, args.end_utc),
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


if __name__ == "__main__":
    main()
