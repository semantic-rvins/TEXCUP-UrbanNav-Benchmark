#!/usr/bin/env python3
"""Convert TEX-CUP (2019-05-09) to the OKVIS2-X extended-EuRoC folder format.

Produces (under --out):
  cam0/data/*.png + cam0/data.csv   port  (LEFT)  images, 2048x732 mono8
  cam1/data/*.png + cam1/data.csv   star  (RIGHT) images (timestamp of the port
                                    frame with the same h5 key is used for both
                                    cameras: same hardware trigger, skew < 1 ms)
  imu0/data.csv                     Lord 3DM-GX5-25 @100 Hz, EuRoC order
                                    (t[ns], gyro xyz [rad/s], acc xyz [m/s2]),
                                    axes flipped x,z: raw Lord x-left/y-fwd/z-down
                                    -> body x-right/y-fwd/z-up (same convention as
                                    projects/GVINS/loaders/make_rosbag.py)
  gps0/data.csv                     RTKLIB-EX antenna solution as
                                    "cartesian": t[ns], ENU x,y,z [m] anchored at
                                    the FIRST ground-truth point (= common/evaluate.py
                                    anchor), fix/float per-axis sigmas defined by
                                    GPS_SIGMA_BY_Q; only Q1/Q2 solutions retained.
  gps0/anchor.json                  anchor lat/lon/h for the inverse mapping.

All timestamps UTC unix ns. Start cut 18:09:40 UTC (unix 1557425380.0); IMU is
exported from 1 s earlier so the first frame is covered by IMU data.
"""
import argparse
import concurrent.futures as cf
import csv
import json
import math
import os
from pathlib import Path

import cv2
import h5py
import numpy as np

BENCH = Path(__file__).resolve().parents[3]
DATA_DIR = Path(os.environ.get("TEXCUP_DATA", BENCH / "data/tex_cup"))
IMU_LOG = DATA_DIR / "lord_imu.log"
H5_PATH = DATA_DIR / "camera_data/images.h5"
GT_LOG = DATA_DIR / "ground_truth.log"
# Benchmark RTKLIB-EX solution: antenna ECEF, UTC seconds of day, quality flag.
RTKLIB_CSV = BENCH / "results/final/RTKLIB.est.csv"
# Per-axis sigma (E,N,U) [m] as a function of the RTKLIB quality flag q.
#   q=1 fix   -> 0.05 m horizontal / 0.10 m vertical
#   q=2 float -> 0.50 m horizontal / 1.00 m vertical
# q=4 (DGPS/single) is excluded, matching the repo IC-GVINS convention
# ("retains Q1/Q2 and excludes Q4"). Mapping documented in RUN.md / BUGFIX_NOTES.md.
GPS_SIGMA_BY_Q = {1: (0.05, 0.05, 0.10), 2: (0.50, 0.50, 1.00)}
GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06 00:00:00 UTC
LEAP = 18.0
DAY0_UNIX = 1557360000.0  # 2019-05-09 00:00:00 UTC
START_UTC = 1557425380.0  # 18:09:40 UTC
A = 6378137.0
E2 = 6.69437999014e-3


def lla2ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon)
    N = A / np.sqrt(1 - E2 * np.sin(la) ** 2)
    return np.stack([(N + h) * np.cos(la) * np.cos(lo),
                     (N + h) * np.cos(la) * np.sin(lo),
                     (N * (1 - E2) + h) * np.sin(la)], -1)


def enu_rotation(ref_lla):
    la, lo = math.radians(ref_lla[0]), math.radians(ref_lla[1])
    return np.array([
        [-math.sin(lo), math.cos(lo), 0],
        [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
        [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def first_gt_lla():
    for line in open(GT_LOG):
        if line.startswith("#") or not line.strip():
            continue
        p = line.split()
        if len(p) < 6 or "/" not in p[0]:
            continue
        return (float(p[3]), float(p[4]), float(p[5]))
    raise RuntimeError("no GT line found")


def convert_imu(out):
    n = 0
    with open(IMU_LOG) as f, open(f"{out}/imu0/data.csv", "w") as o:
        o.write("#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            p = line.split(", ")
            if len(p) < 9:
                continue
            t_gps = int(p[0]) * 604800.0 + float(p[1]) + float(p[2])
            t_utc = GPS_EPOCH_UNIX + t_gps - LEAP
            if t_utc < START_UTC - 1.0:
                continue
            ax, ay, az = -float(p[3]), float(p[4]), -float(p[5])
            gx, gy, gz = -float(p[6]), float(p[7]), -float(p[8])
            ns = int(round(t_utc * 1e9))
            o.write(f"{ns},{gx:.9f},{gy:.9f},{gz:.9f},{ax:.9f},{ay:.9f},{az:.9f}\n")
            n += 1
    print("imu rows:", n, flush=True)


def convert_gps(out):
    """Build gps0/data.csv from the reconciled RTKLIB-EX solution RTKLIB.est.csv.

    Input columns: utc_sec (UTC seconds of day 2019-05-09), ecef_x/y/z (ALT1
    antenna ECEF metres), q (RTKLIB quality flag). Output = "cartesian" ENU
    anchored at the first ground-truth point (= the frame G of the OKVIS global
    trajectory and the same anchor global_to_est.py inverts with). Per-axis
    sigmas come from q via GPS_SIGMA_BY_Q; q not in that map (q=4) is dropped.
    """
    ref = first_gt_lla()
    ref_ecef = lla2ecef(*ref)
    R = enu_rotation(ref)  # ECEF -> ENU
    n = 0
    skipped = {}
    with open(RTKLIB_CSV) as f, open(f"{out}/gps0/data.csv", "w") as o:
        o.write("#timestamp [ns],x,y,z,sigma_x,sigma_y,sigma_z\n")
        for row in csv.DictReader(f):
            q = int(float(row["q"]))
            if q not in GPS_SIGMA_BY_Q:
                skipped[q] = skipped.get(q, 0) + 1
                continue
            utc_sec = float(row["utc_sec"])          # already UTC seconds of day
            t_utc = DAY0_UNIX + utc_sec
            xyz = np.array([float(row["ecef_x"]), float(row["ecef_y"]),
                            float(row["ecef_z"])])
            enu = R @ (xyz - ref_ecef)
            sig = GPS_SIGMA_BY_Q[q]
            ns = int(round(t_utc * 1e9))
            o.write(f"{ns},{enu[0]:.4f},{enu[1]:.4f},{enu[2]:.4f},"
                    f"{sig[0]:.3f},{sig[1]:.3f},{sig[2]:.3f}\n")
            n += 1
    with open(f"{out}/gps0/anchor.json", "w") as o:
        json.dump({"ref_lla": list(ref), "note": "ENU anchor = first GT point; "
                   "frame G of okvis global trajectory = this ENU frame",
                   "source": "results/final/RTKLIB.est.csv (RTKLIB-EX)",
                   "sigma_by_q": {str(k): v for k, v in GPS_SIGMA_BY_Q.items()}},
                  o, indent=1)
    print("gps rows:", n, "skipped (q not in {1,2}):", skipped, "anchor:", ref,
          flush=True)


def _write_png(args):
    path, arr = args
    cv2.imwrite(path, arr, [cv2.IMWRITE_PNG_COMPRESSION, 1])


def convert_images(out, workers):
    f = h5py.File(H5_PATH, "r")
    port, star = f["port"], f["star"]
    keys = sorted(port.keys())
    rows = []
    for k in keys:
        t = float(port[k].attrs["tstamp"])
        if t < START_UTC:
            continue
        rows.append((k, int(round(t * 1e9))))
    print("frames to export:", len(rows), flush=True)
    for cam in ("cam0", "cam1"):
        os.makedirs(f"{out}/{cam}/data", exist_ok=True)
    with open(f"{out}/cam0/data.csv", "w") as o0, open(f"{out}/cam1/data.csv", "w") as o1:
        o0.write("#timestamp [ns],filename\n")
        o1.write("#timestamp [ns],filename\n")
        for k, ns in rows:
            o0.write(f"{ns},{ns}.png\n")
            o1.write(f"{ns},{ns}.png\n")
    done = 0
    batch = []
    with cf.ThreadPoolExecutor(max_workers=workers) as ex:
        for k, ns in rows:
            p0 = f"{out}/cam0/data/{ns}.png"
            p1 = f"{out}/cam1/data/{ns}.png"
            if not (os.path.exists(p0) and os.path.exists(p1)):
                batch.append(ex.submit(_write_png, (p0, port[k][()])))
                batch.append(ex.submit(_write_png, (p1, star[k][()])))
            if len(batch) >= 512:
                for fut in batch:
                    fut.result()
                batch = []
            done += 1
            if done % 4000 == 0:
                print("frames done:", done, flush=True)
        for fut in batch:
            fut.result()
    print("images done:", done, flush=True)


def main():
    global IMU_LOG, H5_PATH, GT_LOG, RTKLIB_CSV
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--data-dir", type=Path, default=DATA_DIR,
                    help="TEXCUP_DATA or repository data/tex_cup by default")
    ap.add_argument("--rtklib-csv", type=Path, default=RTKLIB_CSV,
                    help="RTKLIB-EX antenna ECEF CSV with UTC seconds and q flag")
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--skip-images", action="store_true")
    ap.add_argument("--skip-imu", action="store_true")
    ap.add_argument("--skip-gps", action="store_true")
    args = ap.parse_args()
    IMU_LOG = args.data_dir / "lord_imu.log"
    H5_PATH = args.data_dir / "camera_data/images.h5"
    GT_LOG = args.data_dir / "ground_truth.log"
    RTKLIB_CSV = args.rtklib_csv
    for d in ("imu0", "gps0"):
        os.makedirs(f"{args.out}/{d}", exist_ok=True)
    if not args.skip_imu:
        convert_imu(args.out)
    if not args.skip_gps:
        convert_gps(args.out)
    if not args.skip_images:
        convert_images(args.out, args.workers)


if __name__ == "__main__":
    main()
