#!/usr/bin/env python3
"""Parse a GICI NMEA solution file (GGA + optional ESA) into est.csv for common/evaluate.py.

GGA: $GPGGA,tod,lat ddmm.mmmmmmm,N/S,lon dddmm.mmmmmmm,E/W,q,ns,dop,alt(MSL),M,geoid_sep,M,age,refid
     -> ellipsoidal h = alt + geoid_sep; tod is UTC time of day.
ESA: $GPESA,tod,Ve,Vn,Vu,roll,pitch,yaw  (deg; attitude of GICI body frame wrt ENU,
     intrinsic ZYX i.e. R_enu_body = Rz(yaw)Ry(pitch)Rx(roll); see upstream
     src/utility/transform.cpp quaternionToEulerAngle).

In GICI fusion modes the GGA position is the body (=IMU, with body_to_imu_rotation=0)
position. Pass --lever-arm "x,y,z" (t_imu_to_ant_in_b, IMU z-up frame) and a fixed
ENU origin to translate to the GNSS antenna:
p_ant = p_body + R_ecef_enu(origin_lat,origin_lon) @ R_enu_body @ t.
ESA attitude is relative to GICI's fixed initial ENU frame. Use --enu-origin-lla
for a recorded estimator origin, or --enu-origin-rtklib for a nearby approximation
from the first valid RTKLIB est.csv position. The latter is not GICI's hidden SPP
origin and affects only the lever rotation, not the GGA body coordinates.
For GNSS-only estimators (RTK) the GGA position is already the antenna: no lever arm.

Output columns: utc_sec (seconds of day), lat_deg, lon_deg, h_ell, qw,qx,qy,qz (if ESA).
Quaternion columns retain the ESA body-to-fixed-ENU attitude, not body-to-ECEF.
"""
import argparse
import csv
import math

import numpy as np


def dm2deg(dm, width):
    d = int(dm[:width]); m = float(dm[width:])
    return d + m / 60.0


def parse_tod(s):
    return int(s[0:2]) * 3600 + int(s[2:4]) * 60 + float(s[4:])


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rpy_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)  # w,x,y,z


def ecef_enu_rot(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    # columns: E, N, U in ECEF
    return np.array([
        [-math.sin(lo), -math.sin(la) * math.cos(lo), math.cos(la) * math.cos(lo)],
        [math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la) * math.sin(lo)],
        [0.0, math.cos(la), math.sin(la)]])


A_WGS = 6378137.0
E2 = 6.69437999014e-3


def lla2ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    N = A_WGS / math.sqrt(1 - E2 * math.sin(la) ** 2)
    return np.array([(N + h) * math.cos(la) * math.cos(lo),
                     (N + h) * math.cos(la) * math.sin(lo),
                     (N * (1 - E2) + h) * math.sin(la)])


def ecef2lla(p):
    x, y, z = p
    lon = math.atan2(y, x)
    r = math.hypot(x, y)
    lat = math.atan2(z, r * (1 - E2))
    for _ in range(6):
        N = A_WGS / math.sqrt(1 - E2 * math.sin(lat) ** 2)
        h = r / math.cos(lat) - N
        lat = math.atan2(z, r * (1 - E2 * N / (N + h)))
    N = A_WGS / math.sqrt(1 - E2 * math.sin(lat) ** 2)
    h = r / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), h


def vector3(value):
    try:
        values = tuple(float(v) for v in value.split(','))
    except ValueError as exc:
        raise argparse.ArgumentTypeError('expected three comma-separated numbers') from exc
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise argparse.ArgumentTypeError('expected three finite comma-separated numbers')
    return values


def first_rtklib_origin(path):
    """Return (LLA, UTC seconds of day) from the first valid ECEF CSV row."""
    with open(path, newline='') as source:
        reader = csv.DictReader(source)
        required = {'utc_sec', 'ecef_x', 'ecef_y', 'ecef_z'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'{path}: RTKLIB CSV requires {sorted(required)}')
        for row in reader:
            try:
                tod = float(row['utc_sec'])
                position = np.array([float(row[k]) for k in ('ecef_x', 'ecef_y', 'ecef_z')])
                quality = float(row.get('q', row.get('quality', '1')))
            except (ValueError, TypeError):
                continue
            if (not math.isfinite(tod) or not np.isfinite(position).all()
                    or not math.isfinite(quality) or quality <= 0
                    or np.linalg.norm(position) == 0):
                continue
            return ecef2lla(position), tod
    raise ValueError(f'{path}: no valid RTKLIB ECEF position')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nmea', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--lever-arm', type=vector3, default=None,
                    help='"x,y,z" t_imu_to_ant_in_b; requires ESA attitude in file')
    origin = ap.add_mutually_exclusive_group()
    origin.add_argument('--enu-origin-rtklib', metavar='EST_CSV',
                        help='use first valid RTKLIB ECEF row as approximate fixed ENU origin')
    origin.add_argument('--enu-origin-lla', type=vector3, metavar='LAT,LON,H',
                        help='fixed estimator ENU origin: latitude/longitude degrees, ellipsoidal height m')
    ap.add_argument('--min-quality', type=int, default=1,
                    help='min GGA quality indicator to keep (default 1 = any fix)')
    a = ap.parse_args()

    lever = None
    R_ecef_enu = None
    if a.lever_arm is not None:
        if a.enu_origin_rtklib is None and a.enu_origin_lla is None:
            ap.error('--lever-arm requires --enu-origin-rtklib or --enu-origin-lla')
        lever = np.array(a.lever_arm)
        if a.enu_origin_rtklib is not None:
            try:
                origin_lla, origin_tod = first_rtklib_origin(a.enu_origin_rtklib)
            except (OSError, ValueError) as exc:
                ap.error(str(exc))
            origin_source = f'approximate RTKLIB origin from {a.enu_origin_rtklib}, utc_sec={origin_tod:.3f}'
        else:
            origin_lla = a.enu_origin_lla
            if not (-90 <= origin_lla[0] <= 90 and -180 <= origin_lla[1] <= 180):
                ap.error('--enu-origin-lla latitude/longitude must be within [-90,90]/[-180,180]')
            origin_source = 'explicit fixed ENU origin'
        R_ecef_enu = ecef_enu_rot(*origin_lla[:2])
        print(f'{origin_source}: LLA=({origin_lla[0]:.9f},{origin_lla[1]:.9f},{origin_lla[2]:.4f})')
    elif a.enu_origin_rtklib is not None or a.enu_origin_lla is not None:
        ap.error('an ENU origin is only used with --lever-arm')

    gga = {}   # tod -> (lat, lon, h_ell, q)
    esa = {}   # tod -> (roll, pitch, yaw) rad
    for line in open(a.nmea, errors='replace'):
        line = line.strip()
        if not line.startswith('$'):
            continue
        body = line.split('*')[0]
        f = body.split(',')
        typ = f[0][3:]
        if typ == 'GGA':
            if len(f) < 12 or not f[1] or not f[2]:
                continue
            tod = parse_tod(f[1])
            lat = dm2deg(f[2], 2) * (1 if f[3] == 'N' else -1)
            lon = dm2deg(f[4], 3) * (1 if f[5] == 'E' else -1)
            qual = int(f[6])
            if qual < a.min_quality:
                continue
            h_ell = float(f[9]) + float(f[11])
            gga[round(tod, 3)] = (lat, lon, h_ell, qual)
        elif typ == 'ESA':
            if len(f) < 8 or not f[1]:
                continue
            tod = parse_tod(f[1])
            esa[round(tod, 3)] = tuple(math.radians(float(v)) for v in f[5:8])

    rows = []
    for tod in sorted(gga.keys()):
        lat, lon, h, qual = gga[tod]
        att = esa.get(tod)
        if lever is not None:
            if att is None:
                continue  # cannot apply lever arm without attitude
            R_wb = rpy_to_R(*att)
            d_enu = R_wb @ lever
            p = lla2ecef(lat, lon, h) + R_ecef_enu @ d_enu
            lat, lon, h = ecef2lla(p)
        row = dict(utc_sec=f'{tod:.3f}', lat_deg=f'{lat:.9f}',
                   lon_deg=f'{lon:.9f}', h_ell=f'{h:.4f}', quality=qual)
        if att is not None:
            qw, qx, qy, qz = rpy_to_quat(*att)
            row.update(qw=f'{qw:.6f}', qx=f'{qx:.6f}', qy=f'{qy:.6f}', qz=f'{qz:.6f}')
        rows.append(row)

    if not rows:
        raise SystemExit('no solutions parsed')
    fields = list(rows[-1].keys())
    with open(a.out, 'w', newline='') as g:
        w = csv.DictWriter(g, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f'{a.out}: {len(rows)} epochs '
          f'({sum(1 for r in rows if int(r["quality"]) == 4)} fixed)')


if __name__ == '__main__':
    main()
