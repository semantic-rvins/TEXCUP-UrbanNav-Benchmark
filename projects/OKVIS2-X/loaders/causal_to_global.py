#!/usr/bin/env python3
"""Map the CAUSAL okvis2-vio_trajectory.csv (odometry frame W) into the GNSS frame G.

The non-ROS OKVIS2-X app never writes a causal globally-referenced trajectory; T_GW is
only applied at shutdown in writeGlobalCsvTrajectory. This tool recovers the pipeline's
own final T_GW exactly (rigid fit between the final W-frame trajectory + antenna lever
arm and the exported global trajectory; residual ~0 by construction) and applies that
single transform to the causal states. Diagnostic only: a single posthoc T_GW cannot
remove causal drift.

Output csv has the same "timestamp, p_GA_G_*" layout as the okvis global csvs, so it can
be fed to global_to_est.py.
"""
import argparse

import numpy as np

R_SA = np.array([-0.610, -0.052, 0.01])  # antenna in IMU/body frame


def quat_to_R(q):  # x, y, z, w
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def antenna_positions(arr):
    return np.array([arr[i, 1:4] + quat_to_R(arr[i, 4:8]) @ R_SA for i in range(len(arr))])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--final", required=True, help="okvis2-vio-final[(-ba)]_trajectory.csv (W frame)")
    ap.add_argument("--global-final", dest="glob", required=True,
                    help="matching okvis2-vio-global-final[(-ba)]_trajectory.csv (G frame)")
    ap.add_argument("--causal", required=True, help="okvis2-vio_trajectory.csv (W frame)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    fin = np.loadtxt(args.final, delimiter=",", skiprows=1, usecols=range(8))
    glo = np.loadtxt(args.glob, delimiter=",", skiprows=1)
    assert np.array_equal(fin[:, 0], glo[:, 0]), "timestamp mismatch final vs global"
    aW = antenna_positions(fin)
    G = glo[:, 1:4]
    cA, cG = aW.mean(0), G.mean(0)
    H = (aW - cA).T @ (G - cG)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = cG - R @ cA
    res = np.linalg.norm((aW @ R.T + t) - G, axis=1)
    print(f"T_GW fit residual max: {res.max():.6f} m (should be ~0)")

    cau = np.loadtxt(args.causal, delimiter=",", skiprows=1, usecols=range(8))
    gC = antenna_positions(cau) @ R.T + t
    out = np.column_stack([cau[:, 0], gC])
    np.savetxt(args.out, out, delimiter=",",
               header="timestamp,p_GA_G_x,p_GA_G_y,p_GA_G_z", comments="",
               fmt="%.0f,%.6f,%.6f,%.6f")
    print("causal rows:", len(out))


if __name__ == "__main__":
    main()
