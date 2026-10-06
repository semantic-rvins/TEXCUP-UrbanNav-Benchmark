#!/usr/bin/env python3
"""Check RRR antenna translation, exact logged origin, NMEA and scoring support."""
import argparse
from collections import Counter
import csv
from functools import reduce
import json
import operator
from pathlib import Path
import re
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'common'))
from evaluate import lla2ecef, ecef2enu


def tod(value):
    return int(value[:2])*3600+int(value[2:4])*60+float(value[4:])


def coordinate(value, degree_digits, sign):
    return (int(value[:degree_digits])+float(value[degree_digits:])/60)*sign


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    args = parser.parse_args()
    run = Path(args.run).resolve()
    assert json.loads((run/'status.json').read_text())['phase']=='completed'
    origin = json.loads((run/'enu_origin.json').read_text())
    manifest = json.loads((run/'run_manifest.json').read_text())
    gga, esa, checksums = {}, {}, 0
    for line in (run/'solution.nmea').read_text().splitlines():
        if not line: continue
        assert line.startswith('$') and '*' in line
        body, checksum = line[1:].split('*',1)
        assert reduce(operator.xor, body.encode(), 0)==int(checksum[:2],16)
        checksums += 1
        fields = body.split(',')
        if fields[0]=='GPGGA' and int(fields[6])>=1:
            lat=coordinate(fields[2],2,1 if fields[3]=='N' else -1)
            lon=coordinate(fields[4],3,1 if fields[5]=='E' else -1)
            key=round(tod(fields[1]),3)
            assert key not in gga, 'Duplicate GGA timestamp'
            gga[key]=[lat, lon, float(fields[9])+float(fields[11])]
        elif fields[0]=='GPESA':
            esa[round(tod(fields[1]),3)]=list(map(float,fields[5:8]))
    with (run/'est.csv').open() as source:
        rows=list(csv.DictReader(source))
    times=np.array([float(r['utc_sec']) for r in rows])
    assert np.array_equal(times, sorted(gga)) and np.all(np.diff(times)>0)
    assert all(t in esa for t in times)
    body_lla=np.array([gga[t] for t in times])
    antenna_lla=np.array([[float(r[k]) for k in ('lat_deg','lon_deg','h_ell')] for r in rows])
    quaternions=np.array([[float(r[k]) for k in ('qw','qx','qy','qz')] for r in rows])
    assert np.isfinite(antenna_lla).all() and np.isfinite(quaternions).all()
    assert np.max(np.abs(np.linalg.norm(quaternions,axis=1)-1))<2e-6
    body=lla2ecef(*body_lla.T)
    antenna=lla2ecef(*antenna_lla.T)
    # Independently expand Rz(yaw) Ry(pitch) Rx(roll) times the configured lever.
    roll,pitch,yaw=np.radians(np.array([esa[t] for t in times])).T
    sr,cr=np.sin(roll),np.cos(roll)
    sp,cp=np.sin(pitch),np.cos(pitch)
    sy,cy=np.sin(yaw),np.cos(yaw)
    x,y,z=manifest['lever_arm_rfu_m']
    expected=np.stack([cp*cy*x+(sr*sp*cy-cr*sy)*y+(cr*sp*cy+sr*sy)*z,
                       cp*sy*x+(sr*sp*sy+cr*cy)*y+(cr*sp*sy-sr*cy)*z,
                       -sp*x+sr*cp*y+cr*cp*z],axis=1)
    observed=ecef2enu(antenna,origin['lla_deg_m'])-ecef2enu(body,origin['lla_deg_m'])
    max_translation_error=float(np.linalg.norm(observed-expected,axis=1).max())
    assert max_translation_error<0.0002
    np.testing.assert_allclose(lla2ecef(*origin['lla_deg_m']),origin['ecef_m'],atol=1e-6,rtol=0)
    grid=np.loadtxt(run/'hor_err_timeseries.csv',delimiter=',',skiprows=1)
    assert np.array_equal(grid[:,0],np.arange(65380,69420))
    log=(run/'logs/gici.log').read_text(errors='replace')
    assert 'BENCHMARK_POST_FILES_EOF' in log
    outlier_lines=[line for line in log.splitlines() if 'gnss_estimator_base.cpp' in line and 'Rejected' in line]
    signals=Counter()
    for line in outlier_lines:
        for prn,code in re.findall(r'([A-Z]\d{2})\|([A-Z0-9]+)',line):
            signals[f'{prn[0]}|{code}']+=1
            assert prn[0] in 'GEC', line
    report=dict(run_dir=str(run), checked_antenna_positions=len(rows),
        all_gga_positions_have_matching_esa=True, nmea_valid_checksums=checksums,
        first_utc_sec=float(times[0]), last_utc_sec=float(times[-1]),
        lever_arm_rfu_m=[x,y,z], enu_origin=origin,
        max_antenna_translation_verification_error_m=max_translation_error,
        reference_point='ALT1 antenna; one estimated-attitude lever translation; no GT shift',
        n_window=4040, n_solved=int(np.isfinite(grid[:,1]).sum()),
        post_file_eof_logged=True, gici_returncode=manifest['gici_returncode'],
        estimator_resets=log.count('Reset estimator because it is diverge!'),
        logged_error_lines=(run/'logs/errors.log').read_text().splitlines(),
        observed_outlier_signal_counts=dict(signals),
        signal_verification_limit='These are outlier diagnostics, not a complete inventory of accepted factors; measurement-level system_exclude is R',
        source_script=Path(__file__).name)
    (run/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(f'Checked {len(rows)} antenna positions; maximum lever verification difference {max_translation_error:.9f} m')


if __name__=='__main__':
    main()
