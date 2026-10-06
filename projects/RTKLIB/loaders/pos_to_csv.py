#!/usr/bin/env python3
"""Convert an RTKLIB .pos file (xyz-ecef, GPST time tags) to the common est.csv.

- .pos time tags are GPST; utc_sec = GPST seconds-of-day - 18 (GPS-UTC leap on
  2019-05-09). All epochs are within one UTC day, no wrap handling needed.
- Position is the rover GNSS antenna (RTKLIB solution) -> compare to GT directly.
- Also writes eval_extra.json with solution-quality (Q) statistics: fix rate etc.

Usage: pos_to_csv.py in.pos out.csv [extra.json]
"""
import json, sys
from collections import Counter

LEAP = 18.0  # GPST - UTC seconds on 2019-05-09
QNAME = {1: 'fix', 2: 'float', 3: 'sbas', 4: 'dgps', 5: 'single', 6: 'ppp'}

def main():
    pos_file, csv_file = sys.argv[1], sys.argv[2]
    extra_file = sys.argv[3] if len(sys.argv) > 3 else None
    rows, qcount = [], Counter()
    for line in open(pos_file):
        if line.startswith('%') or not line.strip():
            continue
        p = line.split()
        # 2019/05/09 hh:mm:ss.sss  x  y  z  Q  ns ...
        hh, mm, ss = p[1].split(':')
        gpst_sec = int(hh) * 3600 + int(mm) * 60 + float(ss)
        q = int(p[5])
        qcount[q] += 1
        rows.append((gpst_sec - LEAP, float(p[2]), float(p[3]), float(p[4]), q))
    with open(csv_file, 'w') as f:
        f.write('utc_sec,ecef_x,ecef_y,ecef_z,q\n')
        for r in rows:
            f.write('%.3f,%.4f,%.4f,%.4f,%d\n' % r)
    n = len(rows)
    if extra_file:
        extra = {
            'n_solutions': n,
            'q_counts': {QNAME.get(k, str(k)): v for k, v in sorted(qcount.items())},
            'fix_rate_pct': round(100.0 * qcount[1] / n, 2),
            'float_rate_pct': round(100.0 * qcount[2] / n, 2),
        }
        json.dump(extra, open(extra_file, 'w'), indent=1)
        print(json.dumps(extra, indent=1))
    print('wrote %d epochs -> %s' % (n, csv_file))

if __name__ == '__main__':
    main()
