#!/usr/bin/env python3
"""Cut a RINEX 3 observation file: drop all epochs before a given GPST time.

Header is copied unchanged (TIME OF FIRST OBS left as-is; decoders derive epoch
times from the '>' epoch records). Epoch records start with '> YYYY MM DD HH MM SS...'.
"""
import argparse
from datetime import datetime, timedelta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--cut-gpst', default='2019-05-09 18:09:58',
                    help='GPST cut time YYYY-MM-DD HH:MM:SS (default = 18:09:40 UTC + 18 s)')
    ap.add_argument('--end-gpst', default=None,
                    help='optional inclusive GPST end YYYY-MM-DD HH:MM:SS')
    a = ap.parse_args()

    cut = datetime.strptime(a.cut_gpst, '%Y-%m-%d %H:%M:%S')
    end = datetime.strptime(a.end_gpst, '%Y-%m-%d %H:%M:%S') if a.end_gpst else None
    if end is not None and end < cut:
        ap.error('--end-gpst must not precede --cut-gpst')
    n_kept = n_drop = 0
    with open(a.inp) as f, open(a.out, 'w') as g:
        in_header = True
        keep = False
        for line in f:
            if in_header:
                g.write(line)
                if 'END OF HEADER' in line:
                    in_header = False
                continue
            if line.startswith('>'):
                p = line.split()
                t = datetime(int(p[1]), int(p[2]), int(p[3]),
                             int(p[4]), int(p[5])) + timedelta(seconds=float(p[6]))
                keep = t >= cut and (end is None or t <= end)
                if keep:
                    n_kept += 1
                else:
                    n_drop += 1
            if keep:
                g.write(line)
    print(f"{a.out}: kept {n_kept} epochs, dropped {n_drop}")


if __name__ == '__main__':
    main()
