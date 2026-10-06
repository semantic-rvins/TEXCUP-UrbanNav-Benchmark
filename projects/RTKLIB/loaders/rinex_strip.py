#!/usr/bin/env python3
"""Strip observable types from a RINEX 3.0x observation file (column surgery).

Protocol signal set for TEX-CUP dual-freq runs: GPS C1C+C2W, GAL C1C+C7Q (E1+E5b),
BDS C2I (B1I). This removes the GPS L2C set (C2L/L2L/D2L/S2L) and the GAL E5a set
(C5Q/L5Q/D5Q/S5Q) from both the SYS / # / OBS TYPES header records and the
per-epoch observation columns, keeping everything else byte-identical.

Handles: per-system obs-type ordering differences (rover has E 5Q before 7Q,
base the reverse), header continuation lines (>13 types), short obs lines
(trailing blanks trimmed), event epochs (flag>1: records copied verbatim).

Usage: rinex_strip.py in.obs out.obs [SYS:BANDATTR ...]
       default strip spec: G:2L E:5Q
"""
import sys

FLD = 16  # 14.3f value + LLI + SSI per RINEX 3 observation field
LBL = 'SYS / # / OBS TYPES'


def parse_strip(args):
    spec = {}
    for a in (args or ['G:2L', 'E:5Q']):
        s, ba = a.split(':')
        spec.setdefault(s, set()).add(ba)
    return spec


def main():
    fin, fout = sys.argv[1], sys.argv[2]
    strip = parse_strip(sys.argv[3:])
    out = open(fout, 'w')
    lines = open(fin)
    # ---- header: collect SYS/#/OBS TYPES (with continuations), rewrite filtered
    orig = {}   # sys -> original obs type list (file order)
    keep = {}   # sys -> indices kept
    pend_sys = pend_n = None
    for line in lines:
        label = line[60:80].strip()
        if label == LBL:
            if line[0] != ' ':                       # first line of a system
                pend_sys = line[0]
                pend_n = int(line[3:6])
                orig[pend_sys] = []
            orig[pend_sys] += line[7:60].split()
            if len(orig[pend_sys]) < pend_n:
                continue                             # continuation follows
            s = pend_sys
            keep[s] = [i for i, t in enumerate(orig[s])
                       if t[1:] not in strip.get(s, ())]
            kept = [orig[s][i] for i in keep[s]]
            for c0 in range(0, len(kept), 13):       # rewrite (13 types/line)
                chunk = kept[c0:c0 + 13]
                head = '%c  %3d' % (s, len(kept)) if c0 == 0 else '      '
                body = head + ''.join(' ' + t for t in chunk)
                out.write('%-60s%s\n' % (body, LBL))
            continue
        out.write(line)
        if label == 'END OF HEADER':
            break
    # ---- body
    copy_n = 0
    for line in lines:
        if copy_n > 0:                               # event records: verbatim
            copy_n -= 1
            out.write(line)
            continue
        if line.startswith('>'):
            p = line.split()
            flag, nsat = int(p[7]), int(p[8])
            if flag > 1:
                copy_n = nsat
            out.write(line)
            continue
        s = line[0]
        if s not in keep:                            # unknown sys: keep as-is
            out.write(line)
            continue
        body = line.rstrip('\n')[3:]
        body += ' ' * (FLD * len(orig[s]) - len(body))
        newf = ''.join(body[i * FLD:(i + 1) * FLD] for i in keep[s])
        out.write((line[:3] + newf).rstrip() + '\n')
    out.close()


if __name__ == '__main__':
    main()
