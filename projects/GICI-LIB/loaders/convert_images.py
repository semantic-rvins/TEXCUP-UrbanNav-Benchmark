#!/usr/bin/env python3
"""Convert TEX-CUP images.h5 group 'port' (LEFT camera) to GICI image-pack binary.

image-pack message layout (see upstream src/stream/format_image.c):
  bytes[0:6]   preamble FE CA 00 FF 00 FF
  bytes[6:9]   24-bit BE length field = 13 + W*H*step + 6  (total_len - 9)
  bytes[9:13]  uint32 BE time.time  (unix seconds, UTC timeline)
  bytes[13:17] uint32 BE nanoseconds (time.sec * 1e9)
  bytes[17:19] uint16 BE width
  bytes[19:21] uint16 BE height
  bytes[21]    uint8  step (1 = grayscale)
  bytes[22:22+W*H*step] raw image, row-major
  last 6 bytes tail CC FE FF 00 FF 00

h5 dataset attr 'tstamp' is already UTC unix seconds.
"""
import argparse
import os
from pathlib import Path
import struct

import h5py
import numpy as np

PREAMB = bytes([0xFE, 0xCA, 0x00, 0xFF, 0x00, 0xFF])
TAIL = bytes([0xCC, 0xFE, 0xFF, 0x00, 0xFF, 0x00])


def main():
    ap = argparse.ArgumentParser()
    data = Path(os.environ.get('TEXCUP_DATA', Path(__file__).resolve().parents[3]/'data/tex_cup'))
    ap.add_argument('--h5', default=str(data/'camera_data/images.h5'))
    ap.add_argument('--group', default='port')
    ap.add_argument('--out', required=True)
    ap.add_argument('--cut-utc-unix', type=float, default=1557425380.0)
    ap.add_argument('--end-utc-unix', type=float, default=None,
                    help='exclusive end bound (for generating a head segment)')
    a = ap.parse_args()

    n_out = 0
    with h5py.File(a.h5, 'r') as f, open(a.out, 'wb') as g:
        grp = f[a.group]
        keys = sorted(grp.keys())
        for k in keys:
            ds = grp[k]
            t = float(ds.attrs['tstamp'])
            if t < a.cut_utc_unix:
                continue
            if a.end_utc_unix is not None and t >= a.end_utc_unix:
                break
            img = np.ascontiguousarray(ds[()], dtype=np.uint8)
            h, w = img.shape
            step = 1
            n = w * h * step
            sec = int(t)
            nsec = int(round((t - sec) * 1e9))
            if nsec >= 1_000_000_000:
                sec += 1; nsec -= 1_000_000_000
            length_field = 13 + n + 6
            hdr = struct.pack('>IIHHB', sec, nsec, w, h, step)
            g.write(PREAMB)
            g.write(length_field.to_bytes(3, 'big'))
            g.write(hdr)
            g.write(img.tobytes())
            g.write(TAIL)
            n_out += 1
    print(f"images: {n_out} frames written to {a.out}")


if __name__ == '__main__':
    main()
