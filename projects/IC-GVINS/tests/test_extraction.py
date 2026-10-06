import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

EXTRACT=Path(__file__).resolve().parents[1]/'loaders/extract_est.py'


class AntennaConversionTest(unittest.TestCase):
    def test_ned_frd_lever_and_gpst_timestamp(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            # At lat=lon=0, N=ECEF Z, E=ECEF Y, D=-ECEF X.
            # Yaw 90 degrees rotates FRD lever to N=.610, E=-.052, D=-.010.
            (p/'nav').write_text('0 410998 0 0 0 0 0 0 0 0 90\n')
            subprocess.run([sys.executable,str(EXTRACT),'--nav',str(p/'nav'),'--out',str(p/'est.csv')],check=True,capture_output=True)
            r=np.genfromtxt(p/'est.csv',delimiter=',',names=True)
            self.assertEqual(float(r['utc_sec']),65380)
            np.testing.assert_allclose([r['ecef_x'],r['ecef_y'],r['ecef_z']],
                [6378137+.010,-.052,.610],atol=1e-4,rtol=0)


if __name__=='__main__':unittest.main()
