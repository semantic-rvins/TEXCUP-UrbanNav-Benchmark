"""Check exclusion of unaligned output and an independently known ALT1 transform."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

path = Path(__file__).resolve().parents[1]/'loaders/extract_est.py'
spec = importlib.util.spec_from_file_location('vins_extract',path)
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)


class ExtractionTest(unittest.TestCase):
    def test_only_aligned_output_and_rotated_antenna_in_ecef(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root/'anchor.log').write_text('GLOBAL_FUSION_ANCHOR_LLA 0.0 0.0 0.0\n')
            np.savetxt(root/'global.csv',[
                [100e9,100,100,100,1,0,0,0],
                [101e9,1,2,3,2**-.5,0,0,2**-.5]],delimiter=',')
            extract.mode_global(argparse.Namespace(anchor_log=root/'anchor.log',
                global_csv=root/'global.csv',after_utc=100,out=root/'est.csv'))
            row = np.genfromtxt(root/'est.csv',delimiter=',',names=True)
            self.assertEqual(row.shape,())
            self.assertAlmostEqual(float(row['utc_sec']),101-extract.MIDNIGHT_UNIX)
            # 90-degree yaw rotates the RFU lever to [.052,-.610,.010].
            # At lat=lon=0, U=X, E=Y and N=Z in ECEF.
            np.testing.assert_allclose([row['ecef_x'],row['ecef_y'],row['ecef_z']],
                [6378137+3.01,1.052,1.39],atol=1e-4,rtol=0)


if __name__ == '__main__':
    unittest.main()
