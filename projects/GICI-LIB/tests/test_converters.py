"""Focused regressions for GICI's fixed-ENU antenna conversion and input time cut."""
import csv
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np


LOADERS = Path(__file__).resolve().parents[1] / 'loaders'
SPEC = importlib.util.spec_from_file_location('gici_nmea', LOADERS / 'gici_nmea_to_csv.py')
GICI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GICI)


class ConverterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def run_converter(self, *args):
        return subprocess.run(
            [sys.executable, str(LOADERS / 'gici_nmea_to_csv.py'), *map(str, args)],
            text=True, capture_output=True)

    def nmea_file(self):
        path = self.root / 'solution.nmea'
        # Positions in two different tangent frames, with identical full ESA attitude.
        path.write_text(
            '$GPGGA,180940.000,0000.000000,N,00000.000000,E,4,12,1,100,M,20,M,0,0\n'
            '$GPESA,180940.000,0,0,0,90,90,90\n'
            '$GPGGA,180941.000,0000.000000,N,09000.000000,E,4,12,1,100,M,20,M,0,0\n'
            '$GPESA,180941.000,0,0,0,90,90,90\n')
        return path

    def test_full_attitude_uses_one_fixed_enu_basis(self):
        output = self.root / 'est.csv'
        result = self.run_converter('--nmea', self.nmea_file(), '--out', output,
                                    '--lever-arm=1,2,3', '--enu-origin-lla=0,0,0')
        self.assertEqual(result.returncode, 0, result.stderr)
        with output.open() as source:
            rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 2)
        for row, lon in zip(rows, (0, 90)):
            position = GICI.lla2ecef(*[float(row[key]) for key in ('lat_deg', 'lon_deg', 'h_ell')])
            # Rx(90), Ry(90), Rz(90) maps [1,2,3] to ENU [3,2,-1].
            # ENU at (0,0) maps that to ECEF [-1,3,2], at BOTH body positions.
            expected = GICI.lla2ecef(0, lon, 120) + np.array([-1, 3, 2])
            np.testing.assert_allclose(position, expected, atol=2e-4, rtol=0)

    def test_rtklib_origin_skips_invalid_rows(self):
        origin = self.root / 'rtklib.csv'
        origin.write_text(
            'utc_sec,ecef_x,ecef_y,ecef_z,q\n'
            '65377,6378137,0,0,0\n'
            '65378,nan,0,0,4\n'
            '65379,0,0,0,4\n'
            '65380,6378137,0,0,2\n'
            '65381,0,6378137,0,4\n')
        lla, tod = GICI.first_rtklib_origin(origin)
        np.testing.assert_allclose(lla, [0, 0, 0], atol=1e-8)
        self.assertEqual(tod, 65380)
        output = self.root / 'est.csv'
        result = self.run_converter('--nmea', self.nmea_file(), '--out', output,
                                    '--lever-arm=1,2,3', '--enu-origin-rtklib', origin)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('approximate RTKLIB origin', result.stdout)
        self.assertIn('utc_sec=65380.000', result.stdout)
        with output.open() as source:
            row = next(csv.DictReader(source))
        position = GICI.lla2ecef(*[float(row[k]) for k in ('lat_deg', 'lon_deg', 'h_ell')])
        np.testing.assert_allclose(position, [6378256, 3, 2], atol=2e-4, rtol=0)

    def test_lever_requires_an_explicit_origin(self):
        output = self.root / 'est.csv'
        result = self.run_converter('--nmea', self.nmea_file(), '--out', output,
                                    '--lever-arm=1,2,3')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--lever-arm requires', result.stderr)
        self.assertFalse(output.exists())

    def test_gnss_only_conversion_needs_no_origin(self):
        output = self.root / 'est.csv'
        result = self.run_converter('--nmea', self.nmea_file(), '--out', output)
        self.assertEqual(result.returncode, 0, result.stderr)
        with output.open() as source:
            rows = list(csv.DictReader(source))
        self.assertEqual([float(row['lon_deg']) for row in rows], [0, 90])
        self.assertEqual([float(row['h_ell']) for row in rows], [120, 120])

    def test_default_rinex_cut_matches_utc_start(self):
        source, output = self.root / 'source.obs', self.root / 'cut.obs'
        source.write_text(
            'synthetic header                                             END OF HEADER\n'
            '> 2019 05 09 18 09 57.9990000  0  1\nG01 before\n'
            '> 2019 05 09 18 09 58.0000000  0  1\nG01 at-cut\n'
            '> 2019 05 09 18 09 59.0000000  0  1\nG01 after\n')
        result = subprocess.run(
            [sys.executable, str(LOADERS / 'cut_rinex.py'), '--in', str(source), '--out', str(output)],
            text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('kept 2 epochs, dropped 1', result.stdout)
        self.assertNotIn('G01 before', output.read_text())
        self.assertIn('G01 at-cut', output.read_text())

    def test_rinex_end_is_inclusive_without_rounding_later_epochs_down(self):
        source, output = self.root/'source.obs', self.root/'cut.obs'
        source.write_text(
            'synthetic header                                             END OF HEADER\n'
            '> 2019 05 09 19 17 16.9990000  0  1\nG01 before-end\n'
            '> 2019 05 09 19 17 17.0000000  0  1\nG01 at-end\n'
            '> 2019 05 09 19 17 17.0010000  0  1\nG01 after-end\n')
        subprocess.run([sys.executable, str(LOADERS/'cut_rinex.py'), '--in', str(source),
                        '--out', str(output), '--end-gpst', '2019-05-09 19:17:17'],
                       check=True, capture_output=True, text=True)
        self.assertIn('G01 before-end', output.read_text())
        self.assertIn('G01 at-end', output.read_text())
        self.assertNotIn('G01 after-end', output.read_text())


if __name__ == '__main__':
    unittest.main()
