"""Regression checks for source identity and inclusive observation selection."""

from datetime import datetime, timedelta
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


LOADERS = Path(__file__).resolve().parents[1] / "loaders"


def load(name):
    spec = importlib.util.spec_from_file_location(name, LOADERS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


converter = load("sbf_to_rinex")
validator = load("validate_rinex")


def header(value, label):
    return value.ljust(60) + label + "\n"


def timestamp(value):
    return f"{value.year:6d}{value.month:6d}{value.day:6d}{value.hour:6d}{value.minute:6d}{value.second:13.7f}     GPS"


START = datetime(2019, 5, 9, 18, 9, 58)
PROFILE = {"time": {"scale": "GPST", "start": START.isoformat(),
                    "end_inclusive": (START + timedelta(seconds=1)).isoformat(),
                    "tolerance_seconds": 0.005},
           "interval_seconds": 1, "expected_epochs": 2,
           "signals": {"G": ["1C"]}, "observables": ["C", "L", "D", "S"]}


def observations(times, nonfinite=False):
    output = [header("     3.04           O                   M", "RINEX VERSION / TYPE"),
              header("G    4 C1C L1C D1C S1C", "SYS / # / OBS TYPES"),
              header(timestamp(times[0]), "TIME OF FIRST OBS"),
              header(timestamp(times[-1]), "TIME OF LAST OBS"),
              header("     1.000", "INTERVAL"), header("", "END OF HEADER")]
    for i, epoch in enumerate(times):
        output.append(f"> {epoch:%Y %m %d %H %M} {epoch.second:10.7f}  {i % 2}  1\n")
        values = ["nan" if nonfinite and i == 0 else "22000000.125", "12345.500", "-42.250", "45.000"]
        # Include nonblank LLI/SSI, including unresolved half-cycle bit 2.
        output.append("G01" + "".join(f"{value:>14}{'2' if j == 1 else ' '}{'7' if j == 1 else ' '}"
                                        for j, value in enumerate(values)) + "\n")
    return "".join(output)


class ObservationValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, content):
        path = self.root / "observations.obs"
        path.write_text(content)
        return path

    def test_valid_observations_include_clock_reset_and_phase_flags(self):
        result = validator.validate(self.write(observations([START, START + timedelta(seconds=1)])), PROFILE)
        self.assertTrue(result["valid"], result["errors"])
        self.assertEqual(result["epochs"]["flags"], {"0": 1, "1": 1})
        self.assertEqual(result["lli_field_distribution"]["G:L1C"], {"2": 2})

    def test_missing_and_duplicate_epochs_fail_sampling_validation(self):
        for times in ([START, START + timedelta(seconds=2)], [START, START]):
            with self.subTest(times=times):
                result = validator.validate(self.write(observations(times)), PROFILE)
                self.assertFalse(result["valid"])
                self.assertTrue(any("epoch" in error.lower() for error in result["errors"]))

    def test_nonfinite_measurement_cannot_pass_valid_headers(self):
        result = validator.validate(self.write(observations([START, START + timedelta(seconds=1)], nonfinite=True)), PROFILE)
        self.assertFalse(result["valid"])
        self.assertTrue(any("invalid numeric" in error for error in result["errors"]))

    def test_crop_retains_inclusive_end_and_unchanged_phase_flag_fields(self):
        times = [START - timedelta(seconds=1), START, START + timedelta(seconds=1), START + timedelta(seconds=2)]
        source = self.write(observations(times))
        destination = self.root / "cropped.obs"
        converter.crop_observations(source, destination, PROFILE)
        result = validator.validate(destination, PROFILE)
        self.assertTrue(result["valid"], result["errors"])
        input_records = source.read_text().splitlines()
        output_records = destination.read_text().splitlines()
        self.assertEqual([line for line in output_records if line.startswith((">", "G01"))], input_records[8:12])
        self.assertEqual(result["epochs"]["last"], "2019-05-09T18:09:59.000000")

    def test_input_size_guard_runs_before_converter_or_output_creation(self):
        source = self.root / "unverified.sbf"
        source.write_bytes(b"different source")
        profile = dict(PROFILE, source_file={"bytes": 17})
        profile_path = self.root / "profile.json"
        profile_path.write_text(json.dumps(profile))
        output = self.root / "output"
        result = subprocess.run([sys.executable, str(LOADERS / "sbf_to_rinex.py"), str(source),
                                 "--profile", str(profile_path), "--convbin", str(self.root / "absent-convbin"),
                                 "--output-dir", str(output)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Input size differs from the source recording", result.stderr)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
