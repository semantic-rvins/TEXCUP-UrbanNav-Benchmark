"""Focused import-contract tests; no estimator checkout or GTSAM is required."""

from contextlib import contextmanager
from datetime import timedelta
import importlib.util
import io
from pathlib import Path
import pickle
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'loaders/import_results.py'
spec = importlib.util.spec_from_file_location('sea_import_results', SCRIPT)
importer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = importer
spec.loader.exec_module(importer)


@contextmanager
def fixture_classes():
    modules = {}
    classes = {}
    for module, name in importer.CUSTOM_GLOBALS:
        for count in range(1, len(module.split('.')) + 1):
            parent = '.'.join(module.split('.')[:count])
            modules.setdefault(parent, types.ModuleType(parent))
        cls = type(name, (), {'__module__': module})
        setattr(modules[module], name, cls)
        classes[name] = cls
    with patch.dict(sys.modules, modules):
        yield classes


class ImportContractTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.gt = self.root / 'ground_truth.log'
        self.gt.write_text('2019/05/09 18:09:40 0 30.28666 -97.72742 158.43\n'
                           '2019/05/09 18:09:41 0 30.28666 -97.72742 158.43\n')
        self.context = fixture_classes()
        self.classes = self.context.__enter__()
        self.addCleanup(self.context.__exit__, None, None, None)

    def record(self, utc_sec, error=(0.3, -0.4, 2.0), gnss=True):
        def instance(name, **attributes):
            result = self.classes[name]()
            result.__dict__.update(attributes)
            return result
        utc = importer.UTC_DAY + timedelta(seconds=utc_sec)
        gps = (utc - importer.GPS_EPOCH).total_seconds() + importer.LEAP_SECONDS
        ecef = importer.lla2ecef(30.28666, -97.72742, 158.43)
        enu = importer.base_rotation() @ (ecef - importer.BASE_ECEF_M)
        return instance('EpochLogRecord',
            epoch=instance('GpsTime', gps_timestamp=gps), timestamp_utc=utc.replace(tzinfo=None),
            is_gnss_epoch=gnss,
            estimated=instance('PoseStateLog', antenna_pos_enu_m=enu + error, body_pos_enu_m=enu + 100),
            ground_truth=instance('GroundTruthSingleEpoch', pos_ecef_m=ecef, pos_world_enu_m=enu),
            estimated_error_enu_m=np.array(error),
            vision_log=instance('VisionLog', inlier_count=10, frame_action='full'),
            visibility_scene_snapshot={'service_enabled': False})

    def sanitized_record(self, utc_sec, error=(0.3, -0.4, 2.0), gnss=True):
        legacy = self.record(utc_sec, error=error, gnss=gnss)
        return {
            'epoch': {'gps_timestamp': legacy.epoch.gps_timestamp},
            'timestamp_utc': legacy.timestamp_utc.isoformat(),
            'is_gnss_epoch': legacy.is_gnss_epoch,
            'estimated': {'antenna_pos_enu_m': legacy.estimated.antenna_pos_enu_m.tolist()},
            'estimated_error_enu_m': legacy.estimated_error_enu_m.tolist(),
            'vision_log': {'inlier_count': 10, 'frame_action': 'full'},
            'visibility_scene_snapshot': {'service_enabled': False},
        }

    def chunk(self, records, name='records.pkl', sanitized=False):
        path = self.root / name
        payload = {'records': records, 'count': len(records), 'saved_utc': '2026-09-09T00:00:00'}
        if sanitized:
            payload.update(format='sea-rvins-sanitized-records', schema_version=1)
        path.write_bytes(pickle.dumps(payload, protocol=5))
        return path

    def test_sanitized_records_without_gt_preserve_legacy_estimates_and_diagnostics(self):
        options = [(65380, (.3, -.4, 2.), True), (65380.5, (.3, -.4, 2.), False),
                   (65381, (90000., 0., 0.), True)]
        legacy = [self.record(t, error=e, gnss=g) for t, e, g in options]
        sanitized = [self.sanitized_record(t, error=e, gnss=g) for t, e, g in options]
        old_rows, old_report = importer.convert_pickles([self.chunk(legacy, 'legacy.pkl')], self.gt)
        new_rows, new_report = importer.convert_pickles(
            [self.chunk(sanitized, 'sanitized.pkl', sanitized=True)], self.gt)
        np.testing.assert_array_equal(new_rows, old_rows)
        for key in ('counts', 'vision_frame_actions', 'visibility_service_enabled'):
            self.assertEqual(new_report[key], old_report[key])
        self.assertEqual(new_report['archive_formats'], {'sanitized': 1})
        self.assertEqual(new_report['counts']['gnss_records_with_validated_signed_error'], 2)
        self.assertNotIn('canonical_gt_residual_m', new_report['max_residuals'])

    def test_sanitized_frame_and_error_identity_use_local_gt(self):
        record = self.sanitized_record(65380)
        record['estimated']['antenna_pos_enu_m'][0] += 1.
        with self.assertRaisesRegex(ValueError, 'fixed base ENU frame'):
            importer.convert_pickles([self.chunk([record], sanitized=True)], self.gt)
        self.gt.write_text(self.gt.read_text().replace('158.43', '159.43'))
        with self.assertRaisesRegex(ValueError, 'canonical GT'):
            importer.convert_pickles([self.chunk([self.sanitized_record(65380)], sanitized=True)], self.gt)

    def test_sanitized_finite_estimates_require_finite_signed_error(self):
        for error in (None, [1., 2.], [float('nan'), 0., 0.]):
            with self.subTest(error=error):
                record = self.sanitized_record(65380)
                record['estimated_error_enu_m'] = error
                with self.assertRaisesRegex(ValueError, 'saved signed ENU error'):
                    importer.convert_pickles([self.chunk([record], sanitized=True)], self.gt)

    def test_sanitized_unavailable_estimates_are_not_reconstructed_from_errors(self):
        records = [self.sanitized_record(65380), self.sanitized_record(65381)]
        records[1]['estimated']['antenna_pos_enu_m'] = [float('nan')] * 3
        rows, report = importer.convert_pickles([self.chunk(records, sanitized=True)], self.gt)
        np.testing.assert_array_equal(rows[:, 0], [65380.])
        self.assertEqual(report['counts']['gnss_missing_or_nonfinite_antenna_not_exported'], 1)

    def test_sanitized_content_rejects_direct_gt_even_on_non_gnss_records(self):
        for key in ('ground_truth', 'gt_pose', 'gt_pos', 'GT_ECEF', 'nested_groundtruth_record', 'gt'):
            with self.subTest(key=key):
                records = [self.sanitized_record(65380), self.sanitized_record(65380.5, gnss=False)]
                records[1]['vision_log'][key] = [1., 2., 3.]
                with self.assertRaisesRegex(ValueError, 'direct ground truth'):
                    importer.convert_pickles([self.chunk(records, sanitized=True)], self.gt)

    def test_sanitized_ground_truth_type_tag_is_rejected(self):
        record = self.sanitized_record(65380)
        record['vision_log']['opaque'] = {'__type__': 'imu_utils.imu_data_utils.GroundTruthSingleEpoch'}
        with self.assertRaisesRegex(ValueError, 'direct ground truth'):
            importer.convert_pickles([self.chunk([record], sanitized=True)], self.gt)

    def test_sanitized_rejects_bad_timestamps_and_unknown_schema(self):
        record = self.sanitized_record(65380)
        record['timestamp_utc'] = 'not-a-datetime'
        with self.assertRaisesRegex(ValueError, 'invalid GPS or UTC timestamp'):
            importer.convert_pickles([self.chunk([record], sanitized=True)], self.gt)
        path = self.chunk([self.sanitized_record(65380)], sanitized=True)
        payload = pickle.loads(path.read_bytes())
        payload['schema_version'] = 2
        path.write_bytes(pickle.dumps(payload))
        with self.assertRaisesRegex(ValueError, 'schema version'):
            importer.convert_pickles([path], self.gt)

    def test_local_gt_must_be_finite(self):
        self.gt.write_text(self.gt.read_text().replace('158.43', 'nan'))
        with self.assertRaisesRegex(ValueError, 'finite epochs and coordinates'):
            importer.convert_pickles(
                [self.chunk([self.sanitized_record(65380)], sanitized=True)], self.gt)

    def test_numpy_pickles_and_both_core_module_spellings_are_supported(self):
        for protocol in (4, 5):
            data = {'array': np.array([1., 2., 3.]), 'scalar': np.float64(4.)}
            restored = importer.RestrictedUnpickler(io.BytesIO(pickle.dumps(data, protocol=protocol))).load()
            np.testing.assert_array_equal(restored['array'], data['array'])
            self.assertEqual(restored['scalar'], data['scalar'])
        for prefix in ('numpy.core', 'numpy._core'):
            for module, name in (('multiarray', '_reconstruct'), ('multiarray', 'scalar'),
                                 ('numeric', '_frombuffer')):
                encoded = f'c{prefix}.{module}\n{name}\n.'.encode('ascii')
                resolved = importer.RestrictedUnpickler(io.BytesIO(encoded)).load()
                self.assertIs(resolved, importer._SAFE_GLOBALS[(prefix + '.' + module, name)])

    def test_antenna_frame_time_selection_and_finite_outlier(self):
        records = [self.record(65380), self.record(65380.5, gnss=False),
                   self.record(65381, error=(90000., 0., 0.))]
        rows, report = importer.convert_pickles([self.chunk(records)], self.gt)
        np.testing.assert_array_equal(rows[:, 0], [65380., 65381.])
        expected = importer.lla2ecef(30.28666, -97.72742, 158.43)
        np.testing.assert_allclose(rows[0, 1:] - expected,
                                   importer.base_rotation().T @ [.3, -.4, 2.], atol=1e-9)
        self.assertGreater(np.linalg.norm(rows[1, 1:] - expected), 89999.)
        self.assertEqual(report['counts']['exported_gnss_epochs'], 2)
        self.assertEqual(report['counts']['non_gnss_records_not_exported'], 1)
        self.assertEqual(report['counts']['vision_logs_with_positive_inliers'], 3)

    def test_rejects_18_second_time_mismatch(self):
        record = self.record(65380)
        record.timestamp_utc += timedelta(seconds=18)
        with self.assertRaisesRegex(ValueError, 'UTC and GPS'):
            importer.convert_pickles([self.chunk([record])], self.gt)

    def test_rejects_wrong_origin_and_wrong_canonical_gt(self):
        record = self.record(65380)
        record.ground_truth.pos_world_enu_m[0] += 1.
        with self.assertRaisesRegex(ValueError, 'fixed base ENU'):
            importer.convert_pickles([self.chunk([record])], self.gt)
        self.gt.write_text(self.gt.read_text().replace('158.43', '159.43'))
        with self.assertRaisesRegex(ValueError, 'differs from canonical GT'):
            importer.convert_pickles([self.chunk([self.record(65380)])], self.gt)

    def test_rejects_duplicates_instead_of_choosing_a_pose(self):
        with self.assertRaisesRegex(ValueError, 'Duplicate record GPS'):
            importer.convert_pickles([self.chunk([self.record(65380), self.record(65380)])], self.gt)

    def test_nonfinite_antenna_is_counted_unavailable(self):
        records = [self.record(65380), self.record(65381)]
        records[1].estimated.antenna_pos_enu_m[:] = np.nan
        rows, report = importer.convert_pickles([self.chunk(records)], self.gt)
        self.assertEqual(len(rows), 1)
        self.assertEqual(report['counts']['gnss_missing_or_nonfinite_antenna_not_exported'], 1)

    def test_pickle_cannot_resolve_arbitrary_callable(self):
        class Malicious:
            def __reduce__(self):
                return eval, ('1 + 1',)
        with self.assertRaisesRegex(pickle.UnpicklingError, 'builtins.eval'):
            importer.RestrictedUnpickler(io.BytesIO(pickle.dumps(Malicious()))).load()

    def test_archive_rejects_path_traversal_and_links(self):
        with self.assertRaisesRegex(ValueError, 'Unsafe archive path'):
            importer.archive_members('Path = ../records.pkl\nSize = 1\nAttributes = A_ -rw-r--r--\n')
        with self.assertRaisesRegex(ValueError, 'links are unsupported'):
            importer.archive_members('Path = records.pkl\nSize = 1\nAttributes = A_ lrwxrwxrwx\n')

    def test_real_7z_import_writes_reproducible_manifest(self):
        import json
        chunk = self.chunk([self.sanitized_record(65380), self.sanitized_record(65381)], sanitized=True)
        archive = self.root / 'fixture.7z'
        executable = shutil.which('7z') or shutil.which('7zz')
        if executable is None:
            self.skipTest('7z/7zz is not installed')
        subprocess.run([executable, 'a', str(archive), chunk.name], cwd=self.root,
                       check=True, capture_output=True)
        output = self.root / 'converted'
        report = importer.import_archive(archive, 'latent', output, self.gt)
        manifest = json.loads((output / 'run_manifest.json').read_text())
        self.assertTrue(report['passed'])
        self.assertEqual(report['archive_formats'], {'sanitized': 1})
        self.assertEqual(manifest['display_name'], 'SeA-RVINS (latent)')
        self.assertEqual(manifest['archive_bytes'], archive.stat().st_size)
        self.assertTrue((output / 'est.csv').is_file())
        self.assertTrue(manifest['source_contract_limitations'])
        self.assertFalse((output / 'status.json').exists())

    def test_unfetched_lfs_pointer_has_actionable_error(self):
        archive = self.root / 'pointer.7z'
        archive.write_text('version https://git-lfs.github.com/spec/v1\nsize 100\n')
        with self.assertRaisesRegex(ValueError, 'git lfs pull'):
            importer.import_archive(archive, 'scalar', self.root / 'converted', self.gt)


if __name__ == '__main__':
    unittest.main()
