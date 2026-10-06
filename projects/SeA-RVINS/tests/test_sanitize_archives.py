"""Sanitation removes direct GT while retaining estimates and signed errors."""

from contextlib import contextmanager
from datetime import datetime
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

SCRIPT = Path(__file__).resolve().parents[1] / 'loaders/sanitize_archives.py'
spec = importlib.util.spec_from_file_location('sea_sanitize_archives', SCRIPT)
sanitizer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = sanitizer
spec.loader.exec_module(sanitizer)


@contextmanager
def fixture_classes():
    modules, classes = {}, {}
    for module, name in sanitizer.importer.CUSTOM_GLOBALS:
        for count in range(1, len(module.split('.')) + 1):
            parent = '.'.join(module.split('.')[:count])
            modules.setdefault(parent, types.ModuleType(parent))
        cls = type(name, (), {'__module__': module})
        setattr(modules[module], name, cls)
        classes[name] = cls
    with patch.dict(sys.modules, modules):
        yield classes


class SanitizerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.context = fixture_classes()
        self.classes = self.context.__enter__()
        self.addCleanup(self.context.__exit__, None, None, None)

    def instance(self, name, **fields):
        value = self.classes[name]()
        value.__dict__.update(fields)
        return value

    def payload(self):
        gt = self.instance('GroundTruthSingleEpoch', pos_ecef_m=np.array([1., 2., 3.]))
        record = self.instance('EpochLogRecord',
            epoch=self.instance('GpsTime', gps_timestamp=1241460598.),
            timestamp_utc=datetime(2019, 5, 9, 18, 9, 40), is_gnss_epoch=True,
            estimated=self.instance('PoseStateLog', antenna_pos_enu_m=np.array([1.25, 2.5, -3.])),
            estimated_error_enu_m=np.array([-.3, .4, -2.]), ground_truth=gt,
            diagnostics={'nested': {'gt_pose': [100., 200., 300.], 'gt_pos': [1., 2., 3.]},
                         'other_name': gt, 'inlier_count': 11, 'opaque': b'opaque estimator state'})
        return {'count': 1, 'saved_utc': '2026-09-09T00:00:00', 'records': [record]}

    def chunk(self):
        path = self.root / 'records.pkl'
        path.write_bytes(pickle.dumps(self.payload(), protocol=5))
        return path

    def test_recursively_removes_gt_and_preserves_signed_vectors_and_timing(self):
        source = self.chunk()
        before = source.read_bytes()
        content, audit = sanitizer.sanitize_chunk(source)
        self.assertEqual(source.read_bytes(), before)
        payload = sanitizer.PlainUnpickler(io.BytesIO(content)).load()
        sanitizer.assert_no_ground_truth(payload)
        self.assertEqual(payload['format'], sanitizer.FORMAT)
        record = payload['records'][0]
        self.assertEqual(record['timestamp_utc'], '2019-05-09T18:09:40')
        self.assertEqual(record['epoch']['gps_timestamp'], 1241460598.)
        self.assertEqual(record['estimated']['antenna_pos_enu_m'], [1.25, 2.5, -3.])
        self.assertEqual(record['estimated_error_enu_m'], [-.3, .4, -2.])
        self.assertEqual(record['diagnostics']['inlier_count'], 11)
        self.assertEqual(record['diagnostics']['opaque'], b'opaque estimator state')
        self.assertNotIn('other_name', record['diagnostics'])
        self.assertEqual(audit['counts']['finite_signed_gnss_errors'], 1)
        self.assertEqual(audit['removals']['removed_gt_fields'], 3)
        self.assertEqual(audit['removals']['removed_gt_objects'], 2)

    def test_custom_constructor_args_opaque_state_and_structured_keys_survive(self):
        cls = self.classes['Constellation']
        def reduce_enum(value):
            return cls, (7,), b'opaque enum state'
        cls.__reduce__ = reduce_enum
        value = cls()
        loaded = sanitizer.SanitizingUnpickler(io.BytesIO(pickle.dumps(value))).load()
        plain = sanitizer.plain_value({loaded: loaded}, sanitizer.Counter())
        entry = plain['__mapping_entries__'][0]
        self.assertEqual(entry[1]['_constructor_args'], [7])
        self.assertEqual(entry[1]['_opaque_state'], b'opaque enum state')
        self.assertEqual(entry[1]['__type__'], 'constants.gnss_constants.Constellation')

    def test_numpy_scalar_subclasses_become_plain_python_values(self):
        plain = sanitizer.plain_value({'float': np.float64(1.25), 'text': np.str_('ok'),
                                       'bytes': np.bytes_(b'ok'), 'int': np.int64(7)},
                                      sanitizer.Counter())
        loaded = sanitizer.PlainUnpickler(io.BytesIO(pickle.dumps(plain))).load()
        self.assertEqual(loaded, {'float': 1.25, 'text': 'ok', 'bytes': b'ok', 'int': 7})
        self.assertIs(type(plain['float']), float)

    def test_successive_temporary_lists_and_holder_dicts_keep_distinct_values(self):
        holder_type = sanitizer.HOLDERS[('utils.logging_helper', 'PoseStateLog')]
        values = []
        for index in range(100):
            holder = holder_type()
            holder.antenna_pos_enu_m = np.array([index, index + .25, -index - .5])
            holder.signed_error = np.array([-index, index + .5, index + 1.])
            values.append(holder)
        plain = sanitizer.plain_value(values, sanitizer.Counter())
        for index, value in enumerate(plain):
            self.assertEqual(value['antenna_pos_enu_m'], [index, index + .25, -index - .5])
            self.assertEqual(value['signed_error'], [-index, index + .5, index + 1.])

    def test_scientific_comparison_depends_on_values_instead_of_python_aliases(self):
        shared_timestamp = '2019-05-09T18:09:40'
        def record(timestamp):
            return {'timestamp_utc': timestamp, 'epoch': {'gps_timestamp': 1241460598.},
                    'is_gnss_epoch': True, 'estimated': {'antenna_pos_enu_m': [1., -0., 3.]},
                    'estimated_error_enu_m': [-1., .5, -3.]}
        shared = [record(shared_timestamp), record(shared_timestamp)]
        independent = [record(('x' + shared_timestamp)[1:]), record(('y' + shared_timestamp)[1:])]
        self.assertEqual(sanitizer.scientific_values(shared),
                         sanitizer.scientific_values(independent))
        independent[1]['estimated']['antenna_pos_enu_m'][1] = 0.
        self.assertNotEqual(sanitizer.scientific_values(shared),
                            sanitizer.scientific_values(independent))

    def test_arbitrary_callable_and_nonplain_output_are_rejected(self):
        class Malicious:
            def __reduce__(self):
                return eval, ('1 + 1',)
        with self.assertRaisesRegex(pickle.UnpicklingError, 'builtins.eval'):
            sanitizer.SanitizingUnpickler(io.BytesIO(pickle.dumps(Malicious()))).load()
        with self.assertRaisesRegex(pickle.UnpicklingError, 'contains a global'):
            sanitizer.PlainUnpickler(io.BytesIO(pickle.dumps(datetime(2019, 5, 9)))).load()

    def test_missing_signed_error_cannot_silently_change_export_contract(self):
        payload = self.payload()
        payload['records'][0].estimated_error_enu_m = None
        path = self.root / 'missing.pkl'
        path.write_bytes(pickle.dumps(payload))
        with self.assertRaisesRegex(ValueError, 'lacks its finite signed'):
            sanitizer.sanitize_chunk(path)

    def test_archive_write_is_repeatable_and_existing_files_are_preserved(self):
        executable = shutil.which('7z') or shutil.which('7zz')
        if executable is None:
            self.skipTest('7z/7zz not installed')
        source = self.chunk()
        archive = self.root / 'original.7z'
        subprocess.run([executable, 'a', str(archive), source.name], cwd=self.root,
                       check=True, capture_output=True)
        source_bytes = archive.read_bytes()
        first, second = self.root / 'first.7z', self.root / 'second.7z'
        first_report = sanitizer.sanitize_archive(archive, first)
        second_report = sanitizer.sanitize_archive(archive, second)
        self.assertEqual(archive.read_bytes(), source_bytes)
        self.assertEqual(first.read_bytes(), second.read_bytes())
        self.assertFalse(first.with_suffix('.sanitization.json').exists())
        self.assertTrue(first_report['ground_truth_removed'])
        self.assertTrue(second_report['ground_truth_reconstruction_possible'])
        with self.assertRaisesRegex(ValueError, 'existing outputs are preserved'):
            sanitizer.sanitize_archive(archive, first)
        self.assertEqual(first_report['counts']['finite_gnss_antenna_positions'], 1)


if __name__ == '__main__':
    unittest.main()
