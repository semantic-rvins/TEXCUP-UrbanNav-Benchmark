#!/usr/bin/env python3
"""Remove direct ground-truth records from saved SeA-RVINS archives.

Original archives are restricted-loaded: estimator classes become inert data
holders, and only the importer's allowlisted NumPy constructors run. Output
chunks contain plain Python mappings, lists, scalars and bytes. Estimated
antenna positions and signed estimated_error_enu_m vectors are retained exactly.
Those two retained fields together permit reconstruction of ground truth; this
utility removes direct GT records rather than claiming to prevent reconstruction.

Requires NumPy and 7z/7zz. Source archives are never overwritten. Write to a
separate output path, inspect the report, then verify the normal import/scoring
workflow with privately obtained ground truth before replacing published files.
"""

import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import io
import json
import os
from pathlib import Path
import pickle
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import import_results as importer


FORMAT = 'sea-rvins-sanitized-records'
SCHEMA_VERSION = 1
OMIT = object()
is_ground_truth_key = importer.is_ground_truth_key


class SafeHolder:
    """Retain constructor arguments and opaque state without estimator code."""

    def __new__(cls, *args, **kwargs):
        result = object.__new__(cls)
        result.__dict__['_constructor_args'] = args
        result.__dict__['_constructor_kwargs'] = kwargs
        return result

    def __init__(self, *args, **kwargs):
        pass

    def __setstate__(self, state):
        if isinstance(state, dict):
            self.__dict__.update(state)
        elif isinstance(state, tuple) and all(part is None or isinstance(part, dict)
                                             for part in state):
            for part in state:
                if isinstance(part, dict):
                    self.__dict__.update(part)
        else:
            self.__dict__['_opaque_state'] = state


HOLDERS = {key: type(key[1], (SafeHolder,), {'pickle_type': '.'.join(key)})
           for key in importer.CUSTOM_GLOBALS}


class SanitizingUnpickler(importer.RestrictedUnpickler):
    def find_class(self, module, name):
        key = (module, name)
        if key in HOLDERS:
            return HOLDERS[key]
        return super().find_class(module, name)


class PlainUnpickler(pickle.Unpickler):
    """Output artifacts must require no globals or class constructors."""

    def find_class(self, module, name):
        raise pickle.UnpicklingError(f'Sanitized chunk contains a global: {module}.{name}')


def plain_value(value, audit, memo=None, active=None):
    """Convert allowlisted data to plain containers; recursively remove GT."""
    memo = {} if memo is None else memo
    active = set() if active is None else active
    if isinstance(value, SafeHolder) and 'groundtruth' in value.pickle_type.lower():
        audit['removed_gt_objects'] += 1
        return OMIT
    if isinstance(value, np.generic):
        return plain_value(value.item(), audit, memo, active)
    if value is None or isinstance(value, (bool, int, float, str, bytes)):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, datetime):
        return value.isoformat()
    if isinstance(value, timedelta):
        return {'__timedelta_seconds__': value.total_seconds()}
    if isinstance(value, timezone):
        return {'__timezone__': str(value)}
    identity = id(value)
    if identity in active:
        raise ValueError('Cyclic archive data cannot be sanitized without losing its meaning')
    if identity in memo:
        original, normalized = memo[identity]
        if original is not value:
            raise ValueError('Archive normalization encountered an object identity collision')
        return normalized
    active.add(identity)
    try:
        if isinstance(value, np.ndarray):
            result = plain_value(value.tolist(), audit, memo, active)
        elif isinstance(value, SafeHolder):
            attributes = dict(value.__dict__)
            if not attributes.get('_constructor_args'):
                attributes.pop('_constructor_args', None)
            if not attributes.get('_constructor_kwargs'):
                attributes.pop('_constructor_kwargs', None)
            result = plain_value(attributes, audit, memo, active)
            result['__type__'] = value.pickle_type
        elif isinstance(value, dict):
            entries = []
            for key, child in value.items():
                if is_ground_truth_key(key):
                    audit['removed_gt_fields'] += 1
                    # Count concrete GT objects even when their containing field
                    # is removed before recursive traversal.
                    if isinstance(child, SafeHolder) and 'groundtruth' in child.pickle_type.lower():
                        audit['removed_gt_objects'] += 1
                    continue
                normalized_key = plain_value(key, audit, memo, active)
                normalized_value = plain_value(child, audit, memo, active)
                if normalized_key is not OMIT and normalized_value is not OMIT:
                    entries.append((normalized_key, normalized_value))
            if all(isinstance(key, (str, bytes, int, float, bool, type(None)))
                   for key, _ in entries):
                result = dict(entries)
                if len(result) != len(entries):
                    raise ValueError('Sanitization would merge diagnostic mapping keys')
            else:
                # Preserve structured diagnostic keys (satellite IDs, enums)
                # without converting them to ambiguous strings.
                result = {'__mapping_entries__': [[key, child] for key, child in entries]}
        elif isinstance(value, (list, tuple, set, frozenset)):
            result = []
            for child in value:
                normalized = plain_value(child, audit, memo, active)
                if normalized is not OMIT:
                    result.append(normalized)
            if isinstance(value, (set, frozenset)):
                result.sort(key=lambda child: pickle.dumps(child, protocol=4))
        else:
            raise ValueError(f'Unsupported archive value: {type(value).__module__}.{type(value).__name__}')
        # Keep each source alive, including temporary attribute dictionaries and
        # ndarray.tolist() containers. Otherwise Python may reuse a temporary's
        # id and incorrectly return an earlier, unrelated normalized value.
        memo[identity] = (value, result)
        return result
    finally:
        active.remove(identity)


def assert_no_ground_truth(value, seen=None):
    seen = set() if seen is None else seen
    if isinstance(value, (dict, list, tuple)):
        if id(value) in seen:
            return
        seen.add(id(value))
    if isinstance(value, dict):
        for key, child in value.items():
            if is_ground_truth_key(key):
                raise ValueError(f'Direct ground-truth key remains: {key}')
            if key == '__type__' and 'groundtruth' in str(child).lower():
                raise ValueError('A ground-truth object remains in sanitized data')
            assert_no_ground_truth(key, seen)
            assert_no_ground_truth(child, seen)
    elif isinstance(value, (list, tuple)):
        for child in value:
            assert_no_ground_truth(child, seen)
    elif value is not None and not isinstance(value, (bool, int, float, str, bytes)):
        raise ValueError(f'Nonplain value remains: {type(value).__name__}')


def field(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def scientific_values(records):
    """Encode exact timing, selection, estimates and signed errors for comparison."""
    signature = []
    counts = Counter(records=len(records))
    for record in records:
        timestamp = field(record, 'timestamp_utc')
        timestamp = timestamp.isoformat() if isinstance(timestamp, datetime) else timestamp
        gps = field(field(record, 'epoch'), 'gps_timestamp')
        gnss = bool(field(record, 'is_gnss_epoch', False))
        position = field(field(record, 'estimated'), 'antenna_pos_enu_m')
        error = field(record, 'estimated_error_enu_m')
        position = None if position is None else np.asarray(position, dtype=float).tolist()
        error = None if error is None else np.asarray(error, dtype=float).tolist()
        signature.append((timestamp, None if gps is None else float(gps), gnss, position, error))
        if gnss:
            counts['gnss_records'] += 1
            finite_position = position is not None and np.shape(position) == (3,) and np.isfinite(position).all()
            finite_error = error is not None and np.shape(error) == (3,) and np.isfinite(error).all()
            if finite_position:
                counts['finite_gnss_antenna_positions'] += 1
                if not finite_error:
                    raise ValueError('A finite GNSS antenna estimate lacks its finite signed estimated_error_enu_m')
            if finite_error:
                counts['finite_signed_gnss_errors'] += 1
    def semantic_value(value):
        if isinstance(value, float):
            # float.hex retains exact finite values and signed zero. NaN remains
            # an unavailable value; its nonsemantic payload bits are ignored.
            return {'float_hex': value.hex()}
        if isinstance(value, (tuple, list)):
            return [semantic_value(child) for child in value]
        return value
    # Explicit value encoding ignores Python object-sharing differences while
    # retaining every scientific value exactly, including signed zero.
    content = json.dumps(semantic_value(signature), separators=(',', ':'),
                         ensure_ascii=True, allow_nan=False).encode('ascii')
    return content, dict(counts)


def sanitize_chunk(source):
    source = Path(source)
    raw = source.read_bytes()
    stream = io.BytesIO(raw)
    payload = SanitizingUnpickler(stream).load()
    if stream.read(1):
        raise ValueError(f'Trailing pickle payload in {source.name}')
    if not isinstance(payload, dict) or not isinstance(payload.get('records'), list):
        raise ValueError(f'Unsupported chunk payload: {source.name}')
    if payload.get('count') != len(payload['records']):
        raise ValueError(f'Chunk count mismatch: {source.name}')
    before, counts = scientific_values(payload['records'])
    audit = Counter()
    sanitized = plain_value(payload, audit)
    sanitized.update(format=FORMAT, schema_version=SCHEMA_VERSION, count=len(sanitized['records']))
    assert_no_ground_truth(sanitized)
    after, after_counts = scientific_values(sanitized['records'])
    if before != after or counts != after_counts:
        raise ValueError(f'Retained scientific fields changed: {source.name}')
    content = pickle.dumps(sanitized, protocol=4)
    loaded = PlainUnpickler(io.BytesIO(content)).load()
    assert_no_ground_truth(loaded)
    report = {'file': source.name, 'source_bytes': len(raw), 'sanitized_bytes': len(content),
              'retained_scientific_fields_unchanged': True, 'counts': counts,
              'removals': dict(audit)}
    return content, report


def same_bytes(first, second):
    if first.stat().st_size != second.stat().st_size:
        return False
    with first.open('rb') as left, second.open('rb') as right:
        while block := left.read(1024 * 1024):
            if block != right.read(len(block)):
                return False
        return not right.read(1)


def sanitize_archive(source, output, report_path=None, executable=None):
    source, output = Path(source).resolve(), Path(output).expanduser()
    if output.exists() or output.is_symlink() or source == output.resolve():
        raise ValueError('Choose a new output archive path; source and existing outputs are preserved')
    output = output.resolve()
    if report_path is not None:
        report_path = Path(report_path).expanduser()
        if report_path.exists() or report_path.is_symlink() or report_path.resolve() in (source, output):
            raise ValueError('Choose a new sanitization report path')
        report_path = report_path.resolve()
    executable = executable or shutil.which('7z') or shutil.which('7zz')
    if executable is None:
        raise RuntimeError('7z or 7zz is required to read/write the sanitized .7z archive')
    output.parent.mkdir(parents=True, exist_ok=True)
    if report_path is not None:
        report_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='sea-rvins-sanitize-') as temporary:
        root = Path(temporary)
        source_snapshot = root / 'source.7z'
        shutil.copyfile(source, source_snapshot)
        listing = subprocess.run([executable, 'l', '-slt', '-ba', str(source_snapshot)],
                                 check=True, capture_output=True, text=True).stdout
        members = importer.archive_members(listing)
        extracted, clean = root / 'original', root / 'sanitized'
        extracted.mkdir()
        clean.mkdir()
        subprocess.run([executable, 'x', '-y', '-bd', f'-o{extracted}', str(source_snapshot)],
                       check=True, capture_output=True, text=True)
        chunks = []
        for name, directory in sorted(members):
            if directory:
                continue
            path = extracted / name
            if path.is_symlink() or not path.is_file():
                raise ValueError('Archive member is not a regular file')
            content, chunk = sanitize_chunk(path)
            destination = clean / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(content)
            # Fixed metadata and single-thread compression make identical inputs
            # repeatable; scientific source save times remain in each payload.
            os.utime(destination, (0, 0))
            chunk['file'] = name
            chunks.append(chunk)
        staged = root / 'sanitized.7z'
        filenames = [chunk['file'] for chunk in chunks]
        subprocess.run([executable, 'a', '-t7z', '-mx=7', '-mmt=1', '-mtc=off',
                        '-mta=off', '-mtm=off', '-bd', str(staged), *filenames],
                       cwd=clean, check=True, capture_output=True, text=True)
        if not same_bytes(source, source_snapshot):
            raise ValueError('Source archive changed during sanitization')
        total_counts, removals = Counter(), Counter()
        for chunk in chunks:
            total_counts.update(chunk['counts'])
            removals.update(chunk['removals'])
        report = {
            'schema_version': SCHEMA_VERSION, 'format': FORMAT,
            'source_archive': source.name, 'sanitized_archive': output.name,
            'chunks': chunks, 'counts': dict(total_counts), 'removals': dict(removals),
            'retained': ['record timing', 'GNSS epoch selection', 'estimated antenna positions',
                         'signed estimated_error_enu_m', 'non-GT diagnostic fields and opaque states'],
            'diagnostic_encoding': 'Inert types carry __type__; constructor args and opaque state are retained. '
                                   'Structured mapping keys use __mapping_entries__. Arrays become lists.',
            'ground_truth_removed': True,
            'ground_truth_reconstruction_possible': True,
            'reconstruction_note': 'Retained antenna positions minus signed errors give GT positions in ENU. '
                                   'Direct GT removal does not prevent reconstruction.',
            'validation': 'Restricted original loading; plain-only output loading; recursive GT-field audit; '
                          'exact equality of retained scientific fields and counts.',
        }
        # Exclusive opens prevent replacement if another process creates a
        # destination after the initial existence checks.
        with output.open('xb') as destination, staged.open('rb') as original:
            shutil.copyfileobj(original, destination)
        if not same_bytes(output, staged):
            raise ValueError('Output archive differs from the staged archive')
        if report_path is not None:
            with report_path.open('x', encoding='utf8') as destination:
                destination.write(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--report', type=Path, help='Optional report path; no sidecar is written by default')
    parser.add_argument('--sevenzip', help='Path to 7z/7zz when it is not on PATH')
    args = parser.parse_args()
    report = sanitize_archive(args.archive, args.out, args.report, args.sevenzip)
    print(json.dumps({key: value for key, value in report.items() if key != 'chunks'}, indent=2))


if __name__ == '__main__':
    main()
