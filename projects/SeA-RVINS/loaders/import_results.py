#!/usr/bin/env python3
"""Import archived SeA-RVINS antenna estimates without loading the estimator.

Requires NumPy and a 7z/7zz executable. Only explicitly allowed NumPy constructors
are executable during unpickling; estimator classes become inert data holders.
The fixed base ENU transform is validated against local ground truth and saved
signed errors (or legacy saved coordinate pairs), never estimated by aligning
the trajectory to ground truth.
"""

from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
from collections.abc import Mapping
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path, PurePosixPath
import pickle
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'common'))
from evaluate import default_ground_truth, lla2ecef, load_gt

BASE_ECEF_M = np.array([-742080.4125, -5462031.7412, 3198339.6909])
UTC_DAY = datetime(2019, 5, 9, tzinfo=timezone.utc)
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
LEAP_SECONDS = 18
COORDINATE_TOLERANCE_M = 1e-5
GT_FIELD_PREFIXES = ('gtpos', 'gtpose', 'gtecef', 'gtenu', 'gtlla', 'gtlat', 'gtlon',
                     'gtalt', 'gtheight', 'gtattitude', 'gtvelocity', 'gttrajectory',
                     'gtrecord', 'gtsample', 'gtstate')
CUSTOM_GLOBALS = {
    ('constants.gnss_constants', 'Constellation'),
    ('fgo_solver.utils', 'EpochStateLogInfo'),
    ('gnss_utils.gnss_dataclass', 'SatelliteId'),
    ('gnss_utils.gnss_dataclass', 'SignalChannelId'),
    ('gnss_utils.gnss_dataclass', 'SignalType'),
    ('gnss_utils.time_utils', 'GpsTime'),
    ('gtsam.gtsam', 'Pose3'),
    ('gtsam.gtsam.imuBias', 'ConstantBias'),
    ('imu_utils.imu_data_utils', 'GroundTruthSingleEpoch'),
    *[('utils.logging_helper', name) for name in (
        'AmbiguityFixLog', 'DdResidualLog', 'EpochLogRecord', 'IarResultStatus',
        'PoseStateLog', 'VisibilityChannelPolicyLog', 'VisionLog')],
}


class InertRecord:
    """Preserve pickle attributes without calling any estimator constructor."""

    def __new__(cls, *args, **kwargs):
        return object.__new__(cls)

    def __init__(self, *args, **kwargs):
        pass

    def __setstate__(self, state):
        if isinstance(state, dict):
            self.__dict__.update(state)
        elif isinstance(state, tuple):
            for part in state:
                if isinstance(part, dict):
                    self.__dict__.update(part)
        # Serialized GTSAM internals are deliberately not interpreted.


_INERT_CLASSES = {key: type(key[1], (InertRecord,), {}) for key in CUSTOM_GLOBALS}
_SAFE_GLOBALS = {
    ('numpy', 'ndarray'): np.ndarray,
    ('numpy', 'dtype'): np.dtype,
    ('datetime', 'datetime'): datetime,
    ('datetime', 'timezone'): timezone,
    ('datetime', 'timedelta'): timedelta,
    ('collections', 'OrderedDict'): OrderedDict,
}
# NumPy 2 renamed its private core package. Resolve the installed constructors
# once and accept both pickle spellings so NumPy 1 and 2 archives remain usable.
_NUMPY_CORE = getattr(np, '_core', None)
if _NUMPY_CORE is None:
    _NUMPY_CORE = np.core
for _prefix in ('numpy.core', 'numpy._core'):
    _SAFE_GLOBALS[(_prefix + '.multiarray', '_reconstruct')] = _NUMPY_CORE.multiarray._reconstruct
    _SAFE_GLOBALS[(_prefix + '.multiarray', 'scalar')] = _NUMPY_CORE.multiarray.scalar
    _SAFE_GLOBALS[(_prefix + '.numeric', '_frombuffer')] = _NUMPY_CORE.numeric._frombuffer


class RestrictedUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        key = (module, name)
        if key in _SAFE_GLOBALS:
            return _SAFE_GLOBALS[key]
        if key in _INERT_CLASSES:
            return _INERT_CLASSES[key]
        raise pickle.UnpicklingError(f'Unsupported pickle global: {module}.{name}')


def base_rotation():
    """WGS84 ECEF-to-ENU rotation at the documented TEX-CUP base station."""
    x, y, z = BASE_ECEF_M
    p = np.hypot(x, y)
    lon = np.arctan2(y, x)
    e2 = 6.69437999014e-3
    lat = np.arctan2(z, p * (1 - e2))
    for _ in range(10):
        n = 6378137.0 / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        lat = np.arctan2(z + e2 * n * np.sin(lat), p)
    return np.array([
        [-np.sin(lon), np.cos(lon), 0],
        [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
        [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)],
    ])


def archive_members(listing):
    """Accept only simple directories and regular pickle files before extraction."""
    members = []
    total_bytes = 0
    for block in listing.strip().split('\n\n'):
        fields = dict(line.split(' = ', 1) for line in block.splitlines() if ' = ' in line)
        if 'Path' not in fields:
            continue
        name = fields['Path']
        parts = PurePosixPath(name).parts
        if (not re.fullmatch(r'[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*', name)
                or any(part in ('.', '..') for part in parts)):
            raise ValueError(f'Unsafe archive path: {name!r}')
        attrs = fields.get('Attributes', '')
        if 'Symbolic Link' in fields or 'Hard Link' in fields or re.search(r'(^| )l[rwx-]', attrs):
            raise ValueError(f'Archive links are unsupported: {name}')
        if fields.get('Encrypted', '-') != '-':
            raise ValueError('Encrypted archives are unsupported')
        is_directory = attrs.startswith('D') or fields.get('Folder') == '+'
        if not is_directory and not name.endswith('.pkl'):
            raise ValueError(f'Unexpected archive file: {name}')
        size = int(fields.get('Size', '0'))
        if size < 0:
            raise ValueError('Negative archive member size')
        total_bytes += size
        members.append((name, is_directory))
    names = [name for name, _ in members]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate archive member paths')
    if not members or not any(not directory for _, directory in members):
        raise ValueError('Archive contains no pickle files')
    if len(members) > 10000 or total_bytes > 2_000_000_000:
        raise ValueError('Archive exceeds this importer\'s extraction limits')
    return members


def field(record, name, default=None):
    """Read sanitized mappings and inert legacy records with the same contract."""
    return record.get(name, default) if isinstance(record, Mapping) else getattr(record, name, default)


def is_ground_truth_key(key):
    """Recognize direct GT fields, including conventional nested aliases."""
    if not isinstance(key, str):
        return False
    compact = re.sub(r'[^a-z0-9]', '', key.lower())
    return ('groundtruth' in compact or compact == 'gt'
            or compact.startswith(GT_FIELD_PREFIXES))


def validate_sanitized_content(value):
    """Reject direct GT fields anywhere in a sanitized chunk, including diagnostics."""
    if isinstance(value, Mapping):
        for name, child in value.items():
            if (is_ground_truth_key(name)
                    or name == '__type__' and 'groundtruth' in str(child).lower()):
                raise ValueError('Sanitized records must not contain direct ground truth')
            validate_sanitized_content(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            validate_sanitized_content(child)
    elif isinstance(value, InertRecord):
        raise ValueError('Sanitized records must contain plain data, not estimator objects')


def record_time(record):
    try:
        gps = float(field(field(record, 'epoch'), 'gps_timestamp'))
    except (TypeError, ValueError) as exc:
        raise ValueError('Record has invalid GPS or UTC timestamp') from exc
    utc = field(record, 'timestamp_utc')
    if isinstance(utc, str):
        try:
            utc = datetime.fromisoformat(utc.replace('Z', '+00:00'))
        except ValueError as exc:
            raise ValueError('Record has invalid GPS or UTC timestamp') from exc
    if not np.isfinite(gps) or not isinstance(utc, datetime):
        raise ValueError('Record has invalid GPS or UTC timestamp')
    utc = utc.replace(tzinfo=timezone.utc) if utc.tzinfo is None else utc.astimezone(timezone.utc)
    gps_utc = GPS_EPOCH + timedelta(seconds=gps - LEAP_SECONDS)
    if abs((utc - gps_utc).total_seconds()) > 2e-6:
        raise ValueError('Archived UTC and GPS timestamps disagree')
    if utc.date() != UTC_DAY.date():
        raise ValueError('Record is outside the TEX-CUP recording date')
    return gps, (utc - UTC_DAY).total_seconds()


def vector(value, label):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(f'Invalid {label}: expected a finite three-vector')
    return array


def convert_pickles(files, gt_path):
    """Validate saved frames/times against local GT and export native GNSS poses."""
    rotation = base_rotation()
    gt_times, gt_lla = load_gt(gt_path)
    if (gt_lla.shape != (len(gt_times), 3) or not len(gt_times)
            or not np.isfinite(gt_times).all() or not np.isfinite(gt_lla).all()):
        raise ValueError('Canonical ground truth must contain finite epochs and coordinates')
    gt_ecef = lla2ecef(gt_lla[:, 0], gt_lla[:, 1], gt_lla[:, 2])
    canonical_gt = dict(zip(gt_times, gt_ecef))
    if len(canonical_gt) != len(gt_times):
        raise ValueError('Canonical ground truth contains duplicate epochs')
    counts = Counter()
    maxima = Counter()
    vision_actions = Counter()
    visibility_enabled = Counter()
    seen_times = set()
    gnss_window_times = set()
    rows = []
    chunks = []
    archive_formats = Counter()
    for path in sorted(files):
        with path.open('rb') as stream:
            payload = RestrictedUnpickler(stream).load()
        if not isinstance(payload, dict) or not isinstance(payload.get('records'), list):
            raise ValueError(f'Unsupported chunk payload: {path.name}')
        if payload.get('count') != len(payload['records']):
            raise ValueError(f'Chunk record count mismatch: {path.name}')
        sanitized = payload.get('format') == 'sea-rvins-sanitized-records'
        if 'format' in payload and not sanitized:
            raise ValueError(f'Unsupported chunk format: {path.name}')
        if sanitized and payload.get('schema_version') != 1:
            raise ValueError(f'Unsupported sanitized schema version: {path.name}')
        if sanitized:
            validate_sanitized_content(payload)
        archive_formats['sanitized' if sanitized else 'legacy'] += 1
        chunks.append({'filename': path.name,
                       'records': len(payload['records']), 'saved_utc': payload.get('saved_utc')})
        for record in payload['records']:
            counts['records'] += 1
            gps, utc_sec = record_time(record)
            if gps in seen_times:
                raise ValueError(f'Duplicate record GPS timestamp: {gps}; no last-file-wins policy')
            seen_times.add(gps)
            if sanitized and not isinstance(record, Mapping):
                raise ValueError('Sanitized records must be plain mappings')
            vision = field(record, 'vision_log')
            if vision is not None:
                counts['vision_logs'] += 1
                vision_actions[str(field(vision, 'frame_action'))] += 1
                if field(vision, 'inlier_count', 0) > 0:
                    counts['vision_logs_with_positive_inliers'] += 1
            snapshot = field(record, 'visibility_scene_snapshot')
            if isinstance(snapshot, dict):
                visibility_enabled[str(snapshot.get('service_enabled'))] += 1
            if not field(record, 'is_gnss_epoch', False):
                counts['non_gnss_records_not_exported'] += 1
                continue
            counts['gnss_records'] += 1
            if 65380 <= utc_sec <= 69419:
                counts['gnss_records_in_common_window'] += 1
                gnss_window_times.add(utc_sec)
            if utc_sec not in canonical_gt:
                raise ValueError(f'Archived GNSS epoch has no exact canonical GT epoch: {utc_sec}')
            local_gt_enu = rotation @ (canonical_gt[utc_sec] - BASE_ECEF_M)
            gt = field(record, 'ground_truth')
            if sanitized and gt is not None:
                raise ValueError('Sanitized records must not contain direct ground truth')
            if not sanitized:
                saved_ecef = vector(field(gt, 'pos_ecef_m'), 'saved GT ECEF')
                saved_enu = vector(field(gt, 'pos_world_enu_m'), 'saved GT ENU')
                frame_residual = float(np.linalg.norm(rotation @ (saved_ecef - BASE_ECEF_M) - saved_enu))
                maxima['fixed_base_frame_residual_m'] = max(maxima['fixed_base_frame_residual_m'], frame_residual)
                if frame_residual > COORDINATE_TOLERANCE_M:
                    raise ValueError(f'Archived coordinates disagree with the fixed base ENU frame at {utc_sec}')
                gt_residual = float(np.linalg.norm(saved_ecef - canonical_gt[utc_sec]))
                maxima['canonical_gt_residual_m'] = max(maxima['canonical_gt_residual_m'], gt_residual)
                if gt_residual > COORDINATE_TOLERANCE_M:
                    raise ValueError(f'Archived GT differs from canonical GT at {utc_sec}')
            estimate = field(record, 'estimated')
            try:
                antenna_enu = vector(field(estimate, 'antenna_pos_enu_m'), 'antenna estimate')
            except (ValueError, TypeError):
                counts['gnss_missing_or_nonfinite_antenna_not_exported'] += 1
                continue
            saved_error = field(record, 'estimated_error_enu_m')
            if sanitized:
                error = vector(saved_error, 'saved signed ENU error')
            elif saved_error is not None:
                error = np.asarray(saved_error, dtype=float)
            else:
                error = None
            if error is not None and error.shape == (3,) and np.isfinite(error).all():
                residual = float(np.linalg.norm(antenna_enu - local_gt_enu - error))
                maxima['saved_error_identity_residual_m'] = max(maxima['saved_error_identity_residual_m'], residual)
                counts['gnss_records_with_validated_signed_error'] += 1
                if residual > COORDINATE_TOLERANCE_M:
                    raise ValueError(f'Saved antenna/error identity disagrees with canonical GT and fixed base ENU frame at {utc_sec}')
            # Never reconstruct this estimate from GT + the archived error.
            antenna_ecef = BASE_ECEF_M + rotation.T @ antenna_enu
            rows.append((utc_sec, *antenna_ecef))
    if not rows:
        raise ValueError('Archive has no finite GNSS antenna positions')
    rows = np.array(sorted(rows))
    counts['exported_gnss_epochs'] = len(rows)
    counts['exported_epochs_in_common_window'] = int(np.sum((rows[:, 0] >= 65380) & (rows[:, 0] <= 69419)))
    counts['common_grid_epochs_without_native_gnss_record'] = len(set(range(65380, 69420)) - gnss_window_times)
    counts['chunks'] = len(chunks)
    return rows, {
        'passed': True, 'counts': dict(counts), 'max_residuals': dict(maxima),
        'coordinate_tolerance_m': COORDINATE_TOLERANCE_M,
        'duplicate_timestamp_count': 0, 'duplicate_policy': 'reject every duplicate timestamp',
        'utc_gps_crosscheck_tolerance_s': 2e-6,
        'first_utc_sec': float(rows[0, 0]), 'last_utc_sec': float(rows[-1, 0]),
        'vision_frame_actions': dict(vision_actions),
        'visibility_service_enabled': dict(visibility_enabled), 'chunks': chunks,
        'archive_formats': dict(archive_formats),
        'frame_validation': 'fixed base ENU transform and saved signed errors checked against local canonical GT; '
                           'legacy archives also validate saved GT ECEF/ENU coordinate pairs',
    }


def import_archive(archive, variant, output, ground_truth):
    archive, output, ground_truth = (Path(path).resolve() for path in (archive, output, ground_truth))
    with archive.open('rb') as stream:
        if stream.read(100).startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise ValueError('Archive is a Git LFS pointer; run git lfs pull first')
    executable = shutil.which('7z') or shutil.which('7zz')
    if executable is None:
        raise RuntimeError('7z or 7zz is required; install the p7zip-full or 7zip package')
    listing = subprocess.run([executable, 'l', '-slt', '-ba', str(archive)],
                             check=True, capture_output=True, text=True).stdout
    members = archive_members(listing)
    with tempfile.TemporaryDirectory(prefix='sea-rvins-import-') as temporary:
        extracted = Path(temporary)
        subprocess.run([executable, 'x', '-y', '-bd', f'-o{extracted}', str(archive)],
                       check=True, capture_output=True, text=True)
        files = [extracted / name for name, directory in members if not directory]
        if any(path.is_symlink() or not path.is_file() for path in files):
            raise ValueError('Extracted archive members are not regular files')
        rows, verification = convert_pickles(files, ground_truth)
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(output / 'est.csv', rows, delimiter=',', fmt='%.12f',
               header='utc_sec,ecef_x,ecef_y,ecef_z', comments='')
    manifest = {
        'method': f'SeA-RVINS-{variant}-robust', 'display_name': f'SeA-RVINS ({variant})',
        'operation': 'import existing saved results; estimator was not rerun',
        'archive_filename': archive.name, 'archive_bytes': archive.stat().st_size,
        'position_field': 'estimated.antenna_pos_enu_m', 'reference_point': 'ALT1 antenna',
        'lever_arm_already_applied': True,
        'native_frame': 'fixed ENU at the TEX-CUP base station',
        'enu_origin_ecef_m': BASE_ECEF_M.tolist(), 'ecef_to_enu_rotation': base_rotation().tolist(),
        'conversion': 'antenna_ecef = enu_origin_ecef_m + ecef_to_enu_rotation.T @ antenna_enu',
        'origin_evidence': 'TEX-CUP base RINEX header and benchmark METHOD_SETUP_GUIDE.md; '
                           'validated against local canonical GT and saved signed ENU errors; '
                           'legacy records also validate saved GT ECEF/ENU coordinate pairs',
        'ground_truth_usage': 'validation and subsequent scoring only; no fitted alignment or estimate reconstruction',
        'native_time': 'GPS seconds since 1980-01-06; independently crosschecked against saved UTC datetime',
        'output_time': 'UTC seconds of day 2019-05-09', 'gps_minus_utc_seconds': LEAP_SECONDS,
        'epoch_selection': 'all is_gnss_epoch records with finite saved antenna positions; no IAR/status/error filtering',
        'common_scoring_window_utc': ['18:09:40', '19:16:59'], 'common_scoring_epochs': 4040,
        'source_contract_limitations': [
            'Archives contain records and chunk save times, but no source commit or effective configuration manifest.',
            'Variant labels are supplied by the user and archive names, not independently proven configuration.',
            'Vision and visibility counters describe saved diagnostics; they do not recover every estimator setting.',
            'Imported trajectories do not establish a new estimator rerun or independently reproduce its runtime configuration.',
        ],
    }
    (output / 'verification.json').write_text(json.dumps(verification, indent=2) + '\n')
    (output / 'run_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return verification


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--variant', choices=('scalar', 'latent', 'batch'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--gt', type=Path, default=default_ground_truth())
    args = parser.parse_args()
    report = import_archive(args.archive, args.variant, args.out, args.gt)
    print(json.dumps({key: value for key, value in report.items() if key != 'chunks'}, indent=2))


if __name__ == '__main__':
    main()
