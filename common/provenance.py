#!/usr/bin/env python3
"""Export provenance without tying it to the machine that produced a run.

Paths in the checkout are repository-relative. TEX-CUP inputs previously read
from the estimator checkout refer to locally prepared data/tex_cup inputs. Environment,
home and temporary locations are symbolic provenance references, not commands
or paths that a reader should create. Scientific configuration and numeric
values are retained; verbose file-identity audit records are omitted.

To normalize existing metadata, from the repository root:
  python common/provenance.py results/final/*.source.json figures/*.provenance.json
"""
import argparse
import json
import os
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
PATH_CONVENTION = (
    'Paths within the benchmark are relative to the repository root. '
    'TEX-CUP input paths refer to data/tex_cup. '
    'Angle-bracket environment, home and temporary roots are symbolic references, '
    'not executable paths. Source run paths identify the original generated runs; '
    'those runtime directories need not be distributed with the saved results. '
    'Scientific configuration, coordinate/time rules and run status are retained.'
)


def compact_metadata(value):
    """Keep scientific/run metadata while dropping obsolete file-identity audits."""
    if isinstance(value, dict):
        output = {}
        for key, child in value.items():
            name = str(key).lower()
            if ('sha256' in name or name in ('checksum_provenance', 'checksum_note',
                                            'hash', 'hashes', 'digest', 'digests')
                    or name.endswith('_hashes')):
                continue
            if key == 'chunks' and isinstance(child, list):
                continue
            output[key] = compact_metadata(child)
        return output
    if isinstance(value, list):
        return [compact_metadata(child) for child in value]
    return value


def portable_metadata(value, *, repo_root=ROOT, data_roots=()):
    """Normalize known filesystem roots in nested JSON, including keys and YAML.

    Explicit roots also allow metadata from a relocated checkout to be migrated.
    Root boundaries are checked so similarly named directories are not changed.
    ROS topics, URLs, system paths and scientific values are retained.
    """
    repo_root = os.path.abspath(Path(repo_root).expanduser()).rstrip('/')
    configured_data = os.environ.get('TEXCUP_DATA')
    roots = [*data_roots, *([configured_data] if configured_data else [])]
    # Runners commonly resolve inputs through symlinks. Recognize both the
    # configured spelling and the resolved path without reading dataset bytes.
    dataset_roots = {str(path).rstrip('/') for root in roots
                     for path in (Path(os.path.abspath(Path(root).expanduser())),
                                  Path(root).expanduser().resolve())}
    if '' in dataset_roots or not repo_root:
        raise ValueError('A provenance root cannot be the filesystem root')
    mappings = [(root, 'data/tex_cup') for root in dataset_roots]
    mappings.append((repo_root, ''))

    def normalize_paths(text):
        for root, replacement in sorted(mappings, key=lambda item: -len(item[0])):
            # Remove the separator with the repository prefix, but preserve it
            # after an explicit data-root replacement.
            text = re.sub(r'(?<![\w/<>])' + re.escape(root) + r'/(?=\S)',
                          replacement + '/' if replacement else '', text)
            text = re.sub(r'(?<![\w/<>])' + re.escape(root) + r'(?=$|[\s\"\'<>;,)}\]])',
                          replacement or '.', text)
        # Old dataset locations are recorded in existing run manifests. This
        # recognizes the dataset suffix without encoding an author's username.
        text = re.sub(r'(?<![\w:/<>])/(?:[^/\s\"\'<>]+/)*'
                      r'UrbanRTK-INS-FGO/data/tex_cup(?=/|$|[\s\"\'<>;,)}\]])',
                      'data/tex_cup', text)
        text = re.sub(r'(?<![\w:/<>])/(?:[^/\s\"\'<>]+/)*'
                      r'(?:miniconda\d*|anaconda\d*|miniforge\d*|mambaforge)/envs/'
                      r'([^/\s\"\'<>]+)(?=/|$)',
                      lambda match: '<conda-env:' + match[1] + '>', text)
        text = re.sub(r'(?<![\w:/<>])/(?:home|Users)/[^/\s\"\'<>]+(?=/|$)', '<home>', text)
        text = re.sub(r'(?<![\w:/<>])/root(?=/|$)', '<home>', text)
        text = re.sub(r'(?<![\w:/<>])/(?:private/)?(?:tmp|var/tmp)/[^/\s\"\'<>]+',
                      '<temporary>', text)
        text = re.sub(r'(?<![\w:/<>])/(?:private/)?var/folders/[^/]+/[^/]+/T/[^/\s\"\'<>]+',
                      '<temporary>', text)
        # This parser-compatible navigation file was moved unchanged into the
        # common navigation-input directory.
        text = re.sub(r'(?<![\w/])projects/GVINS/loaders/brdm1290_v304\.19p'
                      r'(?=$|[\s\"\'<>;,)}\]])', 'data/tex_cup/brdm1290_v304.19p', text)
        return text

    def normalize_text(text):
        # An HTTP URL may itself contain /home/name; it is not a local path.
        parts = re.split(r'(https?://[^\s\"\'<>]+)', text)
        return ''.join(part if index % 2 else normalize_paths(part)
                       for index, part in enumerate(parts))

    def visit(item):
        if isinstance(item, dict):
            result = {}
            for key, child in item.items():
                normalized_key = normalize_text(key)
                if normalized_key in result:
                    raise ValueError(f'Path normalization would merge metadata keys: {key}')
                result[normalized_key] = visit(child)
            return result
        if isinstance(item, list):
            return [visit(child) for child in item]
        if isinstance(item, str):
            return normalize_text(item)
        return item

    normalized = visit(compact_metadata(value))
    if isinstance(normalized, dict):
        normalized['path_convention'] = PATH_CONVENTION
    return normalized


def normalize_file(path, *, repo_root=ROOT, data_roots=()):
    """Normalize a JSON sidecar in place; return whether its content changed."""
    path = Path(path)
    original = json.loads(path.read_text())
    normalized = portable_metadata(original, repo_root=repo_root, data_roots=data_roots)
    if normalized == original:
        return False
    path.write_text(json.dumps(normalized, indent=2, allow_nan=False) + '\n')
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', type=Path, nargs='+', help='JSON metadata files to normalize')
    parser.add_argument('--repo-root', type=Path, default=ROOT,
                        help='Original checkout root for metadata copied from another machine')
    parser.add_argument('--data-root', type=Path, action='append', default=[],
                        help='Additional original TEX-CUP input root; repeat if needed')
    args = parser.parse_args()
    for path in args.files:
        if normalize_file(path, repo_root=args.repo_root, data_roots=args.data_root):
            print(path)
