#!/usr/bin/env python3
"""Verify the IC-GVINS checkout equals the pinned source plus active patches."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def verify(project):
    upstream = project/'upstream'
    revision = (project/'patches/icgvins_upstream.commit').read_text().strip()
    actual_revision = subprocess.check_output(
        ['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if actual_revision != revision:
        raise ValueError(f'Expected upstream {revision}, got {actual_revision}')
    patches = sorted((project/'patches').glob('*.patch'))
    paths = sorted({line[6:] for patch in patches for line in patch.read_text().splitlines()
                    if line.startswith('+++ b/')})
    actual_changes = set(subprocess.check_output(
        ['git', '-C', str(upstream), 'diff', '--name-only', revision], text=True).splitlines())
    if actual_changes != set(paths):
        raise ValueError(f'Unexpected changed source files: {actual_changes ^ set(paths)}')
    with tempfile.TemporaryDirectory(prefix='icgvins-source-') as directory:
        temp = Path(directory)
        for path in paths:
            target = temp/path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(
                ['git', '-C', str(upstream), 'show', f'{revision}:{path}']))
        for patch in patches:
            subprocess.run(['git', 'apply', '--check', str(patch)], cwd=temp, check=True)
            subprocess.run(['git', 'apply', str(patch)], cwd=temp, check=True)
        for path in paths:
            if (temp/path).read_bytes() != (upstream/path).read_bytes():
                raise ValueError(f'Checkout does not match the active patches: {path}')
    return dict(upstream_commit=revision, patch_application_matches_source=True,
                active_patches=[p.name for p in patches],
                files_verified=paths, archived_algorithm_patches_applied=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    try:
        report = verify(Path(__file__).resolve().parents[1])
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.error(str(error))
    text = json.dumps(report, indent=2)+'\n'
    if args.out:
        args.out.write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
