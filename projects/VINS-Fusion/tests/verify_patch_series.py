#!/usr/bin/env python3
"""Apply the active patches to pinned files and compare the compiled source tree."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    project = Path(__file__).resolve().parents[1]
    upstream, patches = project/'upstream', project/'patches'
    def git(directory, *args):
        return subprocess.check_output(['git','-C',str(directory),*args])
    revision = (patches/'vins_fusion_upstream.commit').read_text().strip()
    assert git(upstream,'rev-parse','HEAD').decode().strip() == revision
    names = (patches/'series').read_text().splitlines()
    expected_paths = set()
    for name in names:
        for line in (patches/name).read_text().splitlines():
            if line.startswith('+++ b/'): expected_paths.add(line[6:])
    actual_paths = set(git(upstream,'diff','--name-only').decode().splitlines())
    assert actual_paths == expected_paths, (actual_paths,expected_paths)
    with tempfile.TemporaryDirectory(prefix='vins-source-verification-') as directory:
        temp = Path(directory)
        git(temp,'init','-q')
        for relative in expected_paths:
            dest = temp/relative
            dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_bytes(git(upstream,'show',f'{revision}:{relative}'))
        for name in names: git(temp,'apply',str(patches/name))
        for relative in expected_paths:
            assert (temp/relative).read_bytes() == (upstream/relative).read_bytes(), relative
    estimator = (upstream/'vins_estimator/src/estimator/estimator.cpp').read_text()
    global_code = (upstream/'global_fusion/src/globalOpt.cpp').read_text()
    assert 'if(inputImageCnt % 2 == 0)' in estimator
    assert 'options.max_num_iterations = 5;' in global_code
    assert 'SetParameterBlockConstant' not in global_code
    report = dict(revision=revision, patch_series=names,
        all_patched_files_byte_identical=sorted(expected_paths),
        upstream_frame_admission=True, upstream_global_iterations=5,
        all_global_poses_variable=True)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__': main()
