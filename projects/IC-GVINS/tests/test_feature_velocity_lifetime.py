#!/usr/bin/env python3
"""Check the actual feature-velocity expression with Eigen and AddressSanitizer.

Compiles the expression and Camera::pixel2cam from the selected source. Only
the camera/point fixture is simplified; Eigen's expression evaluation is real.
The pinned upstream expression must trigger stack-use-after-scope, while the
patched expression must produce the expected velocity without a sanitizer error.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def fixture(tracking, camera):
    expression = re.search(r'^\s*(?:auto|Vector3d) velocity = \(camera_->pixel2cam\(pts2d_matched_undis\[k\]\).*?;',
                           tracking, re.M)
    assert expression, 'Feature velocity expression not found'
    start = camera.index('Vector3d Camera::pixel2cam(')
    end = camera.index('\n}', start) + 2
    return r'''
#include <Eigen/Core>
#include <cmath>
#include <iostream>
using Eigen::Vector3d;
namespace cv { struct Point2f { float x, y; }; }
struct Camera {
    double fx_=1000, fy_=1200, cx_=100, cy_=200, skew_=0;
    Vector3d pixel2cam(const cv::Point2f &pixel) const;
};
''' + camera[start:end] + r'''
int main() {
    Camera camera;
    Camera *camera_ = &camera;
    cv::Point2f pts2d_matched_undis[]{{140,260}};
    cv::Point2f pts2d_map_undis[]{{120,200}};
    const int k=0;
    const double dt=0.1;
''' + expression.group() + r'''
    volatile double x=velocity.x(), y=velocity.y(), z=velocity.z();
    if (std::abs(x-0.2)>1e-12 || std::abs(y-0.5)>1e-12 || z!=0.0) return 2;
    std::cout << "PASS feature velocity: " << x << ", " << y << ", " << z << "\n";
}
'''


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=project/'upstream/ic_gvins/ic_gvins/tracking/tracking.cc')
    parser.add_argument('--out', type=Path, help='Keep generated C++ fixtures and sanitizer logs')
    args = parser.parse_args()
    compiler = shutil.which(os.environ.get('CXX', 'c++'))
    assert compiler, 'A C++ compiler is required'
    eigen = next((p for p in [Path(os.environ.get('CONDA_PREFIX', '/usr'))/'include/eigen3',
                             Path('/usr/include/eigen3')] if (p/'Eigen/Core').exists()), None)
    assert eigen, 'Eigen headers are required (source projects/IC-GVINS/env.sh)'
    commit = (project/'patches/icgvins_upstream.commit').read_text().strip()
    baseline = subprocess.check_output(['git', '-C', str(project/'upstream'), 'show',
        f'{commit}:ic_gvins/ic_gvins/tracking/tracking.cc'], text=True)
    camera = (args.source.parent/'camera.cc').read_text()
    with tempfile.TemporaryDirectory(prefix='icgvins-feature-velocity-') as temporary:
        out = args.out.resolve() if args.out else Path(temporary)
        out.mkdir(parents=True, exist_ok=True)
        report = {}
        for label, source in [('fixed', args.source.read_text()), ('baseline', baseline)]:
            cpp, binary = out/f'{label}.cc', out/label
            cpp.write_text(fixture(source, camera))
            subprocess.run([compiler, '-std=c++14', '-O1', '-g', '-no-pie', '-I'+str(eigen),
                '-fsanitize=address', '-fsanitize-address-use-after-scope', '-fno-omit-frame-pointer',
                str(cpp), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True,
                env=dict(os.environ, ASAN_OPTIONS='detect_leaks=0:halt_on_error=1'))
            log = result.stdout + result.stderr
            (out/f'{label}.log').write_text(log)
            passed = (result.returncode == 0 and 'PASS feature velocity' in log) if label == 'fixed' else (
                result.returncode != 0 and 'ERROR: AddressSanitizer: stack-use-after-scope' in log)
            report[label] = {'returncode': result.returncode, 'passed': passed}
            assert passed, f'{label} did not meet expectation:\n{log}'
        (out/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
