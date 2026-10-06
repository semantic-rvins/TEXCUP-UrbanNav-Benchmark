#!/usr/bin/env python3
"""Compile the actual feature-counting source with ASan/UBSan edge fixtures.

The fixture extracts featuresDetection through its two counting loops, before
image masking/detection. It uses the unchanged upstream source as a negative
control and needs a C++14 compiler, but no ROS, OpenCV, or estimator replay.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


FIXTURE = r'''
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace cv { struct Point2f { float x, y; }; }
using std::vector;
struct Feature {
    cv::Point2f point;
    const cv::Point2f &keyPoint() { return point; }
};
struct Frame {
    using Ptr = std::shared_ptr<Frame>;
    std::unordered_map<int,std::shared_ptr<Feature>> points;
    auto features() { return points; }
};
struct Tracking {
    int track_max_features_=200, block_cnts_=40, block_cols_=10, block_rows_=4;
    int track_max_block_features_=5;
    vector<cv::Point2f> pts2d_ref_, pts2d_new_;
    vector<std::pair<int,int>> block_indexs_{{204,183}};
    vector<int> observed_counts;
    void featuresDetection(Frame::Ptr &frame, bool ismask);
};
'''

CASES = r'''
void check(bool condition, const char *message) {
    if (!condition) { std::cerr << message << "\n"; std::exit(2); }
}
int main(int argc, char **argv) {
    std::string mode=argv[1];
    Tracking tracking;
    auto frame=std::make_shared<Frame>();
    vector<int> expected(40,0);
    auto feature=[&](float x, float y, int cell) {
        frame->points[frame->points.size()]=std::make_shared<Feature>(Feature{{x,y}});
        if (cell>=0) ++expected[cell];
    };
    auto raw=[&](float x, float y, int cell) {
        tracking.pts2d_new_.push_back({x,y});
        if (cell>=0) ++expected[cell];
    };
    if (mode=="undistorted_feature") {
        // Raw (2030,720), accepted by the 5 px border check, undistorts to this
        // point using the recorded full-resolution TEX-CUP camera calibration.
        feature(2041.2994384765625f,723.9906616210938f,39);
    } else if (mode=="raw_feature") {
        // A valid raw pixel in the width remainder (2040..2047).
        raw(2042.f,726.f,39);
    } else if (mode=="ordinary") {
        for (int row=0;row<4;++row) for (int col=0;col<10;++col) {
            int cell=row*10+col;
            feature(col*204.f+102.f,row*183.f+91.f,cell);
            raw(col*204.f,row*183.f,cell);
        }
    } else if (mode=="edges") {
        feature(2040.f,100.f,9); raw(2047.f,731.f,39);
        feature(100.f,732.f,30); raw(2048.f,732.f,39);
        feature(-1.f,200.f,10); raw(300.f,-1.f,1);
        feature(-300.f,-300.f,0); raw(2040.f,-1000.f,9);
    } else if (mode=="extreme") {
        float largest=std::numeric_limits<float>::max();
        feature(largest,largest,39); raw(-largest,-largest,0);
        feature(largest,-largest,9); raw(-largest,largest,30);
    } else if (mode=="nonfinite") {
        float nan=std::numeric_limits<float>::quiet_NaN();
        float inf=std::numeric_limits<float>::infinity();
        feature(nan,1.f,-1); feature(1.f,nan,-1);
        feature(inf,1.f,-1); feature(1.f,-inf,-1);
        raw(nan,1.f,-1); raw(1.f,nan,-1);
        raw(-inf,1.f,-1); raw(1.f,inf,-1);
        feature(1.f,1.f,0); raw(205.f,184.f,11);
    } else if (mode!="empty") {
        return 3;
    }
    tracking.featuresDetection(frame,false);
    check(tracking.observed_counts==expected,"Per-cell counts differ from expected; point lost or counted twice");
    std::cout << "PASS " << mode << "\n";
}
'''


def source_fixture(source):
    start = source.index('void Tracking::featuresDetection(')
    end = source.index('    // 设置感兴趣区域', start)
    return (FIXTURE + source[start:end]
            + '\n    observed_counts.assign(features_cnts, features_cnts + block_cnts_);\n}\n' + CASES)


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=project/'upstream/ic_gvins/ic_gvins/tracking/tracking.cc')
    parser.add_argument('--baseline-source', type=Path,
                        help='Unpatched tracking.cc; default is the pinned upstream git object')
    parser.add_argument('--out', type=Path, help='Keep generated fixtures and sanitizer logs here')
    args = parser.parse_args()
    compiler = shutil.which(os.environ.get('CXX', 'c++'))
    if not compiler:
        parser.error('A C++ compiler is required')
    if args.baseline_source:
        baseline = args.baseline_source.read_text()
    else:
        commit = (project/'patches/icgvins_upstream.commit').read_text().strip()
        baseline = subprocess.run(['git', '-C', str(project/'upstream'), 'show',
            f'{commit}:ic_gvins/ic_gvins/tracking/tracking.cc'],
            check=True, capture_output=True, text=True).stdout
    with tempfile.TemporaryDirectory(prefix='icgvins-feature-grid-') as temporary:
        output = args.out.resolve() if args.out else Path(temporary)
        output.mkdir(parents=True, exist_ok=True)
        report = {'fixed': {}, 'baseline': {}}
        for label, source in [('fixed', args.source.read_text()), ('baseline', baseline)]:
            cpp, executable = output/f'{label}.cc', output/label
            cpp.write_text(source_fixture(source))
            # UBSan stops the original out-of-bounds access before ASan sees it;
            # the negative control therefore uses ASan alone.
            sanitizers = 'address,undefined,float-cast-overflow' if label == 'fixed' else 'address'
            subprocess.run([compiler, '-std=c++14', '-O0', '-g', '-fno-omit-frame-pointer',
                f'-fsanitize={sanitizers}', '-fno-sanitize-recover=all',
                '-no-pie', str(cpp), '-o', str(executable)], check=True)
            cases = (['undistorted_feature', 'raw_feature', 'ordinary', 'edges', 'extreme', 'nonfinite', 'empty']
                     if label == 'fixed' else ['ordinary', 'undistorted_feature', 'raw_feature'])
            for case in cases:
                result = subprocess.run([str(executable), case], capture_output=True, text=True, timeout=10)
                (output/f'{label}-{case}.log').write_text(result.stdout + result.stderr)
                expected_overflow = label == 'baseline' and case != 'ordinary'
                overflow = 'dynamic-stack-buffer-overflow' in result.stderr
                passed = (result.returncode != 0 and overflow) if expected_overflow else result.returncode == 0
                report[label][case] = {'passed': passed, 'returncode': result.returncode,
                                       'expected_asan_overflow': expected_overflow, 'asan_overflow': overflow}
                if not passed:
                    raise RuntimeError(f'{label}/{case} failed:\n{result.stdout}\n{result.stderr}')
                print(f'PASS {label}/{case}' + (' (expected original overflow)' if expected_overflow else ''))
        report['passed'] = True
        (output/'verification.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
