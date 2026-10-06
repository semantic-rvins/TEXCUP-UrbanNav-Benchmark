#!/usr/bin/env python3
"""Compile actual global-fusion code and test its rigid-rotation invariant.

Only thread scheduling/access are adapted in a temporary copy: the worker is
disabled, private members exposed, and optimize() executes one cycle. Production
pose insertion, factors, Ceres solve and alignment-update code remain intact.
Run after building global_fusion, in the benchmark_ros environment.
"""
import argparse
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile

HARNESS = r'''
#include "globalOpt.h"
#include <iomanip>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
int main() {
    ros::Time::init();
    GlobalOptimization g;
    Eigen::Quaterniond alignment(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d shift(2., -3., .5);
    g.WGPS_T_WVIO.block<3,3>(0,0) = alignment.toRotationMatrix();
    g.WGPS_T_WVIO.block<3,1>(0,3) = shift;
    double max_norm_error=0, max_orthogonality_error=0;
    for (int i=0; i<150; ++i) {
        double t=1000.+i*.1;
        Eigen::Vector3d local(i*.2, std::sin(i*.04), .05*std::cos(i*.03));
        Eigen::Quaterniond odom(Eigen::AngleAxisd(.3*std::sin(i*.04), Eigen::Vector3d::UnitZ()));
        odom.coeffs() *= 1.+1e-10;  // small norm error, unchanged physical attitude
        g.inputOdom(t, local, odom);
        max_norm_error=std::max(max_norm_error, std::abs(g.lastQ.norm()-1.));
        check(std::abs(g.lastQ.norm()-1.)<1e-12, "inputOdom published a non-unit quaternion");
        auto &pose=g.globalPoseMap[t];
        for (int j=3; j<7; ++j) pose[j] *= 1.+1e-8;
        Eigen::Vector3d gps=alignment*local+shift;
        g.GPSPositionMap[t]={gps.x(),gps.y(),gps.z(),.05};
        g.newGPS=true;
        if (i%5!=4) continue;
        g.optimize();
        Eigen::Matrix3d R=g.WGPS_T_WVIO.block<3,3>(0,0);
        double err=(R.transpose()*R-Eigen::Matrix3d::Identity()).norm();
        max_orthogonality_error=std::max(max_orthogonality_error,err);
        check(err<1e-12 && std::abs(R.determinant()-1.)<1e-12, "alignment is not a rigid rotation");
        for (const auto &entry:g.globalPoseMap) {
            const auto &p=entry.second;
            Eigen::Quaterniond q(p[3],p[4],p[5],p[6]);
            max_norm_error=std::max(max_norm_error,std::abs(q.norm()-1.));
            check(q.coeffs().allFinite() && std::abs(q.norm()-1.)<1e-12, "optimizer retained a non-unit quaternion");
        }
        Eigen::Vector3d mapped=R*local+g.WGPS_T_WVIO.block<3,1>(0,3);
        const auto &last=g.globalPoseMap.rbegin()->second;
        check((mapped-Eigen::Vector3d(last[0],last[1],last[2])).norm()<1e-10, "alignment changed the newest position");
    }
    std::cout << std::setprecision(17) << "PASS max_norm_error=" << max_norm_error
              << " max_orthogonality_error=" << max_orthogonality_error << '\n';
}
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,help='alternate globalOpt.cpp for negative control')
    a=ap.parse_args()
    project=Path(__file__).resolve().parents[1]
    src=project/'upstream/global_fusion/src'
    build=project/'catkin_ws/build/global_fusion'
    flags=(build/'CMakeFiles/global_fusion_node.dir/flags.make').read_text()
    includes=shlex.split(re.search(r'^CXX_INCLUDES = (.*)$',flags,re.M)[1])
    link=shlex.split((build/'CMakeFiles/global_fusion_node.dir/link.txt').read_text())
    libs=link[link.index('-o')+2:]
    with tempfile.TemporaryDirectory(prefix='vins-rotation-regression-') as directory:
        temp=Path(directory)
        for name in ['globalOpt.h','Factors.h','tic_toc.h']:
            shutil.copy2(src/name,temp/name)
        header=temp/'globalOpt.h'
        header.write_text(header.read_text().replace('private:', 'public:'))
        code=(a.source or src/'globalOpt.cpp').read_text()
        replacements={
            'threadOpt = std::thread(&GlobalOptimization::optimize, this);':'// worker disabled in deterministic test',
            'threadOpt.detach();':'// no worker in deterministic test',
            'while(true)':'for (int test_cycle=0; test_cycle<1; ++test_cycle)',
            'std::this_thread::sleep_for(dura);':'// no wall-clock sleep in deterministic test',
        }
        for old,new in replacements.items():
            assert code.count(old)==1, old
            code=code.replace(old,new)
        (temp/'globalOpt.cpp').write_text(code)
        (temp/'test.cpp').write_text(HARNESS)
        binary=temp/'rotation_test'
        subprocess.run([link[0],'-std=c++17','-O2',*includes,str(temp/'globalOpt.cpp'),
                        str(temp/'test.cpp'),'-o',str(binary),*libs],cwd=build,check=True)
        result=subprocess.run([str(binary)],cwd=temp)
        raise SystemExit(0 if result.returncode==0 else 1)


if __name__=='__main__':main()
