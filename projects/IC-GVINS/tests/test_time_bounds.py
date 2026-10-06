#!/usr/bin/env python3
"""Exercise the compiled IC-GVINS IMU slicing and empty-state lookup code."""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

SOURCE = r'''
#include "misc.h"
#include <cmath>
#include <iostream>
#include <string>
int main(int argc, char **argv) {
    std::string mode=argv[1];
    if (mode=="empty_states") {
        std::deque<double> times;
        MISC::getStateDataIndex(times,1.,.0001);
        return 0;
    }
    std::deque<std::pair<IMU,IntegrationState>> window;
    for (int i=0; i<30; ++i) {
        IMU imu{}; imu.time=100.+i*.01; imu.dt=.01;
        imu.dtheta=Vector3d(.01,.02,.03)*imu.dt;
        imu.dvel=Vector3d(.1,.2,.3)*imu.dt;
        window.emplace_back(imu,IntegrationState{});
    }
    double start=100.035,end=100.185;
    if(mode=="missing_start") start=99.99;
    if(mode=="missing_end") end=100.4;
    if(mode=="reversed") {start=100.017555;end=99.979259;}
    if(mode=="equal") end=start;
    if(mode=="empty_imu") window.clear();
    std::vector<IMU> series;
    bool ok=MISC::getImuSeriesFromTo(window,start,end,series);
    if(mode!="valid") return (!ok && series.empty())?0:2;
    if(!ok || series.size()<2) return 3;
    double dt=0;Vector3d angle=Vector3d::Zero(),velocity=Vector3d::Zero();
    for(size_t i=1;i<series.size();++i) {
        if(series[i].time<=series[i-1].time || series[i].dt<=0) return 4;
        dt+=series[i].dt;angle+=series[i].dtheta;velocity+=series[i].dvel;
    }
    if(std::abs(dt-(end-start))>1e-10 || (angle-Vector3d(.01,.02,.03)*dt).norm()>1e-10 ||
       (velocity-Vector3d(.1,.2,.3)*dt).norm()>1e-10) return 5;
    std::cout << "PASS valid interval preserves duration and IMU increments\n";
}
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case',action='append',choices=['valid','missing_start','missing_end','reversed','equal','empty_imu','empty_states'])
    a=ap.parse_args()
    project=Path(__file__).resolve().parents[1]
    flags=(project/'catkin_ws/build/ic_gvins/CMakeFiles/ic_gvins_core.dir/flags.make').read_text()
    includes=shlex.split(re.search(r'^CXX_INCLUDES = (.*)$',flags,re.M)[1])
    lib=project/'catkin_ws/build/ic_gvins'
    compiler=Path(os.environ['CONDA_PREFIX'])/'bin/c++'
    with tempfile.TemporaryDirectory(prefix='icgvins-time-bounds-') as directory:
        temp=Path(directory);src=temp/'check.cpp';src.write_text(SOURCE)
        exe=temp/'check'
        subprocess.run([str(compiler),'-std=c++14','-O2',*includes,str(src),'-L'+str(lib),
                        '-Wl,-rpath,'+str(lib),'-lic_gvins_core','-o',str(exe)],check=True)
        failed=[]
        for case in a.case or ['valid','missing_start','missing_end','reversed','equal','empty_imu','empty_states']:
            try:
                result=subprocess.run([str(exe),case],cwd=temp,timeout=5)
                print(f'{case}: returncode={result.returncode}',flush=True)
                if result.returncode:failed.append(case)
            except subprocess.TimeoutExpired:failed.append(case)
        if failed:raise SystemExit('FAILED: '+', '.join(failed))


if __name__=='__main__':main()
