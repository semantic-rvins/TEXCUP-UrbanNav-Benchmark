#!/usr/bin/env python3
"""Compile the actual state-removal method with a small, thread-free fixture.

The fixture supplies state/GNSS containers and records IMU samples forwarded
between preintegrations. It does not reproduce the removal implementation or
test the estimator's numerical accuracy. --source permits a pre-fix control.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


FIXTURE = r'''
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
using Vector = std::vector<double>;
std::ostringstream log_messages;
#define LOGI log_messages
struct Logging { static double doubleData(double time) { return time; } };
struct MISC {
    static constexpr double MINIMUM_TIME_INTERVAL = .0001;
    static bool isTheSameTimeNode(double a, double b, double interval) {
        return std::abs(a-b) < interval;
    }
};
struct GNSS { double time; Vector position; Vector std; };
struct PreintegrationFixture {
    Vector samples;
    Vector imuBuffer() const { return samples; }
    void addNewImu(double sample) { samples.push_back(sample); }
};
class GVINS {
public:
    std::deque<double> timelist_{100.,101.,102.,103.};
    std::deque<int> statedatalist_{0,1,2,3};
    std::deque<GNSS> gnsslist_;
    std::vector<double> unused_time_nodes_;
    std::deque<std::shared_ptr<PreintegrationFixture>> preintegrationlist_;
    // This fixture exercises GNSS ownership with no marginalization prior.
    // The real prior binding functions and state storage are tested separately.
    struct PriorStateBinding { size_t block_index; double time; bool is_pose; };
    std::vector<PriorStateBinding> capturePriorStateBindings() const { return {}; }
    void restorePriorStateBindings(const std::vector<PriorStateBinding> &) {}
    GVINS() {
        for (double t: {100.,101.,102.}) {
            auto interval=std::make_shared<PreintegrationFixture>();
            interval->samples={t,t+.5,t+1.};
            preintegrationlist_.push_back(interval);
        }
    }
    int getStateDataIndex(double time) {
        for (size_t i=0;i<timelist_.size();++i)
            if (MISC::isTheSameTimeNode(timelist_[i],time,MISC::MINIMUM_TIME_INTERVAL))
                return static_cast<int>(i);
        return -1;
    }
    bool removeUnusedTimeNode();
};
'''

CASES = r'''
void check(bool condition, const char *message) {
    if (!condition) { std::cerr << message << "\n"; std::exit(2); }
}
GNSS fix(double time) { return GNSS{time,{10.,20.,30.},{.05,.05,.1}}; }
void check_all_gnss_states(GVINS &g) {
    for (const auto &gnss:g.gnsslist_) {
        check(g.getStateDataIndex(gnss.time)>=0,"GNSS measurement lost its state");
        check(gnss.position==Vector({10.,20.,30.}),"GNSS position changed");
        check(gnss.std==Vector({.05,.05,.1}),"GNSS uncertainty changed");
    }
    check(g.statedatalist_.size()==g.timelist_.size(),"State/time containers disagree");
    check(g.preintegrationlist_.size()+1==g.timelist_.size(),"IMU interval count changed incorrectly");
}
int main(int argc, char **argv) {
    std::string mode=argv[1];
    GVINS g;
    if (mode=="empty") {
        check(!g.removeUnusedTimeNode(),"Empty removal queue should do nothing");
    } else if (mode=="visual_only") {
        g.gnsslist_.push_back(fix(102.));
        g.unused_time_nodes_={101.};
        auto last=g.preintegrationlist_.back();
        g.removeUnusedTimeNode();
        check(g.timelist_==std::deque<double>({100.,102.,103.}),"Unused visual state not removed");
        check(g.statedatalist_==std::deque<int>({0,2,3}),"Wrong state removed");
        check(g.preintegrationlist_[0]->samples==Vector({100.,100.5,101.,101.5,102.}),
              "Merged IMU samples lost or duplicated");
        check(g.preintegrationlist_.back()==last,"Unrelated IMU interval changed");
    } else if (mode=="exact" || mode=="aligned" || mode=="duplicate") {
        double time=mode=="aligned"?101.00005:101.;
        g.gnsslist_.push_back(fix(time));
        if (mode=="duplicate") g.gnsslist_.push_back(fix(101.00005));
        auto intervals=g.preintegrationlist_;
        g.unused_time_nodes_={101.};
        g.removeUnusedTimeNode();
        check_all_gnss_states(g);
        check(g.timelist_==std::deque<double>({100.,101.,102.,103.}),"GNSS state removed");
        check(g.preintegrationlist_==intervals,"Retained GNSS IMU intervals changed");
        check(g.gnsslist_.size()==(mode=="duplicate"?2u:1u),"GNSS measurement discarded");
    } else if (mode=="mixed") {
        g.gnsslist_.push_back(fix(101.));
        g.unused_time_nodes_={101.,102.};
        auto first=g.preintegrationlist_[0];
        g.removeUnusedTimeNode();
        check(g.timelist_==std::deque<double>({100.,101.,103.}),"Mixed cleanup removed wrong states");
        check(g.preintegrationlist_[0]==first,"GNSS interval changed during unrelated removal");
        check(g.preintegrationlist_[1]->samples==Vector({101.,101.5,102.,102.5,103.}),
              "Unrelated IMU merge changed");
    } else if (mode=="release") {
        g.gnsslist_.push_back(fix(101.));
        g.unused_time_nodes_={101.};
        g.removeUnusedTimeNode();
        check_all_gnss_states(g);
        g.gnsslist_.clear();
        g.unused_time_nodes_={101.};
        g.removeUnusedTimeNode();
        check(g.getStateDataIndex(101.)<0,"State retained after its measurement was removed");
    } else {
        return 3;
    }
    check(g.unused_time_nodes_.empty(),"Removal queue not drained");
    check_all_gnss_states(g);
    std::cout << "PASS " << mode << "\n";
}
'''


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=project/'upstream/ic_gvins/ic_gvins/ic_gvins.cc')
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('bool GVINS::removeUnusedTimeNode() {')
    end = source.index('\nbool GVINS::insertNewGnssTimeNode()', start)
    compiler = shutil.which(os.environ.get('CXX', 'c++'))
    if not compiler:
        parser.error('A C++ compiler is required')
    with tempfile.TemporaryDirectory(prefix='icgvins-gnss-state-') as directory:
        temp = Path(directory)
        cpp, exe = temp/'check.cpp', temp/'check'
        cpp.write_text(FIXTURE + source[start:end] + CASES)
        subprocess.run([compiler, '-std=c++14', '-O1', '-g', '-D_GLIBCXX_ASSERTIONS',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        str(cpp), '-o', str(exe)], check=True)
        failed = []
        for case in ['empty', 'visual_only', 'exact', 'aligned', 'duplicate', 'mixed', 'release']:
            result = subprocess.run([str(exe), case], timeout=5)
            if result.returncode:
                failed.append(case)
        if failed:
            raise SystemExit('FAILED: ' + ', '.join(failed))


if __name__ == '__main__':
    main()
