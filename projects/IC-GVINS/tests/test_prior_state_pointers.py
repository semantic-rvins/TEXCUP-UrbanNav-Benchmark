#!/usr/bin/env python3
"""Check actual prior rebinding/container mutations with ASan and UBSan.

Uses the real IntegrationStateData header and extracts capture/restore,
removeUnusedTimeNode, insertNewGnssTimeNode, and addNewGnssTimeNode from the
prepared upstream source. The fixture supplies thread-free map, IMU, and
preintegration plumbing; it does not copy the pointer-binding implementation.
CHECK failures throw in the fixture so tests can inspect transactional failure;
the production CHECK terminates instead.

The embedded pre-fix negative control uses the real state type and must report
an AddressSanitizer heap-use-after-free. All patched cases must pass both
sanitizers. Evidence (generated C++, binaries, per-case logs, and a JSON report)
is retained in --out, or in a printed temporary directory when omitted.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


FIXTURE = r'''
#include "preintegration/integration_state.h"
#include "common/types.h"
#include <algorithm>
#include <cstdint>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

std::ostringstream log_messages;
#define LOGI log_messages
#define LOGW log_messages
struct CheckFailure : std::runtime_error { using std::runtime_error::runtime_error; };
class CheckStream {
    bool passed_;
    std::ostringstream text_;
public:
    explicit CheckStream(bool passed) : passed_(passed) {}
    template<typename T> CheckStream &operator<<(const T &value) {
        if (!passed_) text_ << value;
        return *this;
    }
    ~CheckStream() noexcept(false) {
        if (!passed_) throw CheckFailure(text_.str());
    }
};
#define CHECK(condition) CheckStream(static_cast<bool>(condition))
struct Logging { static double doubleData(double time) { return time; } };
struct MISC {
    static constexpr double MINIMUM_TIME_INTERVAL = .0001;
    static bool isTheSameTimeNode(double a, double b, double interval) {
        return std::abs(a-b) < interval;
    }
    static bool getImuSeriesFromTo(const std::deque<std::pair<IMU, IntegrationState>> &,
                                  double start, double end, std::vector<IMU> &series) {
        series.assign(2, IMU{});
        series[0].time = start;
        series[1].time = end;
        return end > start;
    }
};
struct PreintegrationFixture {
    std::vector<double> samples;
    std::vector<double> imuBuffer() const { return samples; }
    void addNewImu(double sample) { samples.push_back(sample); }
    double deltaTime() const { return samples.back()-samples.front(); }
};
constexpr int KEYFRAME_REMOVE_SECOND_NEW = 2;
struct FrameFixture {
    double time;
    double stamp() const { return time; }
    int keyFrameState() const { return 0; }
};
struct MapFixture {
    std::map<unsigned long, std::shared_ptr<FrameFixture>> frames;
    std::vector<unsigned long> orderedKeyFrames() const {
        std::vector<unsigned long> ids;
        for (const auto &entry : frames) ids.push_back(entry.first);
        return ids;
    }
    const std::map<unsigned long, std::shared_ptr<FrameFixture>> &keyframes() const { return frames; }
};
class GVINS {
public:
    std::deque<IntegrationStateData> statedatalist_;
    std::deque<double> timelist_;
    std::deque<GNSS> gnsslist_;
    std::deque<std::shared_ptr<PreintegrationFixture>> preintegrationlist_;
    std::deque<std::pair<IMU, IntegrationState>> ins_window_;
    std::vector<double> unused_time_nodes_;
    std::vector<double *> last_marginalization_parameter_blocks_;
    double extrinsic_[8]{};
    GNSS gnss_{};
    std::shared_ptr<MapFixture> map_{std::make_shared<MapFixture>()};
    std::vector<double> recreated_times;
    const double MINMUM_SYNC_INTERVAL = .025;
    const double MAXIMUM_PREINTEGRATION_LENGTH = 10.;
    /* ACTUAL_BINDING_DECLARATION */
    std::vector<PriorStateBinding> capturePriorStateBindings() const;
    void restorePriorStateBindings(const std::vector<PriorStateBinding> &bindings);
    bool removeUnusedTimeNode();
    bool insertNewGnssTimeNode();
    void addNewGnssTimeNode();

    static IntegrationStateData state(double time) {
        IntegrationStateData s{};
        s.time = time;
        for (size_t i=0; i<7; ++i) s.pose[i] = time*10+i;
        for (size_t i=0; i<18; ++i) s.mix[i] = time*100+i;
        return s;
    }
    GVINS() {
        // Preserve the real allocator/block layout used by the ASan reproducer.
        for (int i=0; i<10; ++i) {
            double t=100.+i;
            statedatalist_.push_back(state(t));
            timelist_.push_back(t);
            if (i>0) {
                auto interval=std::make_shared<PreintegrationFixture>();
                interval->samples={t-1.,t-.5,t};
                preintegrationlist_.push_back(interval);
            }
            map_->frames[i]=std::make_shared<FrameFixture>(FrameFixture{t});
        }
        statedatalist_.pop_front();
        timelist_.pop_front();
        preintegrationlist_.pop_front();
        for (size_t i=0; i<8; ++i) extrinsic_[i]=7000.+i;
    }
    int getStateDataIndex(double time) const {
        for (size_t i=0; i<timelist_.size(); ++i)
            if (MISC::isTheSameTimeNode(timelist_[i],time,MISC::MINIMUM_TIME_INTERVAL))
                return static_cast<int>(i);
        return -1;
    }
    IntegrationStateData &at(double time) {
        int index=getStateDataIndex(time);
        if (index<0) throw std::runtime_error("fixture requested a missing state");
        return statedatalist_[index];
    }
    // Numeric propagation is outside this memory-lifetime test. Actual
    // insertion/removal routines decide which suffix nodes to create here.
    bool addNewTimeNode(double time) {
        if (time<=timelist_.back()) throw std::runtime_error("unordered recreation");
        auto interval=std::make_shared<PreintegrationFixture>();
        interval->samples={timelist_.back(),time};
        preintegrationlist_.push_back(interval);
        timelist_.push_back(time);
        statedatalist_.push_back(state(time));
        recreated_times.push_back(time);
        return true;
    }
};
'''

CASES = r'''
void check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
template<typename Function> void expect_guard(Function action, const std::string &message) {
    try { action(); }
    catch (const CheckFailure &error) {
        check(std::string(error.what()).find(message)!=std::string::npos,"wrong CHECK failure");
        std::cout << "EXPECTED CHECK: " << error.what() << "\n";
        return;
    }
    throw std::runtime_error("invalid prior binding was silently accepted");
}
void consistent(const GVINS &g) {
    check(g.timelist_.size()==g.statedatalist_.size(),"time/state containers disagree");
    check(g.preintegrationlist_.size()+1==g.statedatalist_.size(),"preintegration count disagrees");
    for (size_t i=0; i<g.timelist_.size(); ++i)
        check(g.timelist_[i]==g.statedatalist_[i].time,"state order changed");
}
void state_values_unchanged(const GVINS &g) {
    for (const auto &s:g.statedatalist_) {
        for (size_t i=0; i<7; ++i) check(s.pose[i]==s.time*10+i,"pose moved to wrong logical state");
        for (size_t i=0; i<18; ++i) check(s.mix[i]==s.time*100+i,"mix moved to wrong logical state");
    }
}
int main(int argc, char **argv) {
    try {
        const std::string mode=argv[1];
        GVINS g;
        if (mode=="middle_removal") {
            auto address_before=reinterpret_cast<std::uintptr_t>(g.at(101.).pose);
            g.last_marginalization_parameter_blocks_={g.at(106.).mix,g.extrinsic_,
                g.at(101.).pose,g.extrinsic_+7,g.at(105.).pose,g.at(101.).mix};
            g.unused_time_nodes_={102.};
            check(g.removeUnusedTimeNode(),"cleanup did not run");
            check(g.getStateDataIndex(102.)<0,"unrelated state was not removed");
            check(reinterpret_cast<std::uintptr_t>(g.at(101.).pose)!=address_before,
                  "fixture did not exercise deque survivor relocation");
            const std::vector<double*> expected{g.at(106.).mix,g.extrinsic_,
                g.at(101.).pose,g.extrinsic_+7,g.at(105.).pose,g.at(101.).mix};
            check(g.last_marginalization_parameter_blocks_==expected,"prior order/identity not restored");
            state_values_unchanged(g);
            // These are the cached addresses Ceres would write. ASan verifies
            // storage lifetime; logical assertions also detect a live wrong node.
            for (size_t i=0; i<expected.size(); ++i)
                g.last_marginalization_parameter_blocks_[i][0]=9000.+i;
            check(g.at(101.).pose[0]==9002. && g.at(101.).mix[0]==9005.,"prior wrote wrong surviving state");
            check(g.at(106.).mix[0]==9000. && g.at(105.).pose[0]==9004.,"prior block order changed");
            check(g.extrinsic_[0]==9001. && g.extrinsic_[7]==9003.,"extrinsic pointers changed");
            check(g.at(103.).pose[0]==1030.,"unrelated state overwritten");
        } else if (mode=="prior_supported_node") {
            const auto times=g.timelist_;
            const auto integrations=g.preintegrationlist_;
            g.last_marginalization_parameter_blocks_={g.at(104.).pose,g.at(104.).mix};
            const auto blocks=g.last_marginalization_parameter_blocks_;
            g.unused_time_nodes_={104.};
            check(g.removeUnusedTimeNode(),"cleanup did not run");
            check(g.timelist_==times && g.preintegrationlist_==integrations,"prior-supported state removed");
            check(g.last_marginalization_parameter_blocks_==blocks,"retained prior addresses changed");
            check(g.unused_time_nodes_.empty(),"cleanup queue not drained");
            state_values_unchanged(g);
        } else if (mode=="suffix_recreation") {
            auto old_pose=reinterpret_cast<std::uintptr_t>(g.at(109.).pose);
            auto old_mix=reinterpret_cast<std::uintptr_t>(g.at(104.).mix);
            g.last_marginalization_parameter_blocks_={g.at(109.).pose,g.at(104.).mix,
                g.extrinsic_+7,g.at(109.).mix,g.at(101.).pose,g.extrinsic_};
            g.gnss_.time=103.5;
            g.gnss_.blh=Vector3d(10.,20.,30.);
            g.gnss_.std=Vector3d(.05,.05,.1);
            check(g.insertNewGnssTimeNode(),"GNSS insertion did not run");
            check(g.timelist_==std::deque<double>({101.,102.,103.,103.5,104.,105.,106.,107.,108.,109.}),
                  "suffix recreation changed time order");
            check(g.recreated_times==std::vector<double>({103.5,104.,105.,106.,107.,108.,109.}),
                  "actual insertion did not recreate expected suffix");
            check(g.gnsslist_.size()==1 && g.gnsslist_.front().time==103.5,"GNSS fix lost");
            check(g.gnsslist_.front().blh==g.gnss_.blh && g.gnsslist_.front().std==g.gnss_.std,
                  "GNSS value/weight changed");
            const std::vector<double*> expected{g.at(109.).pose,g.at(104.).mix,
                g.extrinsic_+7,g.at(109.).mix,g.at(101.).pose,g.extrinsic_};
            check(g.last_marginalization_parameter_blocks_==expected,"suffix prior identity/order changed");
            check(old_pose!=reinterpret_cast<std::uintptr_t>(g.at(109.).pose) ||
                  old_mix!=reinterpret_cast<std::uintptr_t>(g.at(104.).mix),
                  "fixture did not exercise a changed suffix address");
            state_values_unchanged(g);
            for (size_t i=0; i<expected.size(); ++i)
                g.last_marginalization_parameter_blocks_[i][0]=8000.+i;
            check(g.at(109.).pose[0]==8000. && g.at(109.).mix[0]==8003.,"suffix prior write changed identity");
            check(g.at(104.).mix[0]==8001. && g.at(101.).pose[0]==8004.,"mixed parameter ordering changed");
            check(g.extrinsic_[7]==8002. && g.extrinsic_[0]==8005.,"suffix changed extrinsic pointers");
            check(g.at(103.5).pose[0]==1035.,"prior overwrote newly inserted GNSS state");
        } else if (mode=="no_prior_cleanup") {
            g.unused_time_nodes_={102.,104.};
            check(g.removeUnusedTimeNode(),"no-prior cleanup did not run");
            check(g.timelist_==std::deque<double>({101.,103.,105.,106.,107.,108.,109.}),"no-prior cleanup changed");
            check(g.last_marginalization_parameter_blocks_.empty(),"cleanup invented prior blocks");
            state_values_unchanged(g);
        } else if (mode=="unknown_binding") {
            double foreign[7]{};
            g.last_marginalization_parameter_blocks_={g.at(101.).pose,g.extrinsic_,foreign};
            const auto blocks=g.last_marginalization_parameter_blocks_;
            expect_guard([&] { g.capturePriorStateBindings(); },"exactly one live state");
            check(g.last_marginalization_parameter_blocks_==blocks,"failed capture modified prior");
            state_values_unchanged(g);
        } else if (mode=="missing_binding") {
            g.last_marginalization_parameter_blocks_={g.at(101.).pose,g.at(109.).mix,g.extrinsic_};
            const auto bindings=g.capturePriorStateBindings();
            const auto blocks=g.last_marginalization_parameter_blocks_;
            // Force the first binding to need remapping, then remove a later
            // binding. restore must fail without committing even its first edit.
            g.statedatalist_.erase(g.statedatalist_.begin()+1);
            g.timelist_.erase(g.timelist_.begin()+1);
            g.preintegrationlist_.erase(g.preintegrationlist_.begin()+1);
            g.statedatalist_.pop_back();g.timelist_.pop_back();g.preintegrationlist_.pop_back();
            expect_guard([&] { g.restorePriorStateBindings(bindings); },"removed or duplicated");
            check(g.last_marginalization_parameter_blocks_==blocks,"failed restore partially committed prior");
            state_values_unchanged(g);
        } else if (mode=="duplicate_binding") {
            g.last_marginalization_parameter_blocks_={g.at(101.).pose,g.at(109.).mix};
            const auto bindings=g.capturePriorStateBindings();
            const auto blocks=g.last_marginalization_parameter_blocks_;
            g.statedatalist_.back().time=101.;
            expect_guard([&] { g.restorePriorStateBindings(bindings); },"removed or duplicated");
            check(g.last_marginalization_parameter_blocks_==blocks,"duplicate binding partially committed prior");
            g.statedatalist_.back().time=109.;
        } else if (mode=="changed_parameter_count") {
            g.last_marginalization_parameter_blocks_={g.at(101.).pose,g.at(109.).mix};
            const auto bindings=g.capturePriorStateBindings();
            g.last_marginalization_parameter_blocks_.pop_back();
            const auto blocks=g.last_marginalization_parameter_blocks_;
            expect_guard([&] { g.restorePriorStateBindings(bindings); },"parameter order changed");
            check(g.last_marginalization_parameter_blocks_==blocks,"invalid block index modified prior");
        } else {
            throw std::runtime_error("unknown test case");
        }
        consistent(g);
        std::cout << "PASS " << mode << "\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << "\n";
        return 2;
    }
}
'''

# Self-contained copy of the original real-type deque reproducer. Keeping it
# here avoids depending on a developer's temporary path for future verification.
NEGATIVE_CONTROL = r'''
#include "preintegration/integration_state.h"
#include <deque>
#include <cstdio>
int main() {
    std::deque<IntegrationStateData> states;
    for (int i=0; i<10; ++i) {
        IntegrationStateData s{};
        s.time=100.+i;
        s.pose[0]=i;
        states.push_back(s);
    }
    states.pop_front();
    double *retained_prior_pose=states.front().pose;
    std::fprintf(stderr,"retained time %.0f, prior pointer %p\n",states.front().time,(void*)retained_prior_pose);
    states.erase(states.begin()+1);
    std::fprintf(stderr,"retained time %.0f, current pointer %p\n",states.front().time,(void*)states.front().pose);
    retained_prior_pose[0]=42.;
    return 0;
}
'''

TEST_CASES = ['middle_removal', 'prior_supported_node', 'suffix_recreation',
              'no_prior_cleanup', 'unknown_binding', 'missing_binding',
              'duplicate_binding', 'changed_parameter_count']


def main():
    project = Path(__file__).resolve().parents[1]
    source_dir = project/'upstream/ic_gvins/ic_gvins'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, help='fresh directory for all validation evidence')
    parser.add_argument('--source', type=Path, default=source_dir/'ic_gvins.cc')
    parser.add_argument('--negative-source', type=Path,
                        help='optional external original deque reproducer; default is the embedded copy')
    args = parser.parse_args()
    compiler = shutil.which(os.environ.get('CXX', 'c++'))
    if not compiler:
        parser.error('A C++ compiler with AddressSanitizer and UndefinedBehaviorSanitizer is required')
    eigen = Path(os.environ.get('CONDA_PREFIX', '/usr'))/'include/eigen3'
    if not (eigen/'Eigen/Geometry').is_file():
        parser.error('Eigen headers missing; activate projects/IC-GVINS/env.sh')
    out = args.out.resolve() if args.out else Path(tempfile.mkdtemp(prefix='icgvins-prior-pointers-'))
    if args.out:
        if out.exists(): parser.error('--out must be a fresh directory')
        out.mkdir(parents=True)
    print(f'Evidence directory: {out}', flush=True)
    source = args.source.read_text()
    start = source.index('std::vector<GVINS::PriorStateBinding> GVINS::capturePriorStateBindings() const {')
    end = source.index('\nbool GVINS::addNewTimeNode(double time)', start)
    header_path = source_dir/'ic_gvins.h'
    header = header_path.read_text()
    declaration_start = header.index('    struct PriorStateBinding {')
    declaration_end = header.index('\n    };', declaration_start)+len('\n    };')
    fixture = FIXTURE.replace('    /* ACTUAL_BINDING_DECLARATION */',
                              header[declaration_start:declaration_end])
    cpp = out/'prior_state_pointers.cpp'
    cpp.write_text(fixture+source[start:end]+CASES)
    negative_cpp = out/'pre_fix_deque_uaf.cpp'
    negative_cpp.write_text(args.negative_source.read_text() if args.negative_source else NEGATIVE_CONTROL)
    flags = ['-std=c++14', '-O1', '-g', '-D_GLIBCXX_ASSERTIONS',
             '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
             '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
             '-I'+str(source_dir), '-I'+str(eigen)]
    commands = []
    for name, source_file in [('prior_state_pointers', cpp), ('pre_fix_deque_uaf', negative_cpp)]:
        command = [compiler, *flags, str(source_file), '-o', str(out/name)]
        commands.append(command)
        result = subprocess.run(command, capture_output=True, text=True)
        (out/(name+'_build.log')).write_text(result.stdout+result.stderr)
        if result.returncode:
            raise SystemExit(f'Compilation failed; see {out/(name+"_build.log")}')
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:halt_on_error=1:abort_on_error=0:exitcode=86',
               UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1:exitcode=87')
    results = {}
    for case in TEST_CASES:
        result = subprocess.run([str(out/'prior_state_pointers'), case], capture_output=True,
                                text=True, timeout=15, env=env)
        output = result.stdout+result.stderr
        (out/(case+'.log')).write_text(output)
        passed = result.returncode == 0 and f'PASS {case}' in output
        results[case] = dict(returncode=result.returncode, passed=passed, log=case+'.log')
        print(f'{case}: {"PASS" if passed else "FAIL"} (returncode={result.returncode})', flush=True)
    negative = subprocess.run([str(out/'pre_fix_deque_uaf')], capture_output=True,
                              text=True, timeout=15, env=env)
    negative_output = negative.stdout+negative.stderr
    (out/'pre_fix_asan.log').write_text(negative_output)
    reproduced = negative.returncode != 0 and 'AddressSanitizer: heap-use-after-free' in negative_output
    print(f'Pre-fix negative control: {"reproduced heap-use-after-free" if reproduced else "FAILED"}', flush=True)
    report = dict(passed=all(r['passed'] for r in results.values()) and reproduced,
        cases=results, negative_control=dict(reproduced_heap_use_after_free=reproduced,
            returncode=negative.returncode, log='pre_fix_asan.log'),
        compiler=compiler, compile_commands=commands,
        sanitizer_environment={key:env[key] for key in ('ASAN_OPTIONS','UBSAN_OPTIONS')},
        actual_functions=['capturePriorStateBindings', 'restorePriorStateBindings',
            'removeUnusedTimeNode', 'insertNewGnssTimeNode', 'addNewGnssTimeNode'],
        limits='Fixture supplies numeric propagation and map/IMU plumbing; this checks memory lifetime and logical state identity, not replay accuracy.')
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if not report['passed']:
        raise SystemExit(f'FAILED; see {out}/report.json and per-case logs')
    print(f'PASS all prior-pointer checks; report: {out}/report.json')


if __name__ == '__main__': main()
