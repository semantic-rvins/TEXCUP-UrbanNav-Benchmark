#!/usr/bin/env python3
"""Compile GVINS timing code without ROS, then exercise clock/reset regressions.

Run: python projects/GVINS/tests/test_timing.py [--upstream PATH]
Requires g++, Eigen 3 headers, Ceres, and glog development libraries.
The initializer source/header are copied unchanged except for their unrelated
feature-manager include; the GNSS factor source/header are copied unchanged.
Small GNSS adapters supply deterministic calibration/residual problems.
Reset methods, clock-seeding statements, and factor-construction blocks are
extracted verbatim from estimator.cpp/estimator_node.cpp; surrounding estimator
and ROS objects are test doubles. This does not replace a full ROS build/replay.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


def block(source, marker):
    """Return a declaration/statement and its complete brace-delimited body."""
    start = source.index(marker)
    # The constructor also has a braced member initializer on its first line.
    opening = start + re.search(r"\n[ \t]*\{", source[start:]).end() - 1
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


GNSS_STUB = r"""
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>
namespace gnss_comm {
struct gtime_t { double seconds; };
constexpr unsigned SYS_GPS=1, SYS_GLO=4, SYS_GAL=8;
constexpr double LIGHT_SPEED=299792458.0, EARTH_OMG_GPS=7.2921151467e-5;
constexpr double TEST_FREQ=1575420000.0;
struct Obs {
    gtime_t time; Eigen::Matrix<double, 1, 7> row; double measured;
    unsigned sat=SYS_GPS;
    std::vector<double> psr{0.0}, dopp{0.0}, psr_std{0.16}, dopp_std{0.256};
};
struct EphemBase { virtual ~EphemBase()=default; };
struct Ephem : EphemBase {
    double ura=2.0, tgd[2]={0.0,0.0};
    Eigen::Vector3d pos{20000,16000,24000}, vel{50,30,-10};
};
struct GloEphem : Ephem {};
struct SatState {};
using ObsPtr = std::shared_ptr<Obs>;
using EphemBasePtr = std::shared_ptr<EphemBase>;
using EphemPtr = std::shared_ptr<Ephem>;
using GloEphemPtr = std::shared_ptr<GloEphem>;
using SatStatePtr = std::shared_ptr<SatState>;
inline double time2sec(gtime_t time) { return time.seconds; }
inline gtime_t time_add(gtime_t time,double dt) { return {time.seconds+dt}; }
inline unsigned satsys(unsigned sat,void*) { return sat; }
inline double L1_freq(const ObsPtr&,int* idx) { if(idx) *idx=0; return TEST_FREQ; }
inline double eph2svdt(gtime_t,const EphemPtr&) { return 0.0; }
inline double geph2svdt(gtime_t,const GloEphemPtr&) { return 0.0; }
inline Eigen::Vector3d eph2pos(gtime_t,const EphemPtr& eph,double* dt) {
    if(dt) *dt=0.0; return eph->pos;
}
inline Eigen::Vector3d eph2vel(gtime_t,const EphemPtr& eph,double* dt) {
    if(dt) *dt=0.0; return eph->vel;
}
inline Eigen::Vector3d geph2pos(gtime_t,const GloEphemPtr& eph,double* dt) {
    if(dt) *dt=0.0; return eph->pos;
}
inline Eigen::Vector3d geph2vel(gtime_t,const GloEphemPtr& eph,double* dt) {
    if(dt) *dt=0.0; return eph->vel;
}
inline void sat_azel(const Eigen::Vector3d&,const Eigen::Vector3d&,double* azel) {
    azel[0]=0.0; azel[1]=M_PI/2;
}
inline Eigen::Vector3d ecef2geo(const Eigen::Vector3d&) { return Eigen::Vector3d::Zero(); }
inline double calculate_trop_delay(gtime_t,const Eigen::Vector3d&,double*) { return 0.0; }
inline double calculate_ion_delay(gtime_t,const std::vector<double>&,
    const Eigen::Vector3d&,double*) { return 0.0; }
inline Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d&) {
    return Eigen::Matrix3d::Identity();
}
inline std::vector<SatStatePtr> sat_states(const std::vector<ObsPtr>& obs,
                                          const std::vector<EphemBasePtr>&) {
    return std::vector<SatStatePtr>(obs.size());
}
inline Eigen::Matrix<double, 7, 1> psr_pos(const std::vector<ObsPtr>&,
    const std::vector<EphemBasePtr>&, const std::vector<double>&) {
    return Eigen::Matrix<double, 7, 1>::Ones();
}
inline void dopp_res(const Eigen::Vector4d&, const Eigen::Vector3d&,
    const std::vector<ObsPtr>& obs, const std::vector<SatStatePtr>&,
    Eigen::VectorXd& res, Eigen::MatrixXd& J) {
    res = Eigen::VectorXd::Zero(obs.size());
    J = Eigen::MatrixXd::Zero(obs.size(), 4);
}
inline void psr_res(const Eigen::Matrix<double, 7, 1>& state,
    const std::vector<ObsPtr>& obs, const std::vector<SatStatePtr>&,
    const std::vector<double>&, Eigen::VectorXd& res, Eigen::MatrixXd& J,
    std::vector<Eigen::Vector2d>&, std::vector<Eigen::Vector2d>&) {
    res.resize(obs.size()); J.resize(obs.size(), 7);
    for (size_t i = 0; i < obs.size(); ++i) {
        J.row(i) = obs[i]->row;
        res(i) = obs[i]->row.dot(state) - obs[i]->measured;
    }
}
}
"""


HARNESS = r"""
#include "gnss_vi_initializer.h"
#include "gnss_psr_dopp_factor.hpp"
#include <array>
#include <map>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::Vector3d;
constexpr int WINDOW_SIZE = 10, NUM_OF_CAM = 1;
constexpr double TD = 0.0, FOCAL_LENGTH = 460.0;
const std::vector<double> GNSS_IONO_DEFAULT_PARAMS(8, 0.0);
const Vector3d TIC[NUM_OF_CAM] = {Vector3d::Zero()};
const Matrix3d RIC[NUM_OF_CAM] = {Matrix3d::Identity()};
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
struct ProjectionFactor { inline static Matrix2d sqrt_info; };
struct ProjectionTdFactor { inline static Matrix2d sqrt_info; };
struct IntegrationBase {};
struct MarginalizationInfo {};
struct ImageFrame { IntegrationBase *pre_integration = nullptr; };
struct FeatureManager {
    explicit FeatureManager(Matrix3d*) {}
    void clearState() {}
    void setRic(Matrix3d*) {}
};
struct Stamp { double seconds = 0; double toSec() const { return seconds; } };
struct Header { Stamp stamp; };
class Estimator {
public:
    Estimator();
    void clearState();
    void setParameter();
    void inputGNSSTimeDiff(double t_diff);
    bool failureDetection() { return true; }
    void fail() { FAILURE_BLOCK }
    double clockReference() {
        REFERENCE_STATEMENT
        return clock_reference_gpst;
    }
    void seed(const Eigen::Matrix<double,7,1>& rough_xyzt,
              const Eigen::Matrix<double,7,1>& refined_xyzt,
              double aligned_rcv_ddt) { SEED_BLOCK }
    std::unique_ptr<GnssPsrDoppFactor> normalFactor(int i, const ObsPtr& obs,
        const EphemBasePtr& eph, int& position_lower_index) {
        const std::vector<ObsPtr> curr_obs{obs};
        const std::vector<EphemBasePtr> curr_ephem{eph};
        const unsigned j=0;
        NORMAL_FACTOR_BLOCK
        position_lower_index=lower_idx;
        return std::unique_ptr<GnssPsrDoppFactor>(gnss_factor);
    }
    std::unique_ptr<GnssPsrDoppFactor> marginalFactor(const ObsPtr& obs,
        const EphemBasePtr& eph) {
        const std::vector<std::vector<ObsPtr>> gnss_meas_buf{{obs}};
        const std::vector<std::vector<EphemBasePtr>> gnss_ephem_buf{{eph}};
        const unsigned j=0;
        MARGINAL_FACTOR_BLOCK
        return std::unique_ptr<GnssPsrDoppFactor>(gnss_factor);
    }
    Matrix3d Rs[WINDOW_SIZE+1], ric[NUM_OF_CAM], R_ecef_enu;
    Vector3d Ps[WINDOW_SIZE+1], Vs[WINDOW_SIZE+1], Bas[WINDOW_SIZE+1],
             Bgs[WINDOW_SIZE+1], tic[NUM_OF_CAM], anc_ecef;
    FeatureManager f_manager;
    std::vector<double> dt_buf[WINDOW_SIZE+1];
    std::vector<Vector3d> linear_acceleration_buf[WINDOW_SIZE+1],
                          angular_velocity_buf[WINDOW_SIZE+1];
    IntegrationBase *pre_integrations[WINDOW_SIZE+1];
    IntegrationBase *tmp_pre_integration = nullptr;
    MarginalizationInfo *last_marginalization_info = nullptr;
    std::vector<void*> last_marginalization_parameter_blocks;
    std::map<double, ImageFrame> all_image_frame;
    std::map<int,int> sat_track_status, sat2ephem, sat2time_index;
    std::vector<double> latest_gnss_iono_params;
    enum SolverFlag { INITIAL, NON_LINEAR };
    SolverFlag solver_flag;
    bool first_imu, gnss_ready, first_optimization;
    int sum_of_back, sum_of_front, frame_count, failure_occur;
    double initial_timestamp, td, para_yaw_enu_local[1], yaw_enu_local;
    OFFSET_DECLARATION
    Header Headers[WINDOW_SIZE+1];
    double para_rcv_dt[4*(WINDOW_SIZE+1)], para_rcv_ddt[WINDOW_SIZE+1];
};
METHODS
namespace std_msgs {
struct Bool { bool data; };
using BoolConstPtr = std::shared_ptr<const Bool>;
}
std::mutex m_buf, m_estimator;
std::queue<int> feature_buf, imu_buf;
std::unique_ptr<Estimator> estimator_ptr;
double current_time = 0, last_imu_t = 0;
RESTART_CALLBACK
void expectNear(double actual, double expected, double tol = 1e-9) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tol)
        throw std::runtime_error("actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected));
}
void testResets() {
    estimator_ptr = std::make_unique<Estimator>();
    expectNear(estimator_ptr->diff_t_gnss_local, 0.0);
    estimator_ptr->clearState(); // Before online calibration exists.
    expectNear(estimator_ptr->diff_t_gnss_local, 0.0);
    for (double offset : {18.0, 17.321, -0.125, 0.0}) {
        estimator_ptr->inputGNSSTimeDiff(offset);
        estimator_ptr->fail(); // Real internal failure reset branch.
        expectNear(estimator_ptr->diff_t_gnss_local, offset);
        restart_callback(std::make_shared<std_msgs::Bool>(std_msgs::Bool{true}));
        expectNear(estimator_ptr->diff_t_gnss_local, offset);
        // A fresh online pulse can still replace the retained calibration.
        estimator_ptr->inputGNSSTimeDiff(offset + 0.25);
        estimator_ptr->fail();
        expectNear(estimator_ptr->diff_t_gnss_local, offset + 0.25);
    }
}
void testSeed() {
    Estimator estimator;
    const double local_origin = 1557425380.125;
    for (int i = 0; i <= WINDOW_SIZE; ++i)
        estimator.Headers[i].stamp.seconds = local_origin + 0.1*i + 0.013*i*i;
    estimator.inputGNSSTimeDiff(18.0);
    expectNear(estimator.clockReference(), local_origin+18.0);
    Eigen::Matrix<double,7,1> rough, refined;
    rough << 1, 2, 3, 50, 0, 150, 200; // One unobserved constellation.
    refined << 1, 2, 3, 53, 0, 153, 203;
    for (double drift : {2.5, -3.25, 0.0}) {
        estimator.seed(rough, refined, drift);
        for (int i = 0; i <= WINDOW_SIZE; ++i) {
            const double dt = estimator.Headers[i].stamp.seconds-local_origin;
            expectNear(estimator.para_rcv_ddt[i], drift);
            expectNear(estimator.para_rcv_dt[4*i], 53+drift*dt);
            expectNear(estimator.para_rcv_dt[4*i+1], 53+drift*dt);
            expectNear(estimator.para_rcv_dt[4*i+2], 153+drift*dt);
            expectNear(estimator.para_rcv_dt[4*i+3], 203+drift*dt);
        }
    }
}
void testRefinement() {
    const double reference = 1557425398.125;
    const Vector3d true_anchor(1, 2, 3);
    Eigen::Vector4d true_clock; true_clock << 50, 100, 150, 200;
    // Sparse first/last slots and irregular timing must not affect the clock
    // reference. Reindex the SAME physical epochs in a second window.
    for (const std::vector<int>& indices : {std::vector<int>{1,6,10},
                                          std::vector<int>{0,1,2}}) {
        for (double drift : {2.5, -3.25, 0.0}) {
            std::vector<std::vector<ObsPtr>> meas(11);
            std::vector<std::vector<EphemBasePtr>> eph(11);
            std::vector<Vector3d> poses(11, Vector3d::Zero());
            const std::vector<double> elapsed = {-0.07, 1.37, 3.02};
            for (size_t epoch=0; epoch<indices.size(); ++epoch) {
                const int i = indices[epoch];
                poses[i] = Vector3d(0.3*epoch, -0.1*epoch, 0.2);
                const double timestamp = reference+elapsed[epoch];
                Eigen::Matrix<double,7,1> truth;
                truth.head<3>() = true_anchor+poses[i];
                truth.tail<4>() = true_clock +
                    drift*(timestamp-reference)*Eigen::Vector4d::Ones();
                for (int row=0; row<7; ++row) {
                    auto obs = std::make_shared<Obs>();
                    obs->time.seconds = timestamp;
                    obs->row.setZero();
                    if (row<3) { obs->row(row)=1; obs->row(3)=1; }
                    else obs->row(row)=1;
                    obs->measured = obs->row.dot(truth);
                    meas[i].push_back(obs);
                    eph[i].push_back(std::make_shared<EphemBase>());
                }
            }
            const std::vector<double> iono(8, 0.0);
            GNSSVIInitializer initializer(meas, eph, iono, reference);
            Eigen::Matrix<double,7,1> rough, result;
            rough << 10, -4, 0, 10, 10, 10, 10;
            if (!initializer.anchor_refinement(poses, 0.0, drift, rough, result))
                throw std::runtime_error("anchor refinement failed");
            for (int i=0;i<3;++i) expectNear(result(i),true_anchor(i),1e-8);
            for (int i=0;i<4;++i) expectNear(result(i+3),true_clock(i),1e-8);
        }
    }
}
void testFactorClockConnection() {
    Estimator estimator;
    const double origin=1557425380.125;
    estimator.inputGNSSTimeDiff(18.0);
    for(int i=0;i<=WINDOW_SIZE;++i)
        estimator.Headers[i].stamp.seconds=origin+0.1*i+0.013*i*i;
    Eigen::Matrix<double,7,1> rough, refined;
    rough << 1,2,3,75,0,100,150;
    refined=rough;
    const Vector3d anchor(1000,2000,3000), velocity(3,-2,0.4);
    auto eph=std::make_shared<Ephem>();
    for(double drift : {2.5,-3.25,0.0}) {
        estimator.seed(rough,refined,drift);
        for(int clock_index : {0,4,10}) {
            for(double offset : {-0.12,0.0,0.11}) {
                auto obs=std::make_shared<Obs>();
                obs->time.seconds=estimator.Headers[clock_index].stamp.seconds+18.0+offset;
                const double observation_utc=obs->time.seconds-18.0;
                const double dt=observation_utc-estimator.Headers[clock_index].stamp.seconds;
                const Vector3d receiver=anchor+velocity*(observation_utc-origin);
                const Vector3d los=(eph->pos-receiver).normalized();
                const double earth_term=EARTH_OMG_GPS/LIGHT_SPEED;
                const double code_earth=earth_term*(eph->pos.x()*receiver.y()-
                                                  eph->pos.y()*receiver.x());
                const double rate_earth=earth_term*(eph->vel.x()*receiver.y()+
                    eph->pos.x()*velocity.y()-eph->vel.y()*receiver.x()-
                    eph->pos.y()*velocity.x());
                // Independent receiver trajectory and clock law at GNSS time.
                obs->psr[0]=(eph->pos-receiver).norm()+code_earth+
                    75.0+drift*(observation_utc-origin);
                obs->dopp[0]=-((eph->vel-velocity).dot(los)+rate_earth+drift)/
                    (LIGHT_SPEED/TEST_FREQ);
                for(bool marginal : {false,true}) {
                    if(marginal && clock_index!=0) continue;
                    int lower=0;
                    auto factor=marginal ? estimator.marginalFactor(obs,eph) :
                        estimator.normalFactor(clock_index,obs,eph,lower);
                    std::array<double,7> pi{},pj{};
                    std::array<double,9> vi{},vj{};
                    for(int axis=0;axis<3;++axis) {
                        pi[axis]=velocity[axis]*(estimator.Headers[lower].stamp.seconds-origin);
                        pj[axis]=velocity[axis]*(estimator.Headers[lower+1].stamp.seconds-origin);
                        vi[axis]=vj[axis]=velocity[axis];
                    }
                    double bias=estimator.para_rcv_dt[4*clock_index];
                    double rate=estimator.para_rcv_ddt[clock_index], yaw=0.0;
                    const double* parameters[8]={pi.data(),vi.data(),pj.data(),vj.data(),
                        &bias,&rate,&yaw,anchor.data()};
                    double residuals[2], bias_jac[2],rate_jac[2];
                    double* jacobians[8]={nullptr,nullptr,nullptr,nullptr,
                                         bias_jac,rate_jac,nullptr,nullptr};
                    factor->Evaluate(parameters,residuals,jacobians);
                    expectNear(residuals[0],0.0,1e-6);
                    expectNear(residuals[1],0.0,1e-6);
                    // Stub elevation=90 deg, URA=2, nominal measurement stds.
                    expectNear(bias_jac[0],10.0);
                    expectNear(bias_jac[1],0.0);
                    expectNear(rate_jac[0],10.0*dt);
                    expectNear(rate_jac[1],50.0);
                    const double epsilon=0.001;
                    for(int column=0;column<2;++column) {
                        double& value=column==0 ? bias : rate;
                        const double original=value;
                        double plus[2],minus[2];
                        value=original+epsilon; factor->Evaluate(parameters,plus,nullptr);
                        value=original-epsilon; factor->Evaluate(parameters,minus,nullptr);
                        value=original;
                        for(int row=0;row<2;++row)
                            expectNear((plus[row]-minus[row])/(2*epsilon),
                                column==0 ? bias_jac[row] : rate_jac[row],1e-4);
                    }
                }
            }
        }
    }
}
int main(int argc, char** argv) {
    try {
        if (argc!=2) return 2;
        const std::string test=argv[1];
        if (test=="resets") testResets();
        else if (test=="seed") testSeed();
        else if (test=="refinement") testRefinement();
        else if (test=="factor") testFactorClockConnection();
        else return 2;
        std::cout << test << " passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
"""


class TimingRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="gvins-timing-")
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        source_dir = UPSTREAM / "estimator/src"
        estimator = (source_dir / "estimator.cpp").read_text()
        estimator_header = (source_dir / "estimator.h").read_text()
        node = (source_dir / "estimator_node.cpp").read_text()
        initializer_header = (source_dir / "initial/gnss_vi_initializer.h").read_text()
        initializer_header = initializer_header.replace('#include "../feature_manager.h"', "")
        (directory / "gnss_vi_initializer.h").write_text(initializer_header)
        (directory / "gnss_vi_initializer.cpp").write_text(
            (source_dir / "initial/gnss_vi_initializer.cpp").read_text())
        for name in ("gnss_psr_dopp_factor.hpp", "gnss_psr_dopp_factor.cpp"):
            (directory / name).write_text((source_dir / "factor" / name).read_text())
        (directory / "gnss_comm").mkdir()
        for name in ("gnss_utility.hpp", "gnss_spp.hpp", "gnss_constant.hpp"):
            (directory / "gnss_comm" / name).write_text(
                GNSS_STUB if name == "gnss_utility.hpp" else '#include "gnss_utility.hpp"\n')
        seed_start = estimator.index("// restore GNSS states")
        seed_end = estimator.index("    anc_ecef =", seed_start)
        normal_start = estimator.index("                int lower_idx = -1;")
        normal_end = estimator.index("                problem.AddResidualBlock(gnss_factor", normal_start)
        marginal_start = estimator.index(
            "                const double obs_local_ts = time2sec(gnss_meas_buf[0][j]->time)")
        marginal_end = estimator.index("                ResidualBlockInfo *psr_dopp", marginal_start)
        replacements = {
            "FAILURE_BLOCK": block(estimator, "if (failureDetection())"),
            "REFERENCE_STATEMENT": re.search(
                r"const double clock_reference_gpst\s*=.*?;", estimator).group(),
            "SEED_BLOCK": estimator[seed_start:seed_end],
            "OFFSET_DECLARATION": re.search(
                r"double diff_t_gnss_local[^;]*;", estimator_header).group(),
            "NORMAL_FACTOR_BLOCK": estimator[normal_start:normal_end],
            "MARGINAL_FACTOR_BLOCK": estimator[marginal_start:marginal_end],
            "METHODS": "\n".join(block(estimator, marker) for marker in (
                "Estimator::Estimator()", "void Estimator::clearState()",
                "void Estimator::setParameter()", "void Estimator::inputGNSSTimeDiff(")),
            "RESTART_CALLBACK": block(node, "void restart_callback("),
        }
        harness = HARNESS
        for marker, replacement in replacements.items():
            harness = harness.replace(marker, replacement)
        (directory / "timing.cpp").write_text(harness)
        cls.executable = directory / "timing"
        result = subprocess.run([
            "g++", "-std=c++17", "-O1", "-Wall", "-Wextra",
            "-I" + str(directory), "-I/usr/include/eigen3",
            str(directory / "timing.cpp"), str(directory / "gnss_vi_initializer.cpp"),
            str(directory / "gnss_psr_dopp_factor.cpp"),
            "-lceres", "-lglog", "-o", str(cls.executable),
        ], text=True, capture_output=True)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def run_case(self, case):
        result = subprocess.run([str(self.executable), case], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_fixed_and_online_offsets_survive_both_reset_routes(self):
        self.run_case("resets")

    def test_header_time_clock_seeds_and_unobserved_system(self):
        self.run_case("seed")

    def test_sparse_irregular_gnss_epochs_are_invariant_to_frame_reindexing(self):
        self.run_case("refinement")

    def test_clock_seeds_reach_observation_time_with_correct_factor_jacobians(self):
        self.run_case("factor")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--upstream", type=Path,
                        default=Path(__file__).resolve().parents[1] / "upstream")
    args, rest = parser.parse_known_args()
    UPSTREAM = args.upstream.resolve()
    unittest.main(argv=[__file__] + rest)
else:
    UPSTREAM = Path(__file__).resolve().parents[1] / "upstream"
