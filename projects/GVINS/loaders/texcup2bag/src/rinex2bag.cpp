/*
 * TEX-CUP RINEX -> gnss_comm rosbag converter for GVINS.
 *
 * Reads:
 *   - RINEX 3.04 rover observation file (Septentrio AsteRx4)
 *   - RINEX 3.0x multi-GNSS broadcast navigation file (header must say 3.04,
 *     patch the version field if needed; format is identical for our records)
 * Writes a rosbag with:
 *   - /ublox_driver/range_meas   gnss_comm/GnssMeasMsg   (1 Hz epochs)
 *   - /ublox_driver/ephem        gnss_comm/GnssEphemMsg  (all GPS/GAL/BDS eph, dumped at bag start)
 *   - /ublox_driver/iono_params  gnss_comm/StampedFloat64Array (Klobuchar alpha/beta, repeated)
 *
 * Bag record timestamps are UTC unix seconds (= GPST - leap). GVINS must be
 * configured with gnss_local_online_sync: 0 and gnss_local_time_diff: <leap>.
 *
 * GLONASS is dropped: gnss_comm's rinex2obs has no GLONASS slot handling
 * (TODO in upstream) so FDMA frequencies would be wrong.
 *
 * Pseudorange/doppler stds are synthesized (RINEX carries none): GVINS's
 * gnss_psr_dopp_factor scales pr_uura = 2*(psr_std/0.16) and FATALs on 0,
 * so we set psr_std=0.16 m, dopp_std=0.256 Hz => their nominal weight.
 *
 * Usage: rinex2bag <nav_file> <obs_file> <out_bag> <start_utc_unix> <leap_sec>
 */
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ros/time.h>
#include <rosbag/bag.h>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/StampedFloat64Array.h>

using namespace gnss_comm;

static bool parse_iono(const std::string &nav_path, std::vector<double> &iono)
{
    std::ifstream f(nav_path);
    std::string line;
    std::vector<double> alpha, beta;
    while (std::getline(f, line))
    {
        if (line.find("END OF HEADER") != std::string::npos) break;
        if (line.find("IONOSPHERIC CORR") == std::string::npos) continue;
        std::string label = line.substr(0, 4);
        std::string body = line.substr(5, 55);
        for (auto &c : body) if (c == 'D' || c == 'd') c = 'E';
        std::istringstream iss(body);
        std::vector<double> vals;
        double v;
        while (iss >> v) vals.push_back(v);
        if (vals.size() < 4) continue;
        if (label == "GPSA") alpha.assign(vals.begin(), vals.begin() + 4);
        if (label == "GPSB") beta.assign(vals.begin(), vals.begin() + 4);
    }
    if (alpha.size() == 4 && beta.size() == 4)
    {
        iono = alpha;
        iono.insert(iono.end(), beta.begin(), beta.end());
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        fprintf(stderr, "usage: rinex2bag <nav> <obs> <out_bag> <start_utc_unix> <leap_sec>\n");
        return 1;
    }
    const std::string nav_path(argv[1]), obs_path(argv[2]), bag_path(argv[3]);
    const double start_utc = atof(argv[4]);
    const double leap = atof(argv[5]);

    std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem;
    rinex2ephems(nav_path, sat2ephem);
    fprintf(stderr, "parsed ephemerides for %zu satellites\n", sat2ephem.size());
    if (sat2ephem.empty()) { fprintf(stderr, "no ephemeris parsed, abort\n"); return 2; }

    std::vector<std::vector<ObsPtr>> rinex_meas;
    rinex2obs(obs_path, rinex_meas);
    fprintf(stderr, "parsed %zu obs epochs\n", rinex_meas.size());
    if (rinex_meas.empty()) { fprintf(stderr, "no obs parsed, abort\n"); return 3; }

    std::vector<double> iono;
    if (!parse_iono(nav_path, iono))
        fprintf(stderr, "WARNING: no GPSA/GPSB iono params found in nav header\n");
    else
    {
        fprintf(stderr, "iono params:");
        for (double v : iono) fprintf(stderr, " %g", v);
        fprintf(stderr, "\n");
    }

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Write);

    const double t0 = start_utc - 8.0;   // ephem+iono dump slot before first obs

    // ephemerides (GPS/GAL/BDS only; rinex2ephems already only parses G/C/E/R,
    // and we skip GLONASS records)
    size_t n_eph = 0;
    double t_eph = t0;
    for (const auto &kv : sat2ephem)
    {
        for (const auto &base : kv.second)
        {
            EphemPtr eph = std::dynamic_pointer_cast<Ephem>(base);
            if (!eph) continue;   // GLONASS -> skip
            GnssEphemMsg msg = ephem2msg(eph);
            bag.write("/ublox_driver/ephem", ros::Time(t_eph), msg);
            t_eph += 0.001;
            ++n_eph;
        }
    }
    fprintf(stderr, "wrote %zu ephemeris messages\n", n_eph);

    // observations
    size_t n_epoch = 0;
    double last_utc = 0, first_utc = 0, last_iono_utc = 0;
    for (const auto &epoch : rinex_meas)
    {
        if (epoch.empty()) continue;
        const double t_gpst = time2sec(epoch[0]->time);
        const double t_utc = t_gpst - leap;
        if (t_utc < start_utc) continue;

        std::vector<ObsPtr> keep;
        for (const auto &obs : epoch)
        {
            const uint32_t sys = satsys(obs->sat, NULL);
            if (sys != SYS_GPS && sys != SYS_GAL && sys != SYS_BDS) continue;
            if (sat2ephem.count(obs->sat) == 0) continue;
            for (size_t k = 0; k < obs->freqs.size(); ++k)
            {
                obs->psr_std[k] = 0.16;
                obs->dopp_std[k] = 0.256;
                obs->cp_std[k] = 0.01;
            }
            keep.push_back(obs);
        }
        if (keep.empty()) continue;

        // periodic iono params (GVINS latches the latest)
        if (!iono.empty() && (last_iono_utc == 0 || t_utc - last_iono_utc > 30.0))
        {
            StampedFloat64Array iono_msg;
            iono_msg.header.stamp = ros::Time(t_utc - 0.5);
            iono_msg.data = iono;
            bag.write("/ublox_driver/iono_params", ros::Time(t_utc - 0.5), iono_msg);
            last_iono_utc = t_utc;
        }

        GnssMeasMsg msg = meas2msg(keep);
        bag.write("/ublox_driver/range_meas", ros::Time(t_utc), msg);
        if (first_utc == 0) first_utc = t_utc;
        last_utc = t_utc;
        ++n_epoch;
    }
    bag.close();
    fprintf(stderr, "wrote %zu obs epochs, UTC unix %.1f .. %.1f\n",
            n_epoch, first_utc, last_utc);
    return 0;
}
