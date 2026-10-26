// =============================================================================
//  apps/bench_gcs.cpp   ->   the `mtlc_bench_gcs` tool
//
//  Static-stage benchmark of the seeding strategies (the C++ twin of
//  bench_gcs_static.m): for every seed, the SAME scenario is planned once per
//  variant with reallocation OFF, and the plan is scored by the full 1 m
//  evaluator.  One CSV row per (seed, variant).
//
//    ./mtlc_bench_gcs --seeds 21,1,2 --variants current,gcs-cw1 [--order-frac 0.95]
//                     [--no-residual] [--csv FILE] [--verbose]
//
//  A variant is  name[:key=val,key=val...]  where name is a preset (below) and
//  the keys override it:
//    mode=radius|kernel   init=clusters+greedy|gcs|...   drop=info|ratio
//    cw=W (GCS centre pull)   rf=F (set radius / half-width)   batch=0|1
//    frac=F (orderBudgetFrac)   realloc=0|1   stagger=M   nround=N   cuts=N
// =============================================================================
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "mtl_curve/eval/detection.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/planner.hpp"

namespace mc = mtl::curve;

namespace {

std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string t;
    while (std::getline(ss, t, d))
        if (!t.empty()) out.push_back(t);
    return out;
}

const std::map<std::string, std::string> kPresets = {
    {"current",       "mode=radius,init=clusters+greedy"},
    {"clusters",      "mode=radius,init=clusters"},
    {"greedy",        "mode=radius,init=greedy"},
    {"op-kernel",     "mode=kernel,init=clusters"},
    {"gcs",           "mode=kernel,init=gcs,drop=info,cw=0"},
    {"gcs-ratio",     "mode=kernel,init=gcs,drop=ratio,cw=0"},
    {"gcs-cw1",       "mode=kernel,init=gcs,drop=info,cw=1"},
    {"gcs-ratio-cw1", "mode=kernel,init=gcs,drop=ratio,cw=1"},
    {"gcs-radius",    "mode=radius,init=gcs,drop=info,cw=0"},
    {"gcs+greedy",    "mode=kernel,init=gcs+greedy,drop=info,cw=0"},
    {"gcs-op",        "mode=kernel,init=gcs,gmode=op,cw=0"},
    {"gcs-op-radius", "mode=radius,init=gcs,gmode=op,cw=0"},
    {"gcs-op+greedy", "mode=kernel,init=gcs+greedy,gmode=op,cw=0"},
    {"gcs-cov",       "mode=kernel,init=gcs,gmode=cov"},
    {"gcs-cov+greedy","mode=kernel,init=gcs+greedy,gmode=cov"},
    {"cov",           "mode=kernel,init=gcs,gmode=cov,own=1,cell=0.8,pt=0.5"},
    {"cov2",          "mode=kernel,init=gcs,gmode=cov,own=1,cell=0.8,pt=0.5,nseeds=2,extra=1.15/0.4"},
    {"gcs-op-team",   "mode=kernel,init=gcs,gmode=op,cw=0,resid=1,nopart=1"},
    {"gcs-op-team+greedy", "mode=kernel,init=gcs+greedy,gmode=op,cw=0,resid=1,nopart=1"},
    {"gcs-team",      "mode=kernel,init=gcs,drop=info,cw=0,resid=1,nopart=1"},
};

bool apply(const std::string& spec, mc::PlannerParams& p, std::string& err) {
    for (const std::string& kv : split(spec, ',')) {
        const auto eq = kv.find('=');
        if (eq == std::string::npos) { err = "bad key=val: " + kv; return false; }
        const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
        if (k == "mode") p.team.clusterMode = v == "kernel" ? mc::ClusterMode::Kernel : mc::ClusterMode::Radius;
        else if (k == "init") {
            p.team.initStrategies.clear();
            for (const std::string& s : split(v, '+')) {
                if (s == "clusters") p.team.initStrategies.push_back(mc::InitStrategy::Clusters);
                else if (s == "greedy") p.team.initStrategies.push_back(mc::InitStrategy::Greedy);
                else if (s == "gcs") p.team.initStrategies.push_back(mc::InitStrategy::Gcs);
                else if (s == "sector") p.team.initStrategies.push_back(mc::InitStrategy::Sector);
                else { err = "unknown init: " + s; return false; }
            }
        } else if (k == "drop") p.team.gcs.dropRule = v == "ratio" ? mc::GcsDropRule::Ratio : mc::GcsDropRule::Info;
        else if (k == "cw") p.team.gcs.centerWeight = std::stod(v);
        else if (k == "cwinfo") p.team.gcs.centerWeightByInfo = std::stoi(v) != 0;
        else if (k == "rf") p.team.gcs.setRadiusFrac = std::stod(v);
        else if (k == "batch") p.team.gcs.dropBatch = std::stoi(v) != 0;
        else if (k == "frac") p.team.orderBudgetFrac = std::stod(v);
        else if (k == "realloc") p.team.reallocate = std::stoi(v) != 0;
        else if (k == "stagger") p.team.altitudeStagger = std::stod(v);
        else if (k == "nround") p.team.gcs.nRound = std::stoi(v);
        else if (k == "cuts") p.team.gcs.maxCutRounds = std::stoi(v);
        else if (k == "kmin") p.team.kernelClusters.minInfoFrac = std::stod(v);
        else if (k == "gmode") p.team.gcs.mode = v == "op" ? mc::GcsMode::Orienteering : v == "cov" ? mc::GcsMode::Coverage : mc::GcsMode::Drop;
        else if (k == "cell") p.team.gcs.coverageCellFrac = std::stod(v);
        else if (k == "pt") p.team.gcs.coveragePointFrac = std::stod(v);
        else if (k == "nseeds") p.team.gcs.nSeeds = std::stoi(v);
        else if (k == "explore") { p.optimizer.exploreIter = std::stoi(v); p.optimizer.exploreIterWarm = std::min(p.optimizer.exploreIterWarm, std::stoi(v)); }
        else if (k == "extra") {  // extra=0.8/0.5+1.15/0.4
            for (const std::string& t : split(v, '+')) {
                const auto sl = t.find('/');
                p.team.gcs.coverageExtra.emplace_back(std::stod(t.substr(0, sl)), std::stod(t.substr(sl + 1)));
            }
        }
        else if (k == "mu") p.team.corridorMu = std::stod(v);
        else if (k == "muscale") p.team.corridorScale = std::stod(v);
        else if (k == "muall") p.team.corridorGcsOnly = std::stoi(v) == 0;
        else if (k == "mukeep") p.team.corridorKeep = std::stoi(v) != 0;
        else if (k == "own") p.team.gcs.coverageOwnOnly = std::stoi(v) != 0;
        else if (k == "resid") p.team.gcs.residualRewards = std::stoi(v) != 0;
        else if (k == "nopart") p.team.gcs.ignorePartition = std::stoi(v) != 0;
        else if (k == "opt") { if (std::stoi(v) == 0) { p.optimizer.maxIter = 0; p.optimizer.exploreIter = 0; p.optimizer.exploreIterWarm = 0; p.team.coordinationSweeps = 1; } }
        else if (k == "sweeps") p.team.coordinationSweeps = std::stoi(v);
        else if (k == "kbox") p.team.kernelClusters.boxFrac = std::stod(v);
        else { err = "unknown key: " + k; return false; }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<int> seeds = {21, 1, 2, 3, 4, 5};
    std::vector<std::string> variants = {"current", "gcs-cw1"};
    double orderFrac = 0.95;
    bool residual = true, verbose = false;
    std::string csv, dump;
    int numTargets = 50;
    int numAgents = 3;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) {
            seeds.clear();
            for (const std::string& s : split(argv[++i], ',')) seeds.push_back(std::stoi(s));
        } else if (a == "--variants" && i + 1 < argc) {
            variants = split(argv[++i], ';');
            if (variants.size() == 1 && variants[0].find(':') == std::string::npos) variants = split(argv[i], ',');
        } else if (a == "--order-frac" && i + 1 < argc) {
            orderFrac = std::stod(argv[++i]);
        } else if (a == "--no-residual") {
            residual = false;
        } else if (a == "--csv" && i + 1 < argc) {
            csv = argv[++i];
        } else if (a == "--agents" && i + 1 < argc) {
            numAgents = std::stoi(argv[++i]);
        } else if (a == "--dump" && i + 1 < argc) {
            dump = argv[++i];
        } else if (a == "--verbose") {
            verbose = true;
        } else {
            std::cerr << "usage: mtlc_bench_gcs [--seeds 21,1,2] [--variants a,b | 'a:k=v;b:k=v'] [--order-frac F]\n"
                         "                      [--no-residual] [--csv FILE] [--verbose]\n";
            return a == "--help" ? 0 : 2;
        }
    }
    std::ofstream out;
    if (!csv.empty()) {
        out.open(csv, std::ios::app);
        out.seekp(0, std::ios::end);
        if (out.tellp() == 0)
            out << "seed,variant,nClusters,nKept,gcsLenMean,gcsGapMean,gcsSolves,tCluster,tOrder,tGcs,tStatic,"
                   "seedJ0,seedJ1,Jstatic,residual,detected,maxKappa,feasible,winners\n";
    }
    std::printf("%-5s %-28s %4s %5s %7s %7s %7s %8s %9s %9s %9s %4s %s\n", "seed", "variant", "K", "kept",
                "t_wp", "t_stat", "gap", "seedJ0", "J_static", "J_seed1", "residual", "det", "winners");
    for (const int sd : seeds) {
        mc::PlannerParams base;
        base.rngSeed = static_cast<std::uint64_t>(sd);
        base.verbose = verbose;
        base.numAgents = numAgents;
        const mc::BeliefField belief = mc::mapgen::generateBeliefMap(base.mapSize, base.cellSize, mc::BeliefMapParams{}, base.rngSeed);
        const std::vector<mc::Target> targets0 = mc::mapgen::generateTargetPoses(belief, numTargets, base.rngSeed + 1);
        std::vector<mc::Vec2> starts;
        for (int a = 0; a < base.numAgents; ++a) starts.emplace_back(2000.0 + a, 2000.0 + a);
        for (const std::string& vspec : variants) {
            mc::PlannerParams p = base;
            // the GCS defaults the 2026-10-12 presets were written against (the
            // library defaults changed with the 2026-10-22 revival)
            p.team.gcs.mode = mc::GcsMode::Drop;
            p.team.gcs.knn = 0;
            p.team.gcs.warmStart = false;
            p.team.gcs.setRadiusMode = mc::GcsSetRadius::Fixed;
            p.team.reallocate = false;
            p.team.orderBudgetFrac = orderFrac;
            const auto colon = vspec.find(':');
            const std::string name = vspec.substr(0, colon);
            std::string err;
            const auto it = kPresets.find(name);
            if (it != kPresets.end() && !apply(it->second, p, err)) { std::cerr << err << "\n"; return 2; }
            if (colon != std::string::npos && !apply(vspec.substr(colon + 1), p, err)) { std::cerr << err << "\n"; return 2; }
            if (it == kPresets.end() && colon == std::string::npos) { std::cerr << "unknown variant " << name << "\n"; return 2; }
            try {
                mc::Planner planner(p);
                const mc::PlanningResult r = planner.plan(belief, starts);
                const mc::TeamInfo& T = r.team;
                double res = std::nan("");
                int det = 0;
                if (residual) {
                    res = mc::eval::computeResidualBelief(belief, r.trajectories, p.fov, p.sensor).residualMass;
                    std::vector<mc::Target> tg = targets0;
                    mc::eval::updateTargetDetectionProbs(r.trajectories, r.timeVec, tg, p.fov, p.sensor, p.detectionThreshold);
                    for (const mc::Target& t : tg) det += t.detected() ? 1 : 0;
                }
                std::size_t kept = 0;
                double glen = 0.0, ggap = 0.0;
                int gsol = 0;
                for (std::size_t a = 0; a < T.gcsClusters.size(); ++a) {
                    kept += T.gcsClusters[a].size();
                    glen += T.gcsLength[a] / static_cast<double>(T.gcsClusters.size());
                    ggap += T.gcsGap[a] / static_cast<double>(T.gcsClusters.size());
                    gsol += T.gcsSolves[a];
                }
                // seed J of the winner, per agent, summed as the last agent's team J would read it:
                // report agent-mean of the best seed J0 and its J after sweep-1 optimisation
                double sj0 = 0.0, sj1 = 0.0;
                std::string win;
                for (std::size_t a = 0; a < r.plans.size(); ++a) {
                    std::size_t b = 0;
                    for (std::size_t q = 1; q < T.seedJ1[a].size(); ++q)
                        if (T.seedJ1[a][q] < T.seedJ1[a][b]) b = q;
                    sj0 = T.seedJ0[a][b];    // last agent: the team J of the seeds
                    sj1 = T.seedJ1[a][b];
                    win += (a ? "+" : "") + r.plans[a].initStrategy;
                }
                double mk = 0.0;
                bool feas = true;
                for (const mc::AgentPlan& pl : r.plans) {
                    mk = std::max(mk, pl.maxKappa);
                    feas = feas && pl.feasible;
                }
                if (!dump.empty()) {
                    // seed polylines, final flown tracks, GCS waypoints and the fast-grid prior
                    std::string tag = vspec;
                    for (char& ch : tag) if (ch == ':' || ch == ',' || ch == '=' || ch == ';') ch = '_';
                    std::ofstream f(dump + "/s" + std::to_string(sd) + "_" + tag + ".csv");
                    f << "kind,agent,k,x,y\n";
                    for (std::size_t a = 0; a < r.plans.size(); ++a) {
                        const mc::Path2& sp = r.plans[a].seedPath;
                        for (mc::Index k = 0; k < sp.rows(); ++k) f << "seed," << a << ',' << k << ',' << sp(k, 0) << ',' << sp(k, 1) << '\n';
                        const mc::Path3& dr = r.trajectories[a].drone;
                        for (mc::Index k = 0; k < r.trajectories[a].steps; k += 10) f << "final," << a << ',' << k << ',' << dr(k, 0) << ',' << dr(k, 1) << '\n';
                        if (a < T.gcsWaypoints.size())
                            for (mc::Index k = 0; k < T.gcsWaypoints[a].rows(); ++k)
                                f << "gcs," << a << ',' << k << ',' << T.gcsWaypoints[a](k, 0) << ',' << T.gcsWaypoints[a](k, 1) << '\n';
                    }
                    std::ofstream g(dump + "/s" + std::to_string(sd) + "_prior.csv");
                    g << r.grid.x0 << ',' << r.grid.y0 << ',' << r.grid.hg << '\n' << r.grid.prior.format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n"));
                }
                const double tWp = T.orderTime + T.gcsTime + T.clusterTime;
                const double tStat = T.staticTime + T.clusterTime;
                std::printf("%-5d %-28s %4lld %5zu %7.2f %7.1f %7.2f %8.5f %9.5f %9.5f %9.5f %4d %s\n", sd, vspec.c_str(),
                            static_cast<long long>(r.clusters.size()), kept, tWp, tStat, ggap, sj0, T.Jstatic, sj1,
                            res, det, win.c_str());
                std::fflush(stdout);
                if (out)
                    out << sd << ",\"" << vspec << "\"," << r.clusters.size() << ',' << kept << ',' << glen << ',' << ggap
                        << ',' << gsol << ',' << T.clusterTime << ',' << T.orderTime << ',' << T.gcsTime << ','
                        << tStat << ',' << sj0 << ',' << sj1 << ',' << T.Jstatic << ',' << res << ',' << det << ','
                        << mk << ',' << (feas ? 1 : 0) << ',' << win << '\n'
                        << std::flush;
            } catch (const std::exception& e) {
                std::fprintf(stderr, "seed %d variant %s: error %s\n", sd, vspec.c_str(), e.what());
            }
        }
    }
    return 0;
}
