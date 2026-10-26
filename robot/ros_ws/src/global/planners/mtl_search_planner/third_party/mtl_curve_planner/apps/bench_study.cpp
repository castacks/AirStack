// =============================================================================
//  apps/bench_study.cpp   ->   the `mtlc_bench_study` tool
//
//  bench_gcs.cpp extended for the information-aware cluster study
//  (2026-10-22): scenario axes (--prior, --budget-s, --start), the
//  information-aware keys (ia=1, ia_ls, ia_rs, ia_km, ia_rst, ia_mv, ia_topk,
//  ia_pers, ia_peel), GCS set-radius keys, and the WALL time of the whole plan
//  (clustering + search + seeding + optimisation) as `tPlan`.
//
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
    // --- information-aware study (2026-10-22) ---
    // Explicit so the study stays reproducible when the defaults change.
    // ia_rs=0.5/1,ia_sit=0 = the first (raw-seed scored) setting of the study;
    // "ia-tuned" = the recommended one (now the InfoAwareParams defaults).
    // GCS = orienteering mode, 8-nearest-neighbour edges, capture-radius sets,
    // warm-started from the orienteering heuristic's order (gcs/GCS_REVIVAL_NOTES.md).
    {"ia",            "mode=radius,init=clusters+greedy,ia=1,ia_rs=0.5/1,ia_sit=0,ia_sxit=0"},
    {"ia-tuned",      "mode=radius,init=clusters+greedy,ia=1,ia_rs=1,ia_sit=40,ia_sxit=0"},
    {"gcs-r",         "mode=radius,init=gcs,gmode=op,cw=0,knn=8,rmode=capture,warm=1,gtol=1e-4"},
    {"gcs-ia",        "mode=radius,init=gcs,gmode=op,cw=0,knn=8,rmode=capture,warm=1,gtol=1e-4,ia=1,ia_rs=0.5/1,ia_sit=0,ia_sxit=0,ia_score=1"},
    {"gcsF-r",        "mode=radius,init=gcs,gmode=op,cw=0,knn=8,rmode=fixed,warm=1,gtol=1e-4"},
    {"gcsF-ia",       "mode=radius,init=gcs,gmode=op,cw=0,knn=8,rmode=fixed,warm=1,gtol=1e-4,ia=1,ia_rs=0.5/1,ia_sit=0,ia_sxit=0,ia_score=1"},
    {"gcs+g-r",       "mode=radius,init=gcs+greedy,gmode=op,cw=0,knn=8,rmode=capture,warm=1,gtol=1e-4"},
    {"gcs+g-ia",      "mode=radius,init=gcs+greedy,gmode=op,cw=0,knn=8,rmode=capture,warm=1,gtol=1e-4,ia=1,ia_rs=0.5/1,ia_sit=0,ia_sxit=0,ia_score=1"},
    {"gcsop-r",       "mode=radius,init=gcs,gmode=op,cw=0,knn=0,rmode=fixed,warm=0"},
    {"gcsop-ia",      "mode=radius,init=gcs,gmode=op,cw=0,knn=0,rmode=fixed,warm=0,ia=1,ia_rs=0.5/1,ia_sit=0"},
    {"gcsdrop-r",     "mode=radius,init=gcs,drop=info,cw=0"},
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
        else if (k == "ia") p.infoAware.enabled = std::stoi(v) != 0;
        else if (k == "ia_ls") {   // ia_ls=1|0.5/1|0.35/0.7/1
            p.infoAware.levelSets.clear();
            for (const std::string& t : split(v, '|')) {
                std::vector<double> f;
                for (const std::string& x : split(t, '/')) f.push_back(std::stod(x));
                p.infoAware.levelSets.push_back(f);
            }
        }
        else if (k == "ia_rs") { p.infoAware.reachScales.clear(); for (const std::string& x : split(v, '/')) p.infoAware.reachScales.push_back(std::stod(x)); }
        else if (k == "ia_km") p.infoAware.includeKmeans = std::stoi(v) != 0;
        else if (k == "ia_rst") p.infoAware.restarts = std::stoi(v);
        else if (k == "ia_mv") p.infoAware.maxMoves = std::stoi(v);
        else if (k == "ia_topk") p.infoAware.optimizeTopK = std::stoi(v);
        else if (k == "ia_pers") p.infoAware.persistence = std::stod(v);
        else if (k == "ia_peel") p.infoAware.peelKeep = std::stod(v);
        else if (k == "rmode") p.team.gcs.setRadiusMode = v == "capture" ? mc::GcsSetRadius::Capture : mc::GcsSetRadius::Fixed;
        else if (k == "oplw") p.team.gcs.opLengthWeight = std::stod(v);
        else if (k == "nexact") p.team.gcs.nExact = std::stoi(v);
        else if (k == "knn") p.team.gcs.knn = std::stoi(v);
        else if (k == "warm") p.team.gcs.warmStart = std::stoi(v) != 0;
        else if (k == "turnw") p.team.gcs.turnWeight = std::stod(v);
        else if (k == "gtol") p.team.gcs.tol = std::stod(v);
        else if (k == "ia_score") p.infoAware.scoreWithClusters = std::stoi(v) != 0;
        else if (k == "ia_sit") p.infoAware.scoreIters = std::stoi(v);
        else if (k == "ia_sxit") p.infoAware.scoreExploreIters = std::stoi(v);
        else if (k == "mcr") p.maxClusterRadius = std::stod(v);
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
    std::string prior = "default", startName = "default";
    double budgetS = 250.0;
    double evalRes = 1.0;   // [m] grid of the residual evaluator (the prior re-sampled at this step)
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--eval-res" && i + 1 < argc) {
            evalRes = std::stod(argv[++i]);
        } else if (a == "--prior" && i + 1 < argc) {
            prior = argv[++i];
        } else if (a == "--start" && i + 1 < argc) {
            startName = argv[++i];
        } else if (a == "--budget-s" && i + 1 < argc) {
            budgetS = std::stod(argv[++i]);
        } else if (a == "--seeds" && i + 1 < argc) {
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
            out << "prior,budget_s,start,seed,variant,nClusters,nKept,gcsLenMean,gcsGapMean,gcsSolves,tCluster,tOrder,tGcs,tStatic,"
                   "tPlan,seedJ0,seedJ1,Jstatic,residual,detected,maxKappa,feasible,winners,iaChosen,iaReach,iaSeedJ,"
                   "iaPlainSeedJ,iaMoves,iaBasins,iaTime,seedJ1all,iaCands\n";
    }
    std::printf("%-5s %-28s %4s %5s %7s %7s %7s %8s %9s %9s %9s %4s %s\n", "seed", "variant", "K", "kept",
                "t_wp", "t_stat", "gap", "seedJ0", "J_static", "J_seed1", "residual", "det", "winners");
    for (const int sd : seeds) {
        mc::PlannerParams base;
        base.rngSeed = static_cast<std::uint64_t>(sd);
        base.verbose = verbose;
        base.numAgents = numAgents;
        base.maxFlightTime = budgetS;
        mc::BeliefMapParams bp;   // "default": 10 bumps, sigma 200-500 m
        if (prior == "peaked")       { bp.numCentroids = 6;  bp.sigmaMin = 100; bp.sigmaMax = 250; }
        else if (prior == "many")    { bp.numCentroids = 20; bp.sigmaMin = 150; bp.sigmaMax = 350; }
        else if (prior == "diffuse") { bp.numCentroids = 5;  bp.sigmaMin = 500; bp.sigmaMax = 900; }
        else if (prior == "few")     { bp.numCentroids = 3;  bp.sigmaMin = 200; bp.sigmaMax = 500; }
        else if (prior != "default") { std::cerr << "unknown prior " << prior << "\n"; return 2; }
        const mc::BeliefField belief = mc::mapgen::generateBeliefMap(base.mapSize, base.cellSize, bp, base.rngSeed);
        const std::vector<mc::Target> targets0 = mc::mapgen::generateTargetPoses(belief, numTargets, base.rngSeed + 1);
        // the SAME bumps (the rng draws do not depend on the grid) sampled at the
        // evaluation step and normalised: a coarser quadrature of the same prior
        const mc::BeliefField beliefEval = evalRes > 1.0
            ? mc::mapgen::generateBeliefMap(base.mapSize, evalRes, bp, base.rngSeed) : belief;
        mc::Vec2 s0(2000.0, 2000.0);
        if (startName == "center") s0 = mc::Vec2(2500.0, 2500.0);
        else if (startName == "corner") s0 = mc::Vec2(200.0, 200.0);
        else if (startName == "edge") s0 = mc::Vec2(2500.0, 200.0);
        else if (startName != "default") { std::cerr << "unknown start " << startName << "\n"; return 2; }
        std::vector<mc::Vec2> starts;
        for (int a = 0; a < base.numAgents; ++a) starts.emplace_back(s0.x() + a, s0.y() + a);
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
                const auto tP0 = std::chrono::steady_clock::now();
                const mc::PlanningResult r = planner.plan(belief, starts);
                const double tPlan = std::chrono::duration<double>(std::chrono::steady_clock::now() - tP0).count();
                const mc::TeamInfo& T = r.team;
                double res = std::nan("");
                int det = 0;
                if (residual) {
                    res = mc::eval::computeResidualBelief(beliefEval, r.trajectories, p.fov, p.sensor).residualMass;
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
                    {   // the abstraction flown: cells with their cluster / basin / level, and the waypoint clusters
                        std::ofstream h(dump + "/s" + std::to_string(sd) + "_" + tag + "_clusters.csv");
                        h << "x,y,mass,cluster,basin,level,visited\n";
                        std::vector<char> vis(static_cast<std::size_t>(r.clusters.size()), 0);
                        for (const mc::AgentPlan& pl : r.plans)
                            for (const mc::Index k : pl.waypointClusters)
                                if (k >= 0 && k < r.clusters.size()) vis[static_cast<std::size_t>(k)] = 1;
                        for (mc::Index i = 0; i < r.cells.size(); ++i) {
                            const int k = r.clusters.cellCluster[static_cast<std::size_t>(i)];
                            const int b = k >= 0 && static_cast<std::size_t>(k) < r.clusters.basin.size() ? r.clusters.basin[static_cast<std::size_t>(k)] : -1;
                            const int l = k >= 0 && static_cast<std::size_t>(k) < r.clusters.level.size() ? r.clusters.level[static_cast<std::size_t>(k)] : -1;
                            h << r.cells.centers(i, 0) << ',' << r.cells.centers(i, 1) << ',' << r.cells.mass(i) << ','
                              << k << ',' << b << ',' << l << ',' << (k >= 0 ? int(vis[static_cast<std::size_t>(k)]) : 0) << '\n';
                        }
                    }
                    std::ofstream g(dump + "/s" + std::to_string(sd) + "_prior.csv");
                    g << r.grid.x0 << ',' << r.grid.y0 << ',' << r.grid.hg << '\n' << r.grid.prior.format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n"));
                }
                const double tWp = T.orderTime + T.gcsTime + T.clusterTime;
                const double tStat = T.staticTime + T.clusterTime;
                std::printf("%-5d %-28s %4lld %5zu %7.2f %7.1f %7.2f %8.5f %9.5f %9.5f %9.5f %4d %s  tPlan %.1f %s\n", sd, vspec.c_str(),
                            static_cast<long long>(r.clusters.size()), kept, tWp, tStat, ggap, sj0, T.Jstatic, sj1,
                            res, det, win.c_str(), tPlan, r.infoAware.chosen.c_str());
                std::fflush(stdout);
                if (out)
                {
                    const mc::InfoAwareReport& ia = r.infoAware;
                    std::string cands, sjAll;
                    for (std::size_t a = 0; a < r.plans.size(); ++a)
                        for (std::size_t q = 0; q < T.seedJ1[a].size(); ++q)
                            sjAll += (sjAll.empty() ? "" : "|") + std::to_string(T.seedJ0[a][q]) + "/" + std::to_string(T.seedJ1[a][q]);
                    for (const mc::InfoAwareCandidate& c : ia.candidates)
                        cands += (cands.empty() ? "" : "|") + c.label + "=" + std::to_string(c.seedJ) +
                                 (c.finalJ >= 0 ? "/" + std::to_string(c.finalJ) : std::string());
                    out << prior << ',' << budgetS << ',' << startName << ',' << sd << ",\"" << vspec << "\"," << r.clusters.size() << ',' << kept << ',' << glen << ',' << ggap
                        << ',' << gsol << ',' << T.clusterTime << ',' << T.orderTime << ',' << T.gcsTime << ','
                        << tStat << ',' << tPlan << ',' << sj0 << ',' << sj1 << ',' << T.Jstatic << ',' << res << ',' << det << ','
                        << mk << ',' << (feas ? 1 : 0) << ',' << win << ",\"" << ia.chosen << "\"," << ia.chosenReach << ','
                        << ia.chosenSeedJ << ',' << ia.plainSeedJ << ',' << ia.movesAccepted << '/' << ia.movesTried << ','
                        << ia.basins << ',' << ia.seconds << ",\"" << sjAll << "\",\"" << cands << "\"\n"
                        << std::flush;
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "seed %d variant %s: error %s\n", sd, vspec.c_str(), e.what());
            }
        }
    }
    return 0;
}
