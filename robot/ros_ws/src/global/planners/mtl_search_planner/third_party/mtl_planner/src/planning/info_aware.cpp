#include "mtl/planning/info_aware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <future>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "mtl/core/kmeans.hpp"
#include "mtl/mapping/cells.hpp"
#include "mtl/mapping/peak_clusters.hpp"
#include "mtl/planner.hpp"
#include "mtl/planning/coverage_score.hpp"

namespace mtl::planning {
namespace {

/// One abstraction to plan: the parameter set it is planned with and, unless
/// it is the plain plan, the clusters.
struct Job {
    std::string   label;
    PlannerParams params;
    bool          plain = false;  ///< planFromCells (the planner clusters itself)
    ClusterSet    clusters;
};

struct Outcome {
    PlanningResult result;
    double         score = -kInf;
    double         flown = 0.0;
};

std::string fracsLabel(const std::vector<double>& f) {
    std::ostringstream o;
    o << "[";
    for (std::size_t i = 0; i < f.size(); ++i) o << (i ? "," : "") << f[i];
    o << "]";
    return o.str();
}

double teamFlown(const PlanningResult& r) {
    double s = 0.0;
    for (const AgentPlan& p : r.plans) s += p.flownLength;
    return s;
}

/// Plan every job, `threads` at a time.  Order of the output = order of the jobs.
std::vector<Outcome> runJobs(const std::vector<Job>& jobs, const CellSet& cells,
                             const std::vector<Vec2>& starts, const CoverageModel& cov,
                             int lookStride, int threads) {
    std::vector<Outcome> out(jobs.size());
    auto one = [&](std::size_t i) {
        const Job& j = jobs[i];
        Planner planner(j.params);
        Outcome o;
        o.result = j.plain ? planner.planFromCells(cells, starts)
                           : planner.planFromClusters(cells, j.clusters, starts);
        o.score  = cov.detectedMass(o.result.trajectories, lookStride);
        o.flown  = teamFlown(o.result);
        out[i]   = std::move(o);
    };
    if (threads <= 1 || jobs.size() <= 1) {
        for (std::size_t i = 0; i < jobs.size(); ++i) one(i);
        return out;
    }
    for (std::size_t base = 0; base < jobs.size(); base += static_cast<std::size_t>(threads)) {
        std::vector<std::future<void>> fs;
        const std::size_t end = std::min(jobs.size(), base + static_cast<std::size_t>(threads));
        for (std::size_t i = base; i < end; ++i) fs.push_back(std::async(std::launch::async, one, i));
        for (auto& f : fs) f.get();
    }
    return out;
}

/// Better = more detected mass; a tie (within round-off of the model) goes to
/// the shorter flight.
bool better(const Outcome& a, const Outcome& b) {
    if (a.score > b.score + 1e-6) return true;
    if (a.score < b.score - 1e-6) return false;
    return a.flown < b.flown - 1e-6;
}

std::vector<std::vector<Index>> membersOf(const ClusterSet& cl) { return cl.cellIdx; }

/// Largest cell distance from the members' mean.
double radiusOf(const CellSet& cells, const std::vector<Index>& mem) {
    if (mem.empty()) return 0.0;
    Vec2 c = Vec2::Zero();
    for (const Index g : mem) c += cells.centers.row(g).transpose();
    c /= static_cast<double>(mem.size());
    double r = 0.0;
    for (const Index g : mem) r = std::max(r, (cells.centers.row(g).transpose() - c).norm());
    return r;
}

ClusterSet rebuild(const CellSet& cells, const std::vector<std::vector<Index>>& mem,
                   const std::vector<int>& basin, const std::vector<int>& level) {
    // drop empties, keep the per-cluster tags aligned
    std::vector<std::vector<Index>> m2;
    std::vector<int> b2, l2;
    for (std::size_t k = 0; k < mem.size(); ++k) {
        if (mem[k].empty()) continue;
        m2.push_back(mem[k]);
        if (basin.size() == mem.size()) b2.push_back(basin[k]);
        if (level.size() == mem.size()) l2.push_back(level[k]);
    }
    return mapping::clusterSetFromMembers(cells, m2, b2, l2);
}

/// The split / peel / merge neighbourhood of an incumbent plan.
std::vector<std::pair<std::string, ClusterSet>> movesOf(const PlanningResult& inc,
                                                        const ClusterSet& cl, const CellSet& cells,
                                                        const PlannerParams& q,
                                                        const InfoAwareParams& ia,
                                                        std::uint64_t seed) {
    std::vector<std::pair<std::string, ClusterSet>> moves;
    const double reach = q.maxClusterRadius;
    const std::vector<std::vector<Index>> mem = membersOf(cl);
    const auto K = static_cast<Index>(mem.size());
    std::vector<int> basin = cl.basin, level = cl.level;
    if (static_cast<Index>(basin.size()) != K) basin.assign(static_cast<std::size_t>(K), -1);
    if (static_cast<Index>(level.size()) != K) level.assign(static_cast<std::size_t>(K), 0);

    std::vector<char> chosen(static_cast<std::size_t>(K), 0);
    for (const Index c : inc.team.reachedClusters)
        if (c >= 0 && c < K) chosen[static_cast<std::size_t>(c)] = 1;

    // Chosen clusters, richest first: that is where a better abstraction pays most.
    std::vector<Index> order;
    for (Index k = 0; k < K; ++k)
        if (chosen[static_cast<std::size_t>(k)]) order.push_back(k);
    std::stable_sort(order.begin(), order.end(), [&](Index a, Index b) { return cl.reward(a) > cl.reward(b); });

    ClusterParams cp = q.cluster;
    for (const Index c : order) {
        const std::vector<Index>& mc = mem[static_cast<std::size_t>(c)];

        // PEEL: keep the densest cells holding peelKeep of the mass.
        if (mc.size() >= 3) {
            std::vector<Index> s = mc;
            std::stable_sort(s.begin(), s.end(), [&](Index a, Index b) { return cells.mass(a) > cells.mass(b); });
            const double tot = cl.reward(c);
            std::vector<Index> keep, rest;
            double cum = 0.0;
            for (const Index g : s) {
                (cum < ia.peelKeep * tot - 1e-15 || keep.empty() ? keep : rest).push_back(g);
                cum += cells.mass(g);
            }
            if (!rest.empty()) {
                auto m2 = mem;
                auto b2 = basin, l2 = level;
                m2[static_cast<std::size_t>(c)] = keep;
                for (auto& part : mapping::splitUnderRadius(cells, rest, reach, cp, seed + 31ULL * static_cast<std::uint64_t>(c))) {
                    m2.push_back(std::move(part));
                    b2.push_back(basin[static_cast<std::size_t>(c)]);
                    l2.push_back(level[static_cast<std::size_t>(c)] + 1);
                }
                moves.emplace_back("peel", rebuild(cells, m2, b2, l2));
            }
        }

        // SPLIT in two.
        if (mc.size() >= 2) {
            Path2 pts(static_cast<Index>(mc.size()), 2);
            for (std::size_t i = 0; i < mc.size(); ++i) pts.row(static_cast<Index>(i)) = cells.centers.row(mc[i]);
            core::KMeansOptions ko;
            ko.maxIter = cp.kmeansMaxIter;
            ko.replicates = cp.kmeansReplicates;
            ko.seed = seed + 17ULL * static_cast<std::uint64_t>(c);
            const core::KMeansResult r = core::kmeans(pts, 2, ko);
            std::vector<Index> a, b;
            for (std::size_t i = 0; i < mc.size(); ++i) (r.assignment[i] == 0 ? a : b).push_back(mc[i]);
            if (!a.empty() && !b.empty()) {
                auto m2 = mem;
                auto b2 = basin, l2 = level;
                m2[static_cast<std::size_t>(c)] = a;
                m2.push_back(b);
                b2.push_back(basin[static_cast<std::size_t>(c)]);
                l2.push_back(level[static_cast<std::size_t>(c)]);
                moves.emplace_back("split", rebuild(cells, m2, b2, l2));
            }
        }

        // MERGE with the nearest unchosen cluster whose union still fits.
        Index best = -1;
        double bestD = kInf;
        for (Index d = 0; d < K; ++d) {
            if (d == c || chosen[static_cast<std::size_t>(d)]) continue;
            const double dist = (cl.centroids.row(c) - cl.centroids.row(d)).norm();
            if (dist >= bestD || dist > 2.0 * reach) continue;
            std::vector<Index> u = mc;
            u.insert(u.end(), mem[static_cast<std::size_t>(d)].begin(), mem[static_cast<std::size_t>(d)].end());
            if (radiusOf(cells, u) <= reach) {
                bestD = dist;
                best  = d;
            }
        }
        if (best >= 0) {
            auto m2 = mem;
            m2[static_cast<std::size_t>(c)].insert(m2[static_cast<std::size_t>(c)].end(),
                                                   mem[static_cast<std::size_t>(best)].begin(),
                                                   mem[static_cast<std::size_t>(best)].end());
            m2[static_cast<std::size_t>(best)].clear();
            moves.emplace_back("merge", rebuild(cells, m2, basin, level));
        }
    }
    return moves;
}

}  // namespace

// -----------------------------------------------------------------------------
double detectionReach(const PlannerParams& P) {
    double S = P.infoAware.slantMargin * P.sensor.beta;
    if (std::isfinite(P.gimbal.maxSlantRange)) S = std::min(S, P.gimbal.maxSlantRange);
    const double h = P.droneAltitude;
    if (P.singleAxisGimbal) {
        // The swept line: a look at cross angle phi has slant h / (cos(tau) cos(phi))
        // and lands h tan(phi) to the side.
        const double c = h / (S * std::cos(P.sensorTiltAngle));
        if (!(c < 1.0)) return 0.0;
        return h * std::tan(std::acos(c));
    }
    return (S > h) ? std::sqrt(S * S - h * h) : 0.0;
}

// -----------------------------------------------------------------------------
PlanningResult planInfoAware(const PlannerParams& P0, const CellSet& cells,
                             const std::vector<Vec2>& agentStarts) {
    const auto t0 = std::chrono::steady_clock::now();
    const InfoAwareParams& ia = P0.infoAware;

    PlannerParams plain = P0;
    plain.infoAware.enabled = false;
    plain.finalize();

    int threads = ia.threads;
    if (threads <= 0) threads = static_cast<int>(std::max(1u, std::min(8u, std::thread::hardware_concurrency())));

    const CoverageModel cov(cells, ia.subsample, P0.fov, P0.sensor);
    const mapping::PeakBasins basins = mapping::findPeakBasins(cells, ia.persistence);
    const double dReach = detectionReach(P0);

    // --- the candidate abstractions --------------------------------------------
    std::vector<Job> jobs;
    {
        Job j;
        j.label  = "baseline";
        j.params = plain;
        j.plain  = true;
        jobs.push_back(std::move(j));
    }

    struct ReachOpt { double r; bool capped; };
    std::vector<ReachOpt> reaches{{P0.maxClusterRadius, false}};
    if (dReach > 0.0)
        for (const double s : ia.reachScales) reaches.push_back({s * dReach, ia.capGimbalToDetection});

    for (const ReachOpt& ro : reaches) {
        PlannerParams q = plain;
        q.maxClusterRadius = ro.r;
        q.budget.cellAnchor.gimbalReach.reset();  // follows maxClusterRadius in finalize()
        if (ro.capped) {
            const double S = std::min(ia.slantMargin * P0.sensor.beta,
                                      std::isfinite(P0.gimbal.maxSlantRange) ? P0.gimbal.maxSlantRange : kInf);
            q.gimbal.maxSlantRange  = S;
            q.gimbal.maxSensorReach = std::min(q.gimbal.maxSensorReach, dReach);
            q.extension.maxSensorReach = std::min(q.extension.maxSensorReach, dReach);
        }
        q.finalize();
        const std::string at = "@" + std::to_string(static_cast<long long>(std::lround(ro.r))) +
                               (ro.capped ? "d" : "");

        if (ro.capped || std::abs(ro.r - P0.maxClusterRadius) > 1e-9) {
            Job j;
            j.label    = "kmeans" + at;
            j.params   = q;
            j.clusters = mapping::clusterCells(cells.centers, ro.r, q.cluster, q.rngSeed, false);
            mapping::computeClusterRewards(cells.mass, j.clusters);
            jobs.push_back(std::move(j));
        }
        for (const std::vector<double>& ls : ia.levelSets) {
            Job j;
            j.label    = "peaks" + fracsLabel(ls) + at;
            j.params   = q;
            j.clusters = mapping::clusterByPeaks(cells, basins, ls, ro.r, q.cluster, q.rngSeed);
            jobs.push_back(std::move(j));
        }
    }

    // Re-seeded copies of every abstraction (the orienteering heuristic's GRASP
    // restarts depend on the seed); the plain plan itself stays job 0.
    {
        const std::size_t nBase = jobs.size();
        for (int r = 1; r <= ia.restarts; ++r)
            for (std::size_t i = 0; i < nBase; ++i) {
                Job j = jobs[i];
                j.label += "~s" + std::to_string(r);
                j.params.budget.orienteering.seed =
                    jobs[i].params.budget.orienteering.seed + 7919ULL * static_cast<std::uint64_t>(r);
                jobs.push_back(std::move(j));
            }
    }

    std::vector<Outcome> outs = runJobs(jobs, cells, agentStarts, cov, ia.lookStride, threads);

    InfoAwareReport rep;
    rep.enabled        = true;
    rep.detectionReach = dReach;
    rep.basins         = basins.size();
    std::size_t bestI = 0;
    for (std::size_t i = 0; i < jobs.size(); ++i) {
        InfoAwareCandidate c;
        c.label    = jobs[i].label;
        c.reach    = jobs[i].params.maxClusterRadius;
        c.score    = outs[i].score;
        c.info     = outs[i].result.team.info;
        c.flown    = outs[i].flown;
        c.clusters = outs[i].result.clusters.size();
        rep.candidates.push_back(c);
        if (better(outs[i], outs[bestI])) bestI = i;
    }
    rep.baselineScore = outs[0].score;

    Outcome     incumbent = std::move(outs[bestI]);
    std::string incLabel  = jobs[bestI].label;
    PlannerParams incParams = jobs[bestI].params;
    ClusterSet  incClusters = incumbent.result.clusters;

    // --- split / peel / merge on the incumbent -----------------------------------
    if (ia.splitMerge && ia.maxMoves > 0) {
        std::uint64_t seed = P0.rngSeed * 2654435761ULL + 99ULL;
        std::vector<std::pair<std::string, ClusterSet>> moves =
            movesOf(incumbent.result, incClusters, cells, incParams, ia, seed);
        std::size_t next = 0;
        while (rep.movesTried < ia.maxMoves && next < moves.size()) {
            const std::size_t take = std::min<std::size_t>(
                {moves.size() - next, static_cast<std::size_t>(std::max(1, threads)),
                 static_cast<std::size_t>(ia.maxMoves - rep.movesTried)});
            std::vector<Job> batch;
            for (std::size_t i = 0; i < take; ++i) {
                Job j;
                j.label    = "move:" + moves[next + i].first;
                j.params   = incParams;
                j.clusters = moves[next + i].second;
                batch.push_back(std::move(j));
            }
            next += take;
            rep.movesTried += static_cast<int>(take);
            std::vector<Outcome> bo = runJobs(batch, cells, agentStarts, cov, ia.lookStride, threads);

            std::size_t bi = 0;
            for (std::size_t i = 1; i < bo.size(); ++i)
                if (better(bo[i], bo[bi])) bi = i;
            const bool improved = better(bo[bi], incumbent);
            for (std::size_t i = 0; i < bo.size(); ++i) {
                InfoAwareCandidate c;
                c.label    = batch[i].label;
                c.reach    = incParams.maxClusterRadius;
                c.score    = bo[i].score;
                c.info     = bo[i].result.team.info;
                c.flown    = bo[i].flown;
                c.clusters = bo[i].result.clusters.size();
                c.accepted = improved && i == bi;
                rep.candidates.push_back(c);
            }
            if (improved) {
                ++rep.movesAccepted;
                incumbent   = std::move(bo[bi]);
                incLabel    = incLabel + "+" + batch[bi].label.substr(5);
                incClusters = incumbent.result.clusters;
                moves = movesOf(incumbent.result, incClusters, cells, incParams, ia,
                                seed + static_cast<std::uint64_t>(rep.movesTried));
                next = 0;
            }
        }
    }

    PlanningResult out = std::move(incumbent.result);
    rep.chosen      = incLabel;
    rep.chosenScore = incumbent.score;
    rep.chosenReach = incParams.maxClusterRadius;
    rep.seconds     = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    out.infoAware   = std::move(rep);

    if (P0.verbose) {
        std::printf("[mtl] info-aware: %zu candidates, %d/%d moves accepted; chose %s "
                    "(detected %.4f vs baseline %.4f), %.1f s\n",
                    out.infoAware.candidates.size(), out.infoAware.movesAccepted,
                    out.infoAware.movesTried, out.infoAware.chosen.c_str(),
                    out.infoAware.chosenScore, out.infoAware.baselineScore, out.infoAware.seconds);
    }
    return out;
}

}  // namespace mtl::planning
