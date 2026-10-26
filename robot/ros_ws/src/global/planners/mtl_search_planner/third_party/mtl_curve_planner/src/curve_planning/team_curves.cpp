#include "mtl_curve/curve_planning/team_curves.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <unordered_map>

#include "mtl_curve/curve_planning/init_spline.hpp"
#include "mtl_curve/gcs/gcs.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/curve_planning/path_insert.hpp"
#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/optimization/optimizer.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
#include "mtl_curve/planning/orienteering.hpp"
#include "mtl_curve/planning/sector_orienteering.hpp"

namespace mtl::curve::curve_planning {
namespace {

using optimization::AgentProblem;

struct Candidate {
    VecX v;
    CurveRep rep;
    MatX lam;
    OptimizeInfo info;
    Path2 seed;
    double J = kInf;
    bool feasible = false;
    bool skipped = false;   ///< Path seed whose budget could not be repaired
};

double teamJ(const FastGrid& G, const std::vector<MatX>& lam) {
    MatX Lt = MatX::Zero(G.ny, G.nx);
    for (const MatX& l : lam) Lt += l;
    return (G.prior.array() * Lt.array().exp()).sum();
}

MatX othersOf(const std::vector<MatX>& lam, std::size_t a, Index ny, Index nx) {
    MatX Lo = MatX::Zero(ny, nx);
    for (std::size_t b = 0; b < lam.size(); ++b)
        if (b != a) Lo += lam[b];
    return Lo;
}

std::vector<Index> orderClusters(const TeamConfig& cfg, std::size_t a, const std::vector<Index>& own,
                                 const PlannerParams& P) {
    if (own.empty()) return {};
    OrienteeringParams oo = P.team.orienteering;
    oo.verbose = false;
    switch (cfg.modes[a]) {
        case EndpointMode::ReturnHome: oo.endPos = cfg.starts[a]; break;
        case EndpointMode::FixedDest:  oo.endPos = cfg.dests[a]; break;
        default:                       oo.endPos.reset(); break;
    }
    Path2 nodes(static_cast<Index>(own.size()), 2);
    VecX rew(static_cast<Index>(own.size()));
    for (std::size_t i = 0; i < own.size(); ++i) {
        nodes.row(static_cast<Index>(i)) = cfg.centroids.row(own[i]);
        rew(static_cast<Index>(i)) = cfg.rewards(own[i]);
    }
    const planning::OrienteeringSolution sol = planning::solveBudgetedOrienteering(
        nodes, rew, cfg.starts[a], P.team.orderBudgetFrac * cfg.L[a], oo);
    std::vector<Index> wp;
    for (const Index k : sol.visitOrder) wp.push_back(own[static_cast<std::size_t>(k)]);
    return wp;
}

std::vector<Index> insertCheapest(const TeamConfig& cfg, std::size_t a, const std::vector<Index>& wp, Index k) {
    std::vector<Vec2> P{cfg.starts[a]};
    for (const Index w : wp) P.emplace_back(cfg.centroids.row(w).transpose());
    const Vec2 c = cfg.centroids.row(k).transpose();
    double best = kInf;
    std::size_t pos = wp.size();
    for (std::size_t i = 0; i < P.size(); ++i) {
        const double d = i + 1 < P.size()
                             ? (P[i] - c).norm() + (c - P[i + 1]).norm() - (P[i] - P[i + 1]).norm()
                             : (P[i] - c).norm();
        if (d < best) {
            best = d;
            pos = i;
        }
    }
    std::vector<Index> out = wp;
    out.insert(out.begin() + static_cast<std::ptrdiff_t>(pos), k);
    return out;
}

Path2 gatherCentroids(const TeamConfig& cfg, const std::vector<Index>& wp) {
    Path2 out(static_cast<Index>(wp.size()), 2);
    for (std::size_t i = 0; i < wp.size(); ++i) out.row(static_cast<Index>(i)) = cfg.centroids.row(wp[i]);
    return out;
}

bool curveFeasible(const VecX& v, const CurveRep& rep, double denseStep) {
    const Index Md = std::max<Index>(10, static_cast<Index>(std::lround(rep.L / denseStep)));
    const CurveSamples C = evalParametricCurve(v, rep, Md);
    bool ok = C.kappa.cwiseAbs().maxCoeff() <= rep.kappaMax + 1e-4 &&
              std::abs(C.ds.sum() - rep.L) <= 0.002 * rep.L;
    if (rep.hasEq()) ok = ok && (C.endPt - *rep.pGoal).norm() < 1.0;
    return ok;
}

}  // namespace

Eigen::MatrixXi clusterOfPixel(const FastGrid& G, const Path2& cellCenters, const std::vector<int>& cellCluster,
                               double cs) {
    auto key = [cs](double x, double y) {
        return static_cast<long long>(std::floor(y / cs)) * 1000000LL + static_cast<long long>(std::floor(x / cs));
    };
    std::unordered_map<long long, int> cellOf;
    for (Index i = 0; i < cellCenters.rows(); ++i)
        cellOf[key(cellCenters(i, 0), cellCenters(i, 1))] = cellCluster[static_cast<std::size_t>(i)];
    Eigen::MatrixXi cop = Eigen::MatrixXi::Constant(G.ny, G.nx, -1);
    for (Index c = 0; c < G.nx; ++c)
        for (Index r = 0; r < G.ny; ++r) {
            const auto it = cellOf.find(key(G.x(c), G.y(r)));
            if (it != cellOf.end()) cop(r, c) = it->second;
        }
    return cop;
}

namespace {

ClusterAudit auditTeam(const TeamConfig& cfg, const TeamSolution& team, const PlannerParams& P) {
    const FastGrid& G = *cfg.G;
    const std::size_t N = team.v.size();
    const Index K = cfg.centroids.rows();
    MatX Lt = MatX::Zero(G.ny, G.nx);
    for (const MatX& l : team.lam) Lt += l;
    const MatX W = (G.prior.array() * Lt.array().exp()).matrix();

    ClusterAudit A;
    A.priorMass = VecX::Zero(K);
    A.residMass = VecX::Zero(K);
    for (Index c = 0; c < G.nx; ++c)
        for (Index r = 0; r < G.ny; ++r) {
            const int k = cfg.clusterOfPixel(r, c);
            if (k < 0) continue;
            A.priorMass(k) += G.prior(r, c);
            A.residMass(k) += W(r, c);
        }
    A.residFrac = A.residMass.cwiseQuotient(A.priorMass.cwiseMax(1e-300));
    A.distToCurve = MatX::Constant(K, static_cast<Index>(N), kInf);
    std::vector<VecX> yields(N), dsAll(N);
    for (std::size_t a = 0; a < N; ++a) {
        const CurveSamples C = evalParametricCurve(team.v[a], team.rep[a]);
        for (Index k = 0; k < K; ++k) {
            double best = kInf;
            for (Index j = 0; j < C.pts.rows(); ++j)
                best = std::min(best, (C.pts.row(j) - cfg.centroids.row(k)).squaredNorm());
            A.distToCurve(k, static_cast<Index>(a)) = std::sqrt(best);
        }
        Path2 gp;
        VecX gth, gds;
        optimization::swathKernelGradient(C.pts, C.tan, C.ds, G, cfg.kernels[a]->accurate, W, gp, gth, gds,
                                          &yields[a]);
        dsAll[a] = C.ds;
    }
    double hwMin = kInf;
    for (const SwathKernelSet* ks : cfg.kernels) hwMin = std::min(hwMin, ks->accurate.halfWidth);
    A.centroidInSwath.assign(static_cast<std::size_t>(K), 0);
    for (Index k = 0; k < K; ++k) {
        A.centroidInSwath[static_cast<std::size_t>(k)] = A.distToCurve.row(k).minCoeff() <= hwMin ? 1 : 0;
        if (A.residFrac(k) > P.team.unservicedResidualFrac && A.priorMass(k) > 0.0) A.unserviced.push_back(k);
    }
    double ySum = 0.0, dsSum = 0.0;
    for (std::size_t a = 0; a < N; ++a) {
        ySum += yields[a].sum();
        dsSum += dsAll[a].sum();
    }
    const double ybar = ySum / std::max(dsSum, 1e-300);
    A.slack = VecX::Zero(static_cast<Index>(N));
    A.ownCoverage = VecX::Zero(static_cast<Index>(N));
    A.isSlack.assign(N, 0);
    for (std::size_t a = 0; a < N; ++a) {
        double low = 0.0;
        for (Index j = 0; j < yields[a].size(); ++j)
            if (yields[a](j) / dsAll[a](j) < P.team.slackYieldFrac * ybar) low += dsAll[a](j);
        A.slack(static_cast<Index>(a)) = low / dsAll[a].sum();
        double ownPrior = 0.0, ownResid = 0.0;
        for (Index k = 0; k < K; ++k)
            if (cfg.assign[static_cast<std::size_t>(k)] == static_cast<int>(a)) {
                ownPrior += A.priorMass(k);
                ownResid += A.residMass(k);
            }
        A.ownCoverage(static_cast<Index>(a)) = 1.0 - ownResid / std::max(ownPrior, 1e-300);
        A.isSlack[a] = (A.slack(static_cast<Index>(a)) > 0.02 ||
                        A.ownCoverage(static_cast<Index>(a)) >= P.team.slackMarginThreshold) ? 1 : 0;
    }
    A.teamJ = W.sum();
    return A;
}

}  // namespace

TeamSolution reallocateCurveClusters(const TeamConfig& cfg, const PlannerParams& P, bool seedOnly) {
    const std::size_t N = cfg.starts.size();
    const FastGrid& G = *cfg.G;
    const bool hasX = cfg.Gx != nullptr && P.optimizer.exploreIter > 0;
    const bool verbose = P.verbose;
    const double denseStep = P.curve.denseStep;

    TeamSolution team;
    team.v.assign(N, VecX());
    team.rep.assign(N, CurveRep());
    team.lam.assign(N, MatX::Zero(G.ny, G.nx));
    team.wp.assign(N, {});
    team.initStrategy.assign(N, "");
    team.lastOpt.assign(N, OptimizeInfo());
    team.seedPath.assign(N, Path2());
    std::vector<MatX> lamX(N, hasX ? MatX::Zero(cfg.Gx->ny, cfg.Gx->nx) : MatX());

    // --- one agent's problem (and its exploration twin) ----------------------
    struct Probs {
        MatX Lo, LoX;
        AgentProblem prob, probX;
    };
    auto makeProbs = [&](std::size_t a, Probs& pr) {
        pr.Lo = othersOf(team.lam, a, G.ny, G.nx);
        pr.prob = AgentProblem{&G, &cfg.kernels[a]->accurate, &pr.Lo};
        if (hasX) {
            pr.LoX = othersOf(lamX, a, cfg.Gx->ny, cfg.Gx->nx);
            pr.probX = AgentProblem{cfg.Gx, &cfg.kernels[a]->explore, &pr.LoX};
        }
    };
    auto deposit = [&](std::size_t a, const VecX& v, const CurveRep& rep, MatX& L, MatX& LX) {
        L = optimization::agentDeposit(v, rep, G, cfg.kernels[a]->accurate);
        if (hasX) LX = optimization::agentDeposit(v, rep, *cfg.Gx, cfg.kernels[a]->explore);
    };
    auto solveSeed = [&](std::size_t a, const Probs& pr, const Path2& wps, int exploreIter, double* J0 = nullptr,
                         bool corridor = false) {
        Candidate cand;
        const InitResult init = initParametricSpline(cfg.starts[a], wps, cfg.L[a], cfg.modes[a], cfg.dests[a],
                                                     P.curve, pr.prob);
        cand.rep = init.rep;
        cand.seed = init.path;
        if (J0) *J0 = optimization::objectiveResidualBelief(init.v0, cand.rep, pr.prob);
        AgentProblem pc = pr.prob, pcX = pr.probX;
        if (corridor && P.team.corridorMu > 0.0) {
            for (AgentProblem* q : {&pc, &pcX}) {
                q->corridor = &cand.seed;
                q->corridorMu = P.team.corridorMu;
                q->corridorScale = P.team.corridorScale;
            }
        }
        cand.v = optimization::optimizeAgentCurve(init.v0, cand.rep, pc, hasX ? &pcX : nullptr,
                                                  P.optimizer, exploreIter, denseStep, &cand.info);
        cand.J = optimization::objectiveResidualBelief(cand.v, cand.rep, pr.prob);   // without the corridor
        return cand;
    };
    std::vector<Path2> corridorOf(N, Path2(0, 2));

    // the Path seed: splice cluster k into agent a's OPTIMISED curve (see
    // pathInsertSeed), copy the kept arcs into the seed, re-solve warm
    auto solvePathSeed = [&](std::size_t a, const Probs& pr, Index k) {
        Candidate cand;
        const MatX W = (G.prior.array() * (pr.Lo + team.lam[a]).array().exp()).matrix();
        const PathInsertSeed ps = pathInsertSeed(team.v[a], team.rep[a], Vec2(cfg.centroids.row(k).transpose()),
                                                 cfg.kernels[a]->accurate, G, W, P.team);
        if (verbose)
            std::fprintf(stderr,
                         "      path seed: insert after wp %lld (detour %.0f m), overrun %.0f -> %.0f m, %d cut(s), "
                         "%zu piece(s), prefix %.0f m kept\n",
                         static_cast<long long>(ps.insertAfter), ps.detour, ps.overrun0, ps.overrun, ps.cuts,
                         ps.pieces.size(), ps.sPrefix);
        if (ps.overrun > 0.0) {
            // even with every removable arc cut, the detour does not fit the
            // budget: the cluster cannot be claimed this way - do not spend an
            // optimisation on it
            cand.skipped = true;
            return cand;
        }
        const Splice sp{&team.v[a], &team.rep[a], &ps.pieces};
        const InitResult init = initParametricSpline(cfg.starts[a], Path2(0, 2), cfg.L[a], cfg.modes[a],
                                                     cfg.dests[a], P.curve, pr.prob, &sp);
        cand.rep = init.rep;
        cand.seed = init.path;
        const int xIt = P.team.reallocPathWarm ? P.optimizer.exploreIterWarm : P.optimizer.exploreIter;
        OptimizerParams op = P.optimizer;
        if (P.team.reallocPathMaxIter > 0) op.maxIter = P.team.reallocPathMaxIter;
        cand.v = optimization::optimizeAgentCurve(init.v0, cand.rep, pr.prob, hasX ? &pr.probX : nullptr,
                                                  op, xIt, denseStep, &cand.info);
        cand.J = cand.info.J;
        return cand;
    };

    const auto tStatic0 = std::chrono::steady_clock::now();
    // ---- 1. initial ordering + solve ---------------------------------------
    // seedOnly scores the abstraction: the greedy and sector seeds do not
    // depend on the clusters, so they are left out unless nothing else is listed
    std::vector<InitStrategy> strategies;
    for (const InitStrategy s0 : P.team.initStrategies)
        if (!seedOnly || (s0 != InitStrategy::Greedy && s0 != InitStrategy::Sector)) strategies.push_back(s0);
    if (strategies.empty()) strategies = P.team.initStrategies;
    auto uses = [&](InitStrategy s) { return std::find(strategies.begin(), strategies.end(), s) != strategies.end(); };
    auto ownOf = [&](std::size_t a) {
        std::vector<Index> own;
        for (std::size_t k = 0; k < cfg.assign.size(); ++k)
            if (cfg.assign[k] == static_cast<int>(a)) own.push_back(static_cast<Index>(k));
        return own;
    };
    {
        const auto tO = std::chrono::steady_clock::now();
        if (uses(InitStrategy::Clusters))
            for (std::size_t a = 0; a < N; ++a) team.wp[a] = orderClusters(cfg, a, ownOf(a), P);
        team.info.orderTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - tO).count();
    }
    // GCS seed: a route through the clusters' convex sets (Voronoi cell of the
    // centroid cut to the swath half-width), trimmed to orderBudgetFrac * L by
    // dropping clusters (gcs::planAgentGcs)
    std::vector<Path2> gcsWp(N, Path2(0, 2));
    std::vector<std::vector<Path2>> gcsAlt(N);
    std::vector<std::vector<Index>> gcsCl(N);
    std::vector<gcs::ConvexSet> gcsSets;
    const bool useGcs = uses(InitStrategy::Gcs);
    const bool gcsCoverage = P.team.gcs.mode == GcsMode::Coverage;
    const bool gcsResidual = P.team.gcs.residualRewards || P.team.gcs.ignorePartition || gcsCoverage;
    gcs::CoverageCells covCells;
    std::vector<gcs::CoverageCells> covExtra;
    // one agent's GCS route; rewards = static cluster rewards, or the belief
    // still unswept by the agents already planned (residualRewards)
    auto planGcs = [&](std::size_t a, const VecX& rewards, const std::vector<Index>& ids) {
        const auto tG = std::chrono::steady_clock::now();
        // turning overhead Rmin (D - sin D) per turn, scaled by turnWeight
        const double Rmin = P.team.gcs.turnWeight / P.curve.maxCurvature;
        const std::optional<Vec2> goal = cfg.modes[a] == EndpointMode::ReturnHome ? std::optional<Vec2>(cfg.starts[a])
                                         : cfg.modes[a] == EndpointMode::FixedDest ? cfg.dests[a]
                                                                                    : std::nullopt;
        const gcs::AgentGcsResult g =
            gcsCoverage ? gcs::planAgentGcsCoverage(covCells, rewards, cfg.starts[a], goal,
                                                    P.team.orderBudgetFrac * cfg.L[a], Rmin, P.team.gcs)
            : P.team.gcs.mode == GcsMode::Orienteering
                ? [&] {
                      std::vector<std::vector<Index>> warm;
                      if (P.team.gcs.warmStart) {
                          // the heuristic's order over the same clusters, rewards and budget
                          TeamConfig c2 = cfg;
                          c2.rewards = rewards;
                          warm.push_back(orderClusters(c2, a, ids, P));
                      }
                      return gcs::planAgentGcsOrienteering(gcsSets, rewards, ids, cfg.starts[a], goal,
                                                           P.team.orderBudgetFrac * cfg.L[a], Rmin, P.team.gcs,
                                                           warm.empty() ? nullptr : &warm);
                  }()
                : gcs::planAgentGcs(gcsSets, rewards, ids, cfg.starts[a], goal, P.team.orderBudgetFrac * cfg.L[a],
                                    Rmin, P.team.gcs);
        gcsWp[a] = g.wp;
        gcsAlt[a] = g.altWp;
        gcsCl[a] = g.clusters;
        team.info.gcsClusters[a] = g.clusters;
        team.info.gcsWaypoints[a] = g.wp;
        team.info.gcsLength[a] = g.len;
        team.info.gcsLowerBound[a] = g.lb;
        team.info.gcsGap[a] = g.gap;
        team.info.gcsSolves[a] = g.nSolves;
        team.info.gcsTime += std::chrono::duration<double>(std::chrono::steady_clock::now() - tG).count();
        if (verbose)
            std::fprintf(stderr,
                         "  agent %zu GCS: %zu of %zu clusters kept, route %.0f m (budget %.0f), lb %.0f, "
                         "%d solves, %.2f s\n",
                         a + 1, g.clusters.size(), ids.size(), g.len, P.team.orderBudgetFrac * cfg.L[a], g.lb,
                         g.nSolves, g.time);
    };
    if (useGcs) {
        const auto tG = std::chrono::steady_clock::now();
        double hw = kInf;
        for (const SwathKernelSet* ks : cfg.kernels) hw = std::min(hw, ks->accurate.halfWidth);
        const Eigen::Vector4d bounds =
            cfg.bounds ? *cfg.bounds
                       : Eigen::Vector4d(G.x(0) - 0.5 * G.hg, G.x(G.nx - 1) + 0.5 * G.hg, G.y(0) - 0.5 * G.hg,
                                         G.y(G.ny - 1) + 0.5 * G.hg);
        if (gcsCoverage) {
            const SwathKernel* kmin = &cfg.kernels[0]->accurate;
            for (const SwathKernelSet* ks : cfg.kernels)
                if (ks->accurate.halfWidth < kmin->halfWidth) kmin = &ks->accurate;
            covCells = gcs::coverageCells(G, *kmin, P.team.gcs.coverageCellFrac, bounds);
            for (const auto& cp : P.team.gcs.coverageExtra)
                covExtra.push_back(gcs::coverageCells(G, *kmin, cp.first, bounds));
            if (verbose)
                std::fprintf(stderr, "  coverage GCS: %zu hexagonal cells, circumradius %.0f m\n",
                             covCells.sets.size(), covCells.rho);
        } else {
            VecX radii = VecX::Constant(cfg.centroids.rows(), P.team.gcs.setRadiusFrac * hw);
            if (P.team.gcs.setRadiusMode == GcsSetRadius::Capture && cfg.clusterRadius.size() == cfg.centroids.rows())
                for (Index k = 0; k < radii.size(); ++k)
                    radii(k) = std::clamp(hw - cfg.clusterRadius(k), P.team.gcs.captureMinFrac * hw,
                                          P.team.gcs.setRadiusFrac * hw);
            gcsSets = gcs::convexSets(cfg.centroids, radii, bounds, P.team.gcs.nPoly);
        }
        team.info.gcsClusters.assign(N, {});
        team.info.gcsWaypoints.assign(N, Path2(0, 2));
        team.info.gcsLength.assign(N, 0.0);
        team.info.gcsLowerBound.assign(N, 0.0);
        team.info.gcsGap.assign(N, 0.0);
        team.info.gcsSolves.assign(N, 0);
        team.info.gcsTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - tG).count();
        if (!gcsResidual)
            for (std::size_t a = 0; a < N; ++a) planGcs(a, cfg.rewards, ownOf(a));
    }
    // sector seed: coverage orienteering over a sector grid, planned per agent
    // inside the first sweep on the belief the agents already planned leave
    // unswept (planning::solveSectorOrienteering)
    const bool useSector = uses(InitStrategy::Sector);
    if (useSector) {
        team.info.sectorWaypoints.assign(N, Path2(0, 2));
        team.info.sectorReward.assign(N, 0.0);
        team.info.sectorLength.assign(N, 0.0);
    }
    auto planSector = [&](std::size_t a, const Probs& pr) {
        const auto tS = std::chrono::steady_clock::now();
        // the exploration grid (50 m) when there is one, else the fast grid
        const FastGrid& Gs = hasX ? *cfg.Gx : G;
        const MatX& Lo = hasX ? pr.LoX : pr.Lo;
        const MatX W = (Gs.prior.array() * Lo.array().exp()).matrix();
        const Eigen::Vector4d bounds =
            cfg.bounds ? *cfg.bounds
                       : Eigen::Vector4d(Gs.x(0) - 0.5 * Gs.hg, Gs.x(Gs.nx - 1) + 0.5 * Gs.hg, Gs.y(0) - 0.5 * Gs.hg,
                                         Gs.y(Gs.ny - 1) + 0.5 * Gs.hg);
        const std::optional<Vec2> goal = cfg.modes[a] == EndpointMode::ReturnHome ? std::optional<Vec2>(cfg.starts[a])
                                         : cfg.modes[a] == EndpointMode::FixedDest ? cfg.dests[a]
                                                                                    : std::nullopt;
        const planning::SectorRoute sr = planning::solveSectorOrienteering(
            W, Gs.x0, Gs.y0, Gs.hg, bounds, cfg.kernels[a]->accurate.halfWidth, cfg.starts[a], goal, cfg.L[a],
            P.team.sector);
        team.info.sectorWaypoints[a] = sr.wp;
        team.info.sectorReward[a] = sr.reward;
        team.info.sectorLength[a] = sr.length;
        team.info.sectorTime += std::chrono::duration<double>(std::chrono::steady_clock::now() - tS).count();
        if (verbose)
            std::fprintf(stderr, "  agent %zu sector route: %lld of %lld sectors, %.0f m (budget %.0f), reward %.4f, %.2f s\n",
                         a + 1, static_cast<long long>(sr.wp.rows()), static_cast<long long>(sr.nCandidates), sr.length,
                         sr.budget, sr.reward, sr.seconds);
        return sr.wp;
    };
    team.info.seedJ0.assign(N, std::vector<double>(strategies.size(), kNaN));
    team.info.seedJ1.assign(N, std::vector<double>(strategies.size(), kNaN));

    for (int sweep = 1; sweep <= P.team.coordinationSweeps; ++sweep) {
        for (std::size_t a = 0; a < N; ++a) {
            Probs pr;
            makeProbs(a, pr);
            if (verbose) std::fprintf(stderr, "  agent %zu, sweep %d:\n", a + 1, sweep);
            if (sweep == 1) {
                // coordination-aware GCS: re-score the clusters with the belief
                // the agents already planned leave unswept (and, with
                // ignorePartition, offer this agent every cluster)
                if (useGcs && gcsCoverage) {
                    // the belief the agents already planned leave unswept, per cell
                    const MatX W = (G.prior.array() * pr.Lo.array().exp()).matrix();
                    planGcs(a, covCells.cellMass(W), {});
                    for (std::size_t e = 0; e < covExtra.size(); ++e) {
                        const auto tE = std::chrono::steady_clock::now();
                        GcsParams go = P.team.gcs;
                        go.coveragePointFrac = P.team.gcs.coverageExtra[e].second;
                        const std::optional<Vec2> goal =
                            cfg.modes[a] == EndpointMode::ReturnHome ? std::optional<Vec2>(cfg.starts[a])
                            : cfg.modes[a] == EndpointMode::FixedDest ? cfg.dests[a] : std::nullopt;
                        const gcs::AgentGcsResult ge = gcs::planAgentGcsCoverage(
                            covExtra[e], covExtra[e].cellMass(W), cfg.starts[a], goal,
                            P.team.orderBudgetFrac * cfg.L[a], 1.0 / P.curve.maxCurvature, go);
                        gcsAlt[a].insert(gcsAlt[a].begin() + static_cast<std::ptrdiff_t>(std::min(e, gcsAlt[a].size())), ge.wp);
                        team.info.gcsTime += std::chrono::duration<double>(std::chrono::steady_clock::now() - tE).count();
                    }
                } else if (useGcs && gcsResidual) {
                    VecX rw = cfg.rewards;
                    if (P.team.gcs.residualRewards) {
                        rw.setZero();
                        const MatX W = (G.prior.array() * pr.Lo.array().exp()).matrix();
                        for (Index c = 0; c < G.nx; ++c)
                            for (Index r = 0; r < G.ny; ++r) {
                                const int k = cfg.clusterOfPixel(r, c);
                                if (k >= 0) rw(k) += W(r, c);
                            }
                    }
                    std::vector<Index> ids;
                    if (P.team.gcs.ignorePartition) {
                        for (Index k = 0; k < cfg.centroids.rows(); ++k)
                            if (rw(k) >= P.team.gcs.minResidualFrac * cfg.rewards(k)) ids.push_back(k);
                    } else {
                        ids = ownOf(a);
                    }
                    planGcs(a, rw, ids);
                }
                // multi-start: the plan's cluster-ordered seed and the greedy
                // information-pursuit seed; keep whichever optimises better
                // the seeds: one per strategy, and the GCS's next-best routes
                // when nSeeds > 1
                std::vector<InitStrategy> sq;
                std::vector<Path2> wq;
                for (const InitStrategy st : strategies) {
                    if (st == InitStrategy::Gcs) {
                        sq.push_back(st);
                        wq.push_back(gcsWp[a]);
                        for (int k = 1; k < P.team.gcs.nSeeds && k - 1 < static_cast<int>(gcsAlt[a].size()); ++k) {
                            sq.push_back(st);
                            wq.push_back(gcsAlt[a][static_cast<std::size_t>(k - 1)]);
                        }
                    } else if (st == InitStrategy::Sector) {
                        sq.push_back(st);
                        wq.push_back(planSector(a, pr));
                    } else {
                        sq.push_back(st);
                        wq.push_back(st == InitStrategy::Clusters ? gatherCentroids(cfg, team.wp[a]) : Path2(0, 2));
                    }
                }
                std::vector<Candidate> cands(sq.size());
                team.info.seedJ0[a].assign(sq.size(), kNaN);
                team.info.seedJ1[a].assign(sq.size(), kNaN);
#if defined(MTLC_HAVE_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
                for (int q = 0; q < static_cast<int>(sq.size()); ++q) {
                    const auto uq = static_cast<std::size_t>(q);
                    const bool corr = !P.team.corridorGcsOnly || sq[uq] == InitStrategy::Gcs;
                    if (seedOnly) {
                        const InitResult init = initParametricSpline(cfg.starts[a], wq[uq], cfg.L[a], cfg.modes[a],
                                                                     cfg.dests[a], P.curve, pr.prob);
                        cands[uq].rep = init.rep;
                        cands[uq].seed = init.path;
                        cands[uq].v = init.v0;
                        if (P.infoAware.scoreIters > 0 || P.infoAware.scoreExploreIters > 0) {
                            // a short solve: a better predictor of the optimised objective
                            OptimizerParams so = P.optimizer;
                            so.verbose = false;
                            so.maxIter = std::max(1, P.infoAware.scoreIters);
                            so.alOuter = 1;
                            cands[uq].v = optimization::optimizeAgentCurve(init.v0, init.rep, pr.prob,
                                                                           hasX ? &pr.probX : nullptr, so,
                                                                           P.infoAware.scoreExploreIters,
                                                                           denseStep, nullptr);
                        }
                        cands[uq].J = optimization::objectiveResidualBelief(cands[uq].v, init.rep, pr.prob);
                        team.info.seedJ0[a][uq] = cands[uq].J;
                    } else {
                        cands[uq] = solveSeed(a, pr, wq[uq], P.optimizer.exploreIter, &team.info.seedJ0[a][uq], corr);
                    }
                    team.info.seedJ1[a][uq] = cands[uq].J;
                }
                std::size_t best = 0;
                for (std::size_t q = 0; q < cands.size(); ++q) {
                    if (verbose)
                        std::fprintf(stderr, "      seed '%s': J %.5f -> %.5f\n", toString(sq[q]),
                                     team.info.seedJ0[a][q], cands[q].J);
                    if (cands[q].J < cands[best].J) best = q;
                }
                if (sq[best] == InitStrategy::Gcs && !gcsCoverage) team.wp[a] = gcsCl[a];
                if (sq[best] == InitStrategy::Gcs && gcsCoverage) team.wp[a].clear();
                team.v[a] = cands[best].v;
                team.rep[a] = cands[best].rep;
                team.lastOpt[a] = cands[best].info;
                team.seedPath[a] = cands[best].seed;
                if (P.team.corridorMu > 0.0 && P.team.corridorKeep &&
                    (!P.team.corridorGcsOnly || sq[best] == InitStrategy::Gcs))
                    corridorOf[a] = cands[best].seed;
                team.initStrategy[a] = toString(sq[best]);
            } else {
                AgentProblem pc = pr.prob, pcX = pr.probX;
                if (corridorOf[a].rows() > 0)
                    for (AgentProblem* q : {&pc, &pcX}) {
                        q->corridor = &corridorOf[a];
                        q->corridorMu = P.team.corridorMu;
                        q->corridorScale = P.team.corridorScale;
                    }
                team.v[a] = optimization::optimizeAgentCurve(team.v[a], team.rep[a], pc,
                                                             hasX ? &pcX : nullptr, P.optimizer,
                                                             P.optimizer.exploreIterWarm, denseStep, &team.lastOpt[a]);
            }
            MatX LX;
            deposit(a, team.v[a], team.rep[a], team.lam[a], LX);
            if (hasX) lamX[a] = LX;
        }
        team.info.Jhist.push_back(teamJ(G, team.lam));
        team.info.stageNames.push_back(seedOnly ? std::string("seeds") : "coordination sweep " + std::to_string(sweep));
        if (verbose) std::fprintf(stderr, "  team J after sweep %d: %.5f\n", sweep, team.info.Jhist.back());
        if (seedOnly) break;
    }
    if (seedOnly) {
        team.info.Jstatic = team.info.Jfinal = team.info.Jhist.back();
        team.vStatic = team.v;
        team.repStatic = team.rep;
        team.info.staticTime =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - tStatic0).count();
        return team;
    }
    team.info.Jstatic = team.info.Jhist.empty() ? teamJ(G, team.lam) : team.info.Jhist.back();
    team.vStatic = team.v;
    team.repStatic = team.rep;
    const auto tRealloc0 = std::chrono::steady_clock::now();
    team.info.staticTime = std::chrono::duration<double>(tRealloc0 - tStatic0).count();

    // ---- 2-4. audit, slack, reallocation ------------------------------------
    team.info.audit = auditTeam(cfg, team, P);
    if (P.team.reallocate) {
        int nTrials = 0;
        for (int rnd = 1; rnd <= P.team.reallocRounds; ++rnd) {
            ClusterAudit A = team.info.audit;
            std::vector<Index> pool = A.unserviced;
            if (pool.empty()) {
                if (verbose) std::fprintf(stderr, "  realloc round %d: no unserviced clusters.\n", rnd);
                break;
            }
            std::stable_sort(pool.begin(), pool.end(),
                             [&](Index x, Index y) { return A.residMass(x) > A.residMass(y); });
            if (verbose) std::fprintf(stderr, "  realloc round %d: %zu unserviced cluster(s)\n", rnd, pool.size());
            bool anyAccepted = false;
            for (const Index k : pool) {
                if (nTrials >= P.team.maxReallocTrials) break;
                std::vector<std::size_t> cand;
                for (std::size_t a = 0; a < N; ++a)
                    if (A.isSlack[a]) cand.push_back(a);
                if (cand.empty()) break;
                std::stable_sort(cand.begin(), cand.end(), [&](std::size_t x, std::size_t y) {
                    return A.distToCurve(k, static_cast<Index>(x)) < A.distToCurve(k, static_cast<Index>(y));
                });
                cand.resize(std::min<std::size_t>(cand.size(), static_cast<std::size_t>(P.team.maxCandidatesPerCluster)));
                for (const std::size_t a : cand) {
                    if (nTrials >= P.team.maxReallocTrials) break;
                    ++nTrials;
                    const double Jold = teamJ(G, team.lam);
                    Probs pr;
                    makeProbs(a, pr);
                    const auto& rs = P.team.reallocInitStrategies;
                    std::vector<Candidate> cands(rs.size());
                    std::vector<std::vector<Index>> wpq(rs.size());
                    for (std::size_t q = 0; q < rs.size(); ++q)
                        wpq[q] = rs[q] == ReallocStrategy::Direct ? std::vector<Index>{k}
                                                                  : insertCheapest(cfg, a, team.wp[a], k);
#if defined(MTLC_HAVE_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
                    for (int q = 0; q < static_cast<int>(rs.size()); ++q) {
                        const auto uq = static_cast<std::size_t>(q);
                        // the splice needs the curvature representation; a
                        // B-spline curve falls back to the Insert seed
                        const bool pathOk = team.rep[a].type == CurveRepresentation::Curvature;
                        Candidate c = rs[uq] == ReallocStrategy::Path && pathOk
                                          ? solvePathSeed(a, pr, k)
                                          : solveSeed(a, pr, gatherCentroids(cfg, wpq[uq]), P.optimizer.exploreIter);
                        if (c.skipped) {
                            cands[uq] = std::move(c);
                            continue;
                        }
                        c.lam = optimization::agentDeposit(c.v, c.rep, G, cfg.kernels[a]->accurate);
                        c.J = (G.prior.array() * (pr.Lo + c.lam).array().exp()).sum();
                        c.feasible = curveFeasible(c.v, c.rep, denseStep);
                        cands[uq] = std::move(c);
                    }
                    double Jnew = kInf;
                    std::size_t best = rs.size();
                    for (std::size_t q = 0; q < rs.size(); ++q)
                        if (cands[q].feasible && cands[q].J < Jnew) {
                            Jnew = cands[q].J;
                            best = q;
                        }
                    const bool acc = best < rs.size() && Jnew < Jold - P.team.reallocEps;
                    ReallocTrial tr;
                    tr.round = rnd;
                    tr.cluster = k;
                    tr.agent = static_cast<int>(a);
                    tr.Jold = Jold;
                    tr.Jnew = Jnew;
                    tr.accepted = acc;
                    tr.strategy = best < rs.size() ? toString(rs[best]) : "";
                    team.info.log.push_back(tr);
                    if (verbose)
                        std::fprintf(stderr, "    cluster %lld -> agent %zu (%s): team J %.5f -> %.5f  %s\n",
                                     static_cast<long long>(k), a + 1, tr.strategy.c_str(), Jold, Jnew,
                                     acc ? "ACCEPTED" : "rejected");
                    if (acc) {
                        team.v[a] = cands[best].v;
                        team.rep[a] = cands[best].rep;
                        team.lam[a] = cands[best].lam;
                        if (hasX) lamX[a] = optimization::agentDeposit(team.v[a], team.rep[a], *cfg.Gx, cfg.kernels[a]->explore);
                        team.wp[a] = wpq[best];
                        team.lastOpt[a] = cands[best].info;
                        team.seedPath[a] = cands[best].seed;
                        team.initStrategy[a] = std::string("realloc:") + tr.strategy;
                        anyAccepted = true;
                        A = auditTeam(cfg, team, P);
                        break;
                    }
                }
            }
            if (anyAccepted) {
                // warm coordination sweep so the others adapt to the new split
                for (std::size_t a = 0; a < N; ++a) {
                    Probs pr;
                    makeProbs(a, pr);
                    team.v[a] = optimization::optimizeAgentCurve(team.v[a], team.rep[a], pr.prob,
                                                                 hasX ? &pr.probX : nullptr, P.optimizer,
                                                                 P.optimizer.exploreIterWarm, denseStep, &team.lastOpt[a]);
                    MatX LX;
                    deposit(a, team.v[a], team.rep[a], team.lam[a], LX);
                    if (hasX) lamX[a] = LX;
                }
            }
            team.info.Jhist.push_back(teamJ(G, team.lam));
            team.info.stageNames.push_back("realloc round " + std::to_string(rnd));
            team.info.audit = auditTeam(cfg, team, P);
            if (verbose) std::fprintf(stderr, "  team J after realloc round %d: %.5f\n", rnd, team.info.Jhist.back());
            if (!anyAccepted) break;
        }
    }
    team.info.Jfinal = teamJ(G, team.lam);
    team.info.reallocTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - tRealloc0).count();
    return team;
}

}  // namespace mtl::curve::curve_planning
