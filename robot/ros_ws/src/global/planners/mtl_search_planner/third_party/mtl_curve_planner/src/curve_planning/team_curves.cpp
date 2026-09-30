#include "mtl_curve/curve_planning/team_curves.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <unordered_map>

#include "mtl_curve/curve_planning/init_spline.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/optimization/optimizer.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
#include "mtl_curve/planning/orienteering.hpp"

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

TeamSolution reallocateCurveClusters(const TeamConfig& cfg, const PlannerParams& P) {
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
    auto solveSeed = [&](std::size_t a, const Probs& pr, const Path2& wps, int exploreIter) {
        Candidate cand;
        const InitResult init = initParametricSpline(cfg.starts[a], wps, cfg.L[a], cfg.modes[a], cfg.dests[a],
                                                     P.curve, pr.prob);
        cand.rep = init.rep;
        cand.seed = init.path;
        cand.v = optimization::optimizeAgentCurve(init.v0, cand.rep, pr.prob, hasX ? &pr.probX : nullptr,
                                                  P.optimizer, exploreIter, denseStep, &cand.info);
        cand.J = cand.info.J;
        return cand;
    };

    // ---- 1. initial ordering + solve ---------------------------------------
    for (std::size_t a = 0; a < N; ++a) {
        std::vector<Index> own;
        for (std::size_t k = 0; k < cfg.assign.size(); ++k)
            if (cfg.assign[k] == static_cast<int>(a)) own.push_back(static_cast<Index>(k));
        team.wp[a] = orderClusters(cfg, a, own, P);
    }

    const auto& strategies = P.team.initStrategies;
    for (int sweep = 1; sweep <= P.team.coordinationSweeps; ++sweep) {
        for (std::size_t a = 0; a < N; ++a) {
            Probs pr;
            makeProbs(a, pr);
            if (verbose) std::fprintf(stderr, "  agent %zu, sweep %d:\n", a + 1, sweep);
            if (sweep == 1) {
                // multi-start: the plan's cluster-ordered seed and the greedy
                // information-pursuit seed; keep whichever optimises better
                std::vector<Candidate> cands(strategies.size());
#if defined(MTLC_HAVE_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
                for (int q = 0; q < static_cast<int>(strategies.size()); ++q) {
                    const auto uq = static_cast<std::size_t>(q);
                    const Path2 wps = strategies[uq] == InitStrategy::Clusters ? gatherCentroids(cfg, team.wp[a])
                                                                               : Path2(0, 2);
                    cands[uq] = solveSeed(a, pr, wps, P.optimizer.exploreIter);
                }
                std::size_t best = 0;
                for (std::size_t q = 0; q < cands.size(); ++q) {
                    if (verbose) std::fprintf(stderr, "      seed '%s': J %.5f\n", toString(strategies[q]), cands[q].J);
                    if (cands[q].J < cands[best].J) best = q;
                }
                team.v[a] = cands[best].v;
                team.rep[a] = cands[best].rep;
                team.lastOpt[a] = cands[best].info;
                team.seedPath[a] = cands[best].seed;
                team.initStrategy[a] = toString(strategies[best]);
            } else {
                team.v[a] = optimization::optimizeAgentCurve(team.v[a], team.rep[a], pr.prob,
                                                             hasX ? &pr.probX : nullptr, P.optimizer,
                                                             P.optimizer.exploreIterWarm, denseStep, &team.lastOpt[a]);
            }
            MatX LX;
            deposit(a, team.v[a], team.rep[a], team.lam[a], LX);
            if (hasX) lamX[a] = LX;
        }
        team.info.Jhist.push_back(teamJ(G, team.lam));
        team.info.stageNames.push_back("coordination sweep " + std::to_string(sweep));
        if (verbose) std::fprintf(stderr, "  team J after sweep %d: %.5f\n", sweep, team.info.Jhist.back());
    }
    team.info.Jstatic = team.info.Jhist.empty() ? teamJ(G, team.lam) : team.info.Jhist.back();
    team.vStatic = team.v;
    team.repStatic = team.rep;

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
                        wpq[q] = rs[q] == ReallocStrategy::Insert ? insertCheapest(cfg, a, team.wp[a], k)
                                                                  : std::vector<Index>{k};
#if defined(MTLC_HAVE_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
                    for (int q = 0; q < static_cast<int>(rs.size()); ++q) {
                        const auto uq = static_cast<std::size_t>(q);
                        Candidate c = solveSeed(a, pr, gatherCentroids(cfg, wpq[uq]), P.optimizer.exploreIter);
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
    return team;
}

}  // namespace mtl::curve::curve_planning
