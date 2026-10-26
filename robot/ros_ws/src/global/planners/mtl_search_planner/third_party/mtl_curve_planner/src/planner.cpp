#include "mtl_curve/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>

#include "mtl_curve/core/kmeans.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/curve_planning/info_aware.hpp"
#include "mtl_curve/curve_planning/team_curves.hpp"
#include "mtl_curve/gcs/gcs.hpp"
#include "mtl_curve/mapping/cells.hpp"
#include "mtl_curve/optimization/fast_grid.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
#include "mtl_curve/quadrotor/quad_trajectory.hpp"
#include "mtl_curve/sensing/sweep.hpp"
#include "mtl_curve/trajectory/curve_trajectory.hpp"

namespace mtl::curve {

struct Planner::Impl {
    PlannerParams params;

    explicit Impl(PlannerParams p) : params(std::move(p)) {
        params.finalize();
        params.validate();
    }

    std::vector<double> budgets() const {
        std::vector<double> b(static_cast<std::size_t>(params.numAgents), params.budgetDist());
        if (!params.perAgentBudgetDist.empty()) b = params.perAgentBudgetDist;
        return b;
    }
    std::vector<double> altitudes() const {
        std::vector<double> h;
        for (int a = 0; a < params.numAgents; ++a) h.push_back(params.agentAltitude(a));
        return h;
    }

    /// The initial partition: exactly cpp_planner's (k-means on the centroids,
    /// same seed), so both planners start from the same split.
    std::vector<int> partition(const ClusterSet& clusters) const {
        const Index K = clusters.size();
        std::vector<int> owner(static_cast<std::size_t>(K), 0);
        if (K == 0 || K < params.numAgents) return owner;
        core::KMeansOptions ko;
        ko.replicates = params.kmeansReplicatesAgents;
        ko.maxIter    = params.cluster.kmeansMaxIter;
        ko.seed       = params.rngSeed * 104729ULL + 17ULL;
        return core::kmeans(clusters.centroids, params.numAgents, ko).assignment;
    }

    PlanningResult solve(const CellSet& cells, ClusterSet clusters, const std::vector<Vec2>& agentStarts,
                         const BeliefField* belief) const {
        const PlannerParams& P = params;
        if (static_cast<int>(agentStarts.size()) < P.numAgents) {
            throw std::invalid_argument("mtl::curve::Planner::plan: fewer launch points than agents (need " +
                                        std::to_string(P.numAgents) + ")");
        }
        if (!clusters.empty() && static_cast<Index>(clusters.cellCluster.size()) != cells.size())
            throw std::invalid_argument("mtl::curve::Planner::plan: clusters.cellCluster must have one entry per cell");

        PlanningResult out;
        out.cells = cells;
        if (clusters.reward.size() != clusters.size() ||
            static_cast<Index>(clusters.cellIdx.size()) != clusters.size())
            mapping::computeClusterRewards(cells.mass, clusters);
        out.clusters = std::move(clusters);

        const auto N = static_cast<std::size_t>(P.numAgents);
        const std::vector<double> L = budgets();
        const std::vector<double> h = altitudes();

        // --- sweep and calibrated kernel per agent (cached by altitude) -------
        std::vector<SweepParams> sweeps(N);
        std::map<double, SwathKernelSet> cache;
        std::vector<const SwathKernelSet*> kernels(N);
        double Rk = 0.0, RkX = 0.0;
        for (std::size_t a = 0; a < N; ++a) {
            sweeps[a] = sensing::computeSweepParams(h[a], P.sweep, P.gimbal, P.sensor, P.sensorTiltAngle);
            auto it = cache.find(h[a]);
            if (it == cache.end()) {
                it = cache.emplace(h[a], optimization::calibrateSwathKernel(h[a], sweeps[a], P.sensor, P.fov,
                                                                            P.avgDroneSpeed, P.dt, P.kernel)).first;
                if (P.verbose) {
                    const SwathKernel& K = it->second.accurate;
                    std::fprintf(stderr,
                                 "  swath kernel: h %.0f m, tilt %.0f deg, sweep +/-%.1f deg @ %.2f Hz -> "
                                 "P>=0.5 half-width %.0f m (reach %.0f m), straight-line model err %.3f\n",
                                 h[a], rad2deg(P.sensorTiltAngle), rad2deg(sweeps[a].alphaMax), sweeps[a].freq,
                                 K.halfWidth, K.reach, K.fitMaxErrP);
                }
            }
            kernels[a] = &it->second;
            Rk = std::max(Rk, it->second.accurate.Rk);
            RkX = std::max(RkX, it->second.explore.Rk);
        }

        // --- the fast grid and the exploration grid --------------------------
        FastGrid G, Gx;
        const bool explore = P.optimizer.exploreIter > 0 || P.optimizer.exploreIterWarm > 0;
        if (belief) {
            const double res = belief->gridResX();
            G = optimization::buildFastGrid(*belief, static_cast<int>(std::lround(P.team.fastGridStep / res)), Rk);
            if (explore)
                Gx = optimization::buildFastGrid(*belief, static_cast<int>(std::lround(P.team.exploreGridStep / res)), RkX);
        } else {
            G = optimization::rasterizeCells(cells, P.mapSize, P.team.fastGridStep, P.targetCellSize, Rk);
            if (explore)
                Gx = optimization::rasterizeCells(cells, P.mapSize, P.team.exploreGridStep, P.targetCellSize, RkX);
        }
        if (P.verbose)
            std::fprintf(stderr, "  fast objective grid: %lld x %lld at %.0f m\n", static_cast<long long>(G.ny),
                         static_cast<long long>(G.nx), G.hg);

        // --- clusters: cpp_planner's (Radius) or the camera kernel's (Kernel) ---
        Eigen::MatrixXi kernelPixelCluster;
        double clusterTime = 0.0;
        if (P.team.clusterMode == ClusterMode::Kernel) {
            const auto tK = std::chrono::steady_clock::now();
            double hw = kInf;
            for (const SwathKernelSet* ks : kernels) hw = std::min(hw, ks->accurate.halfWidth);
            gcs::KernelClusters KC = gcs::kernelClusters(G, hw, P.team.kernelClusters);
            ClusterSet kc;
            kc.centroids = KC.centroids;
            kc.reward = KC.info;
            kc.cellIdx.assign(static_cast<std::size_t>(KC.centroids.rows()), {});
            kc.cellCluster.assign(static_cast<std::size_t>(cells.size()), 0);
            for (Index i = 0; i < cells.size(); ++i) {
                // the cell joins the box that took its fast pixel, else the nearest centre
                const Index c = std::clamp<Index>(static_cast<Index>(std::lround((cells.centers(i, 0) - G.x0) / G.hg)), 0, G.nx - 1);
                const Index r = std::clamp<Index>(static_cast<Index>(std::lround((cells.centers(i, 1) - G.y0) / G.hg)), 0, G.ny - 1);
                int k = KC.pixelCluster(r, c);
                if (k < 0 && KC.centroids.rows() > 0) {
                    Index kb = 0;
                    (KC.centroids.rowwise() - cells.centers.row(i)).rowwise().squaredNorm().minCoeff(&kb);
                    k = static_cast<int>(kb);
                }
                kc.cellCluster[static_cast<std::size_t>(i)] = std::max(k, 0);
                if (k >= 0) kc.cellIdx[static_cast<std::size_t>(k)].push_back(i);
            }
            out.clusters = std::move(kc);
            kernelPixelCluster = std::move(KC.pixelCluster);
            clusterTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - tK).count();
            if (P.verbose)
                std::fprintf(stderr, "  kernel clusters: %lld (box %.0f m), %.1f%% of the prior, %.3f s\n",
                             static_cast<long long>(out.clusters.size()), 2.0 * KC.boxHalf,
                             100.0 * out.clusters.reward.sum(), clusterTime);
        }
        // --- the team solve over one abstraction ---------------------------------
        // (also used, seedOnly, to score the information-aware candidates)
        auto teamConfig = [&](const ClusterSet& cl, const std::vector<int>& assign, bool kernelPix) {
            curve_planning::TeamConfig cfg;
            cfg.starts.assign(agentStarts.begin(), agentStarts.begin() + P.numAgents);
            cfg.L = L;
            for (int a = 0; a < P.numAgents; ++a) {
                const EndpointMode m = P.curve.modeOf(a);
                cfg.modes.push_back(m);
                cfg.dests.push_back(m == EndpointMode::FixedDest
                                        ? std::optional<Vec2>(P.curve.destinations[static_cast<std::size_t>(a)])
                                        : std::nullopt);
            }
            cfg.G = &G;
            cfg.Gx = explore ? &Gx : nullptr;
            cfg.kernels = kernels;
            cfg.centroids = cl.centroids;
            cfg.rewards = cl.reward;
            cfg.assign = assign;
            cfg.clusterOfPixel = kernelPix ? kernelPixelCluster
                                           : curve_planning::clusterOfPixel(G, cells.centers, cl.cellCluster,
                                                                            P.targetCellSize);
            cfg.bounds = Eigen::Vector4d(0.0, P.mapSize, 0.0, P.mapSize);
            cfg.clusterRadius = VecX::Zero(cl.size());
            for (Index k = 0; k < cl.size() && k < static_cast<Index>(cl.cellIdx.size()); ++k)
                for (const Index i : cl.cellIdx[static_cast<std::size_t>(k)])
                    cfg.clusterRadius(k) = std::max(cfg.clusterRadius(k), (cells.centers.row(i) - cl.centroids.row(k)).norm());
            return cfg;
        };
        const bool kernelMode = P.team.clusterMode == ClusterMode::Kernel;

        // --- information-aware cluster search (PlannerParams::infoAware) -------
        bool clusterDependent = false;
        for (const InitStrategy s : P.team.initStrategies)
            clusterDependent = clusterDependent || s == InitStrategy::Clusters || s == InitStrategy::Gcs;
        curve_planning::TeamSolution team;
        bool solved = false;
        bool chosenKernelPix = kernelMode;
        if (P.infoAware.enabled && clusterDependent && !cells.empty()) {
            const auto tIA = std::chrono::steady_clock::now();
            const InfoAwareParams& ia = P.infoAware;
            InfoAwareReport rep;
            rep.enabled = true;
            double hw = kInf;
            for (const SwathKernelSet* ks : kernels) hw = std::min(hw, ks->accurate.halfWidth);
            rep.swathHalfWidth = hw;
            std::vector<curve_planning::AbstractionCandidate> base =
                curve_planning::infoAwareCandidates(cells, out.clusters, hw, P, &rep.basins);

            struct Cand {
                std::string label;
                double reach = 0.0;
                ClusterSet cl;
                PlannerParams q;
                bool kernelPix = false;
                double seedJ = kInf;
                std::vector<Index> visited;
            };
            std::vector<Cand> cs;
            for (int r = 0; r <= ia.restarts; ++r)
                for (std::size_t i = 0; i < base.size(); ++i) {
                    Cand c;
                    c.label = base[i].label + (r ? "~s" + std::to_string(r) : std::string());
                    c.reach = base[i].reach;
                    c.cl = base[i].clusters;
                    c.q = P;
                    c.q.verbose = false;
                    c.kernelPix = kernelMode && i == 0;
                    if (r > 0) {
                        c.q.team.orienteering.seed += 7919ULL * static_cast<std::uint64_t>(r);
                        c.q.team.orienteering.nStarts = std::max(c.q.team.orienteering.nStarts, 4);
                        c.q.team.gcs.seed += static_cast<std::uint64_t>(r);
                    }
                    cs.push_back(std::move(c));
                }
            auto score = [&](Cand& c) {
                const curve_planning::TeamConfig cfg = teamConfig(c.cl, partition(c.cl), c.kernelPix);
                PlannerParams qs = c.q;
                if (ia.scoreWithClusters) qs.team.initStrategies = {InitStrategy::Clusters};
                const curve_planning::TeamSolution t = curve_planning::reallocateCurveClusters(cfg, qs, true);
                c.seedJ = t.info.Jstatic;
                c.visited.clear();
                for (const auto& w : t.wp) c.visited.insert(c.visited.end(), w.begin(), w.end());
            };
            auto record = [&](const Cand& c, bool accepted) {
                InfoAwareCandidate r;
                r.label = c.label;
                r.reach = c.reach;
                r.clusters = c.cl.size();
                r.seedJ = c.seedJ;
                r.accepted = accepted;
                rep.candidates.push_back(r);
            };
            std::size_t bestI = 0;
            for (std::size_t i = 0; i < cs.size(); ++i) {
                score(cs[i]);
                record(cs[i], false);
                if (cs[i].seedJ < cs[bestI].seedJ - 1e-12) bestI = i;
            }
            rep.plainSeedJ = cs[0].seedJ;
            Cand inc = cs[bestI];

            // --- peel / split / merge on the incumbent ------------------------
            if (ia.splitMerge && ia.maxMoves > 0) {
                std::uint64_t mseed = P.rngSeed * 2654435761ULL + 99ULL;
                auto moves = curve_planning::infoAwareMoves(inc.cl, inc.visited, cells, inc.reach, ia, P.cluster, mseed);
                std::size_t next = 0;
                while (rep.movesTried < ia.maxMoves && next < moves.size()) {
                    Cand m = inc;
                    m.label = inc.label + "+" + moves[next].first;
                    m.cl = std::move(moves[next].second);
                    m.kernelPix = false;
                    ++next;
                    ++rep.movesTried;
                    score(m);
                    const bool better = m.seedJ < inc.seedJ - 1e-9;
                    record(m, better);
                    if (better) {
                        ++rep.movesAccepted;
                        inc = std::move(m);
                        moves = curve_planning::infoAwareMoves(inc.cl, inc.visited, cells, inc.reach, ia, P.cluster,
                                                               mseed + static_cast<std::uint64_t>(rep.movesTried));
                        next = 0;
                    }
                }
            }

            // --- full solve of the best K -----------------------------------------
            std::vector<const Cand*> top{&inc};
            {
                std::vector<std::size_t> rk(cs.size());
                for (std::size_t i = 0; i < rk.size(); ++i) rk[i] = i;
                std::stable_sort(rk.begin(), rk.end(), [&](std::size_t a, std::size_t b) { return cs[a].seedJ < cs[b].seedJ; });
                for (const std::size_t i : rk) {
                    if (static_cast<int>(top.size()) >= ia.optimizeTopK) break;
                    if (cs[i].label == inc.label) continue;
                    top.push_back(&cs[i]);
                }
            }
            double bestJ = kInf;
            const Cand* win = top[0];
            for (const Cand* c : top) {
                curve_planning::TeamConfig cfg = teamConfig(c->cl, partition(c->cl), c->kernelPix);
                PlannerParams q = c->q;
                q.verbose = P.verbose;
                curve_planning::TeamSolution t = curve_planning::reallocateCurveClusters(cfg, q, false);
                const double J = t.info.Jfinal;
                for (InfoAwareCandidate& r : rep.candidates)
                    if (r.label == c->label) r.finalJ = J;
                if (J < bestJ) {
                    bestJ = J;
                    win = c;
                    team = std::move(t);
                }
            }
            solved = true;
            rep.chosen = win->label;
            rep.chosenSeedJ = win->seedJ;
            rep.chosenReach = win->reach;
            out.clusters = win->cl;
            chosenKernelPix = win->kernelPix;
            rep.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - tIA).count();
            if (P.verbose)
                std::fprintf(stderr,
                             "  info-aware clusters: %zu scored, %d/%d moves accepted; chose %s (%lld clusters, "
                             "seed J %.5f vs plain %.5f), %.1f s\n",
                             rep.candidates.size(), rep.movesAccepted, rep.movesTried, rep.chosen.c_str(),
                             static_cast<long long>(out.clusters.size()), rep.chosenSeedJ, rep.plainSeedJ,
                             rep.seconds);
            out.infoAware = std::move(rep);
        }
        out.agentOfCluster = partition(out.clusters);
        curve_planning::TeamConfig cfg = teamConfig(out.clusters, out.agentOfCluster, chosenKernelPix);
        if (!solved) team = curve_planning::reallocateCurveClusters(cfg, P);
        team.info.clusterTime = clusterTime;

        // --- fly it ----------------------------------------------------------
        out.trajectories = trajectory::generateCurveTrajectories(team.v, team.rep, h, sweeps, P.avgDroneSpeed,
                                                                 P.dt, P.gimbal, out.timeVec);
        // --- the same plan for a quadrotor with a body-fixed camera -----------
        if (P.quad.enabled)
            out.quad = quadrotor::generateQuadTrajectories(out.trajectories, P.avgDroneSpeed, P.dt,
                                                           P.sensorTiltAngle, P.sensor.beta, P.quad,
                                                           out.quadTimeVec);
        bool changed = false;
        for (std::size_t a = 0; a < N; ++a) changed = changed || !(team.v[a].size() == team.vStatic[a].size() &&
                                                                    team.v[a] == team.vStatic[a]);
        if (changed) {
            VecX tS;
            out.staticTrajectories = trajectory::generateCurveTrajectories(team.vStatic, team.repStatic, h, sweeps,
                                                                           P.avgDroneSpeed, P.dt, P.gimbal, tS);
        }

        out.plans.resize(N);
        for (std::size_t a = 0; a < N; ++a) {
            AgentPlan& pl = out.plans[a];
            const AgentTrajectory& t = out.trajectories[a];
            pl.agentId = static_cast<int>(a);
            pl.start = cfg.starts[a];
            pl.rep = team.rep[a];
            pl.v = team.v[a];
            pl.vStatic = team.vStatic[a];
            pl.waypointClusters = team.wp[a];
            pl.initStrategy = team.initStrategy[a];
            pl.altitude = h[a];
            pl.sweep = sweeps[a];
            pl.swathHalfWidth = kernels[a]->accurate.halfWidth;
            pl.budget = L[a];
            pl.flownLength = t.length;
            pl.flightTime = static_cast<double>(t.steps - 1) * P.dt;
            pl.budgetUsed = t.length / L[a];
            pl.maxKappa = t.maxKappaDiscrete;
            pl.endpointError = pl.rep.pGoal ? (t.endPt - *pl.rep.pGoal).norm() : 0.0;
            pl.feasible = pl.maxKappa <= P.curve.maxCurvature + 1e-4 &&
                          std::abs(t.length - L[a]) <= 0.002 * L[a] && pl.endpointError < 1.0;
            pl.lastOpt = team.lastOpt[a];
            pl.seedPath = team.seedPath[a];
        }
        // --- per-cell coverage (fast model at the cell centres) ---------------
        VecX teamLam = VecX::Zero(cells.size());
        for (std::size_t a = 0; a < N; ++a) {
            const CurveSamples C = curve_planning::evalParametricCurve(team.v[a], team.rep[a]);
            const VecX la = optimization::swathKernelAtPoints(C.pts, C.tan, C.ds, kernels[a]->accurate, cells.centers);
            teamLam += la;
            for (Index i = 0; i < cells.size(); ++i)
                if (1.0 - std::exp(la(i)) >= 0.5) out.plans[a].servicedCellIdx.push_back(i);
        }
        out.team = std::move(team.info);
        for (Index i = 0; i < cells.size(); ++i) {
            if (1.0 - std::exp(teamLam(i)) >= 0.5) {
                out.team.servicedCellIdx.push_back(i);
                out.team.info += cells.mass(i);
            } else {
                out.team.unservicedCellIdx.push_back(i);
            }
        }
        out.team.infoTotal = cells.mass.sum();
        out.team.infoFraction = out.team.infoTotal > 0.0 ? out.team.info / out.team.infoTotal : 0.0;
        out.teamLambda = MatX::Zero(G.ny, G.nx);
        for (const MatX& l : team.lam) out.teamLambda += l;
        out.grid = std::move(G);
        return out;
    }
};

Planner::Planner(PlannerParams params) : impl_(std::make_unique<Impl>(std::move(params))) {}
Planner::~Planner()                             = default;
Planner::Planner(Planner&&) noexcept            = default;
Planner& Planner::operator=(Planner&&) noexcept = default;

const PlannerParams& Planner::params() const noexcept { return impl_->params; }
void Planner::setParams(PlannerParams params) { impl_ = std::make_unique<Impl>(std::move(params)); }
std::vector<double> Planner::agentBudgets() const { return impl_->budgets(); }
std::vector<double> Planner::agentAltitudes() const { return impl_->altitudes(); }

PlanningResult Planner::plan(const BeliefField& belief, const std::vector<Vec2>& agentStarts) {
    const PlannerParams& P = impl_->params;
    const CellSet cells = mapping::extractValidCells(belief, P.targetCellSize, P.minimumBeliefMass, P.verbose);
    ClusterSet clusters;
    if (!cells.empty())
        clusters = mapping::clusterCells(cells.centers, P.maxClusterRadius, P.cluster, P.rngSeed, P.verbose);
    return impl_->solve(cells, std::move(clusters), agentStarts, &belief);
}

PlanningResult Planner::planFromCells(const CellSet& cells, const std::vector<Vec2>& agentStarts) {
    const PlannerParams& P = impl_->params;
    if (cells.empty()) throw std::invalid_argument("mtl::curve::Planner::planFromCells: no cells to plan over");
    ClusterSet clusters = mapping::clusterCells(cells.centers, P.maxClusterRadius, P.cluster, P.rngSeed, P.verbose);
    return impl_->solve(cells, std::move(clusters), agentStarts, nullptr);
}

PlanningResult Planner::planFromClusters(const CellSet& cells, ClusterSet clusters,
                                         const std::vector<Vec2>& agentStarts, const BeliefField* belief) {
    return impl_->solve(cells, std::move(clusters), agentStarts, belief);
}

}  // namespace mtl::curve
