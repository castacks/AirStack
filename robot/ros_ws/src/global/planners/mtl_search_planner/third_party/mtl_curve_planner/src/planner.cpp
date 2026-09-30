#include "mtl_curve/planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>

#include "mtl_curve/core/kmeans.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/curve_planning/team_curves.hpp"
#include "mtl_curve/mapping/cells.hpp"
#include "mtl_curve/optimization/fast_grid.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
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
        out.agentOfCluster = partition(out.clusters);

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

        // --- the team solve ---------------------------------------------------
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
        cfg.centroids = out.clusters.centroids;
        cfg.rewards = out.clusters.reward;
        cfg.assign = out.agentOfCluster;
        cfg.clusterOfPixel = curve_planning::clusterOfPixel(G, cells.centers, out.clusters.cellCluster, P.targetCellSize);

        curve_planning::TeamSolution team = curve_planning::reallocateCurveClusters(cfg, P);

        // --- fly it ----------------------------------------------------------
        out.trajectories = trajectory::generateCurveTrajectories(team.v, team.rep, h, sweeps, P.avgDroneSpeed,
                                                                 P.dt, P.gimbal, out.timeVec);
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
