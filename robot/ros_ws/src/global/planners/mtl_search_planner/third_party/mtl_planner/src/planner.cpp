#include "mtl/planner.hpp"

#include <algorithm>
#include <cstdio>
#include <numeric>
#include <stdexcept>

#include "mtl/core/kmeans.hpp"
#include "mtl/core/numeric.hpp"
#include "mtl/mapping/cells.hpp"
#include "mtl/planning/agent_sortie.hpp"
#include "mtl/planning/team_allocation.hpp"
#include "mtl/sensing/airframe.hpp"
#include "mtl/sensing/gimbal_scheduler.hpp"

namespace mtl {
namespace {

std::vector<Index> uniqueSorted(std::vector<Index> v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

Path2 gather(const Path2& src, const std::vector<Index>& idx) {
    Path2 out(static_cast<Index>(idx.size()), 2);
    for (std::size_t i = 0; i < idx.size(); ++i) out.row(static_cast<Index>(i)) = src.row(idx[i]);
    return out;
}

}  // namespace

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

    /// The team partition: split the macro-clusters geographically.  With fewer
    /// clusters than agents everything goes to agent 0 and the rest are
    /// grounded - which allocateBudgetedTeam's reallocation then unpicks if the
    /// budget allows.
    std::vector<int> partition(const ClusterSet& clusters) const {
        const Index K = clusters.size();
        std::vector<int> owner(static_cast<std::size_t>(K), 0);
        if (K == 0) return owner;
        if (K < params.numAgents) return owner;

        core::KMeansOptions ko;
        ko.replicates = params.kmeansReplicatesAgents;
        ko.maxIter    = params.cluster.kmeansMaxIter;
        ko.seed       = params.rngSeed * 104729ULL + 17ULL;
        const core::KMeansResult r = core::kmeans(clusters.centroids, params.numAgents, ko);
        return r.assignment;
    }

    /// Everything after the route: fly the plan, and in single-axis mode schedule
    /// the gimbal against it.
    void realiseTrajectories(const CellSet& cells, PlanningResult& out) const {
        const auto numAgents = static_cast<std::size_t>(params.numAgents);
        out.trajectories.assign(numAgents, AgentTrajectory{});

        for (std::size_t a = 0; a < numAgents; ++a) {
            const AgentPlan& plan = out.plans[a];
            AgentTrajectory& tr   = out.trajectories[a];

            if (plan.grounded()) {
                // No cluster assigned, or none reachable inside the budget.
                tr.drone  = plan.droneTraj;
                tr.sensor = plan.sensorTraj;
                tr.rpy    = sensing::computeAirframeRPY(plan.droneTraj, params.gimbal);
                continue;
            }

            if (params.singleAxisGimbal) {
                // ---- SINGLE-AXIS GIMBAL: the schedule IS the problem --------
                // The boresight is locked to a cross-track line, so WHEN each
                // centre can be observed is handed over by the geometry, and the
                // only freedom is which pass to take it on plus a few degrees of
                // pitch.  That has to be scheduled, and the scheduler bends the
                // trajectory doing it.  One solver covers both mounts:
                // sensorTiltAngle = 0 is the nadir law, and a positive tilt
                // stands the swept line off along track, shifting every service
                // instant earlier.
                const sensing::GimbalSchedule sch = sensing::optimizeDroneSensorTraj(
                    plan.droneTraj, plan.sensorTraj, plan.sensorTargets, params.maxPitchChangeRate,
                    params.gimbal);

                tr.drone       = sch.droneTraj;
                tr.sensor      = sch.sensorTraj;
                tr.rpy         = sch.rpy;
                tr.diagnostics = sch.diagnostics;
                tr.scheduled   = true;

                // Map the scheduler's missed centres back onto GLOBAL cell
                // indices, so the information report can separate "the budget
                // refused it" from "the gimbal could not bring it abeam".
                std::vector<char> missed(plan.sensorCellIdx.size(), 0);
                for (const Index m : sch.diagnostics.targetMissedIdx)
                    if (m >= 0 && m < static_cast<Index>(plan.sensorCellIdx.size()))
                        missed[static_cast<std::size_t>(m)] = 1;

                std::vector<Index> realized;
                std::vector<Index> missedCells;
                for (std::size_t i = 0; i < plan.sensorCellIdx.size(); ++i) {
                    if (missed[i]) {
                        missedCells.push_back(plan.sensorCellIdx[i]);
                    } else {
                        realized.push_back(plan.sensorCellIdx[i]);
                    }
                }
                // A cell serviced twice on the sweep counts as realized if ANY
                // of its visits was observed - setdiff semantics, as in MATLAB.
                tr.realizedCellIdx = uniqueSorted(realized);
                {
                    const std::vector<Index> missedU = uniqueSorted(missedCells);
                    std::vector<Index>       keep;
                    std::set_difference(tr.realizedCellIdx.begin(), tr.realizedCellIdx.end(),
                                        missedU.begin(), missedU.end(), std::back_inserter(keep));
                    tr.realizedCellIdx = keep;
                }
                tr.missedTargets = gather(plan.sensorTargets, sch.diagnostics.targetMissedIdx);

                if (params.verbose && sch.diagnostics.targetCoverage < 1.0) {
                    std::fprintf(stderr,
                                 "[mtl] agent %d: %lld of %lld valid-cell centres could not be "
                                 "observed.\n",
                                 plan.agentId,
                                 static_cast<long long>(sch.diagnostics.targetMissedIdx.size()),
                                 static_cast<long long>(sch.diagnostics.nTargets));
                }
            } else {
                // ---- MULTI-AXIS GIMBAL: there is nothing to schedule --------
                // With two or three gimbal axes the boresight points along-track
                // AND cross-track independently of the airframe, so every centre
                // on the micro-TSP sweep is reachable when the aircraft is in its
                // cluster's activation bubble.  No abeam instant to hit, no pitch
                // to spend, no trajectory to bend: the aircraft flies the nominal
                // Dubins path and the sensor follows the nominal sweep, and the
                // only attitude to work out is the airframe's.
                tr.drone           = plan.droneTraj;
                tr.sensor          = plan.sensorTraj;
                tr.rpy             = sensing::computeAirframeRPY(plan.droneTraj, params.gimbal);
                tr.realizedCellIdx = uniqueSorted(plan.sensorCellIdx);
            }
        }

        // --- pad every agent onto one team-wide timeline --------------------
        // The agent that took longest sets the length; the others hold their
        // final state, which is what a loiter-on-station looks like to a host
        // simulation stepping all agents together.
        Index maxSteps = 1;
        for (const AgentTrajectory& t : out.trajectories)
            maxSteps = std::max(maxSteps, t.drone.rows());

        out.timeVec.resize(maxSteps);
        for (Index i = 0; i < maxSteps; ++i) out.timeVec(i) = static_cast<double>(i) * params.dt;

        // Holding the final state means holding the final gimbal state too, so
        // the per-step diagnostics are padded with the trajectory.  They
        // describe the trajectory that was delivered, and the geometry audit
        // reads them alongside it - a short diagnostics vector would make the
        // audit compare the schedule against the wrong track.
        auto padVec = [](VecX& v, Index n) {
            if (v.size() == 0 || v.size() >= n) return;
            VecX held(n);
            held.head(v.size()) = v;
            held.tail(n - v.size()).setConstant(v(v.size() - 1));
            v = std::move(held);
        };

        for (AgentTrajectory& t : out.trajectories) {
            const Index n = t.drone.rows();
            if (n >= maxSteps || n == 0) continue;
            Path3 d(maxSteps, 3);
            Path2 s(maxSteps, 2);
            Path3 r(maxSteps, 3);
            d.topRows(n) = t.drone;
            s.topRows(n) = t.sensor;
            r.topRows(n) = t.rpy;
            d.bottomRows(maxSteps - n).rowwise() = t.drone.row(n - 1);
            s.bottomRows(maxSteps - n).rowwise() = t.sensor.row(n - 1);
            r.bottomRows(maxSteps - n).rowwise() = t.rpy.row(n - 1);
            t.drone  = std::move(d);
            t.sensor = std::move(s);
            t.rpy    = std::move(r);

            GimbalDiagnostics& g = t.diagnostics;
            padVec(g.gimbalAngle, maxSteps);
            padVec(g.crossAngle, maxSteps);
            padVec(g.roll, maxSteps);
            padVec(g.lookAngle, maxSteps);
            padVec(g.slantRange, maxSteps);
            padVec(g.lookForward, maxSteps);
            padVec(g.lookCross, maxSteps);
            padVec(g.altError, maxSteps);
            padVec(g.footprintRadius, maxSteps);
        }

        // --- what the gimbal actually realised, team-wide -------------------
        std::vector<Index> realized;
        for (const AgentTrajectory& t : out.trajectories)
            realized.insert(realized.end(), t.realizedCellIdx.begin(), t.realizedCellIdx.end());
        out.realizedCellIdx = uniqueSorted(realized);
        (void)cells;
    }
};

Planner::Planner(PlannerParams params) : impl_(std::make_unique<Impl>(std::move(params))) {}
Planner::~Planner()                         = default;
Planner::Planner(Planner&&) noexcept        = default;
Planner& Planner::operator=(Planner&&) noexcept = default;

const PlannerParams& Planner::params() const noexcept { return impl_->params; }

void Planner::setParams(PlannerParams params) { impl_ = std::make_unique<Impl>(std::move(params)); }

std::vector<double> Planner::agentBudgets() const { return impl_->budgets(); }

PlanningResult Planner::plan(const BeliefField& belief, const std::vector<Vec2>& agentStarts) {
    const PlannerParams& P = impl_->params;
    const CellSet cells = mapping::extractValidCells(belief, P.targetCellSize,
                                                     P.meanInformationThresh, P.verbose);
    return planFromCells(cells, agentStarts);
}

PlanningResult Planner::planFromCells(const CellSet& cells, const std::vector<Vec2>& agentStarts) {
    const PlannerParams& P = impl_->params;
    ClusterSet clusters =
        mapping::clusterCells(cells.centers, P.maxClusterRadius, P.cluster, P.rngSeed, P.verbose);
    return planFromClusters(cells, std::move(clusters), agentStarts);
}

PlanningResult Planner::planFromClusters(const CellSet& cells, ClusterSet clusters,
                                         const std::vector<Vec2>& agentStarts) {
    const PlannerParams& P = impl_->params;

    if (static_cast<int>(agentStarts.size()) < P.numAgents) {
        throw std::invalid_argument(
            "mtl::Planner::plan: fewer launch points than agents (need " +
            std::to_string(P.numAgents) + ")");
    }
    if (!clusters.empty() &&
        static_cast<Index>(clusters.cellCluster.size()) != cells.size()) {
        throw std::invalid_argument(
            "mtl::Planner::plan: clusters.cellCluster must have one entry per cell");
    }

    PlanningResult out;
    out.cells = cells;

    // Lift the per-cell mass onto the clusters if the caller has not.
    if (clusters.reward.size() != clusters.size() ||
        static_cast<Index>(clusters.cellIdx.size()) != clusters.size()) {
        mapping::computeClusterRewards(cells.mass, clusters);
    }
    out.clusters = std::move(clusters);

    // --- multi-agent task allocation ---------------------------------------
    out.agentOfCluster = impl_->partition(out.clusters);

    std::vector<Vec2> starts(agentStarts.begin(),
                             agentStarts.begin() + P.numAgents);

    planning::SortieContext ctx;
    ctx.clusters = &out.clusters;
    ctx.cells    = &out.cells;
    ctx.params   = &impl_->params;

    // --- route + trajectory planning (budget aware) ------------------------
    // One call replaces the per-agent TSP / hierarchical sensor path /
    // trajectory block.  With an infinite budget it performs exactly those three
    // steps; with a finite budget it solves the budgeted problem, prunes the
    // clusters and cells that cannot be reached, and verifies the FLOWN arc.
    planning::TeamResult team =
        planning::allocateBudgetedTeam(ctx, starts, out.agentOfCluster, impl_->budgets());
    out.plans = std::move(team.plans);
    out.team  = std::move(team.info);

    // Clusters and cells the budget put out of reach.  Everything downstream
    // sees only what is actually going to be flown and swept.
    out.reachedCentroids = gather(out.clusters.centroids, out.team.reachedClusters);
    out.droppedCentroids = gather(out.clusters.centroids, out.team.unreachedClusters);
    out.servicedCenters  = gather(out.cells.centers, out.team.servicedCellIdx);
    out.droppedCenters   = gather(out.cells.centers, out.team.unservicedCellIdx);

    impl_->realiseTrajectories(out.cells, out);
    return out;
}

}  // namespace mtl
