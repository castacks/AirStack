#include "mtl/planning/agent_sortie.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

#include "mtl/core/dubins.hpp"
#include "mtl/core/numeric.hpp"
#include "mtl/planning/cell_anchors.hpp"
#include "mtl/planning/info_score.hpp"
#include "mtl/planning/macro_route.hpp"
#include "mtl/routing/tsp.hpp"
#include "mtl/trajectory/trajectory_gen.hpp"

namespace mtl::planning {
namespace {

/// A grounded agent: the shape the rest of the pipeline already handles.
AgentPlan emptyPlan(const Vec2& startPos, const PlannerParams& P, int id, double budget) {
    AgentPlan plan;
    plan.agentId = id;
    plan.droneRoute.resize(1, 2);
    plan.droneRoute.row(0) = startPos.transpose();
    plan.droneTraj.resize(1, 3);
    plan.droneTraj.row(0) << startPos.x(), startPos.y(), P.droneAltitude;
    plan.sensorTraj.resize(1, 2);
    plan.sensorTraj.row(0) = startPos.transpose();
    plan.timeVec = VecX::Zero(1);
    plan.extInfo.enabled = P.extendForLateralCoverage();
    plan.extInfo.note    = "no route";
    plan.budget   = budget;
    plan.feasible = true;
    plan.routeInfo.note  = "no clusters assigned";
    plan.refineInfo.note = "not run";
    return plan;
}

/// Shortlist the cells worth offering to the refinement.
///
/// Ranked in the same currency the refinement itself spends, so the shortlist is
/// not merely "closest" but "best value", and the O(candidates) inner loops stay
/// small.  That currency is set by the gimbal, so the score is the mass of a
/// cell's whole REACH BALL over the detour needed to bring the ball into reach:
///
///     value = sum(mass within reach of the cell) / max(dist to route - reach, 1)
///
/// Ranking on the single cell's own mass splits neighbourhoods: a rich pocket of
/// small cells scores nothing individually and never reaches the solver, even
/// though one anchor would sweep all of it.  Subtracting the reach from the
/// distance matters just as much - a cell 520 m off the track with a 500 m reach
/// is 20 m of detour away, not 520 m, and would otherwise rank as unaffordable.
std::vector<Index> pickNearestCandidates(const std::vector<Index>& cellIdx, const CellSet& cells,
                                         const Path2& routeXY, int maxN, double reach) {
    if (static_cast<int>(cellIdx.size()) <= maxN) return cellIdx;

    const auto n = static_cast<Index>(cellIdx.size());
    Path2 pts(n, 2);
    VecX  m(n);
    for (Index i = 0; i < n; ++i) {
        pts.row(i) = cells.centers.row(cellIdx[static_cast<std::size_t>(i)]);
        m(i)       = cells.mass(cellIdx[static_cast<std::size_t>(i)]);
    }

    VecX d(n);
    for (Index i = 0; i < n; ++i) {
        double best = kInf;
        for (Index r = 0; r < routeXY.rows(); ++r)
            best = std::min(best, (routeXY.row(r) - pts.row(i)).norm());
        d(i) = best;
    }

    VecX ballMass = m;
    if (reach > 0.0) {
        for (Index i = 0; i < n; ++i) {
            double s = 0.0;
            for (Index q = 0; q < n; ++q)
                if ((pts.row(q) - pts.row(i)).norm() <= reach) s += m(q);
            ballMass(i) = s;
        }
    }

    std::vector<Index> order(static_cast<std::size_t>(n));
    for (Index i = 0; i < n; ++i) order[static_cast<std::size_t>(i)] = i;
    std::stable_sort(order.begin(), order.end(), [&](Index a, Index b) {
        return ballMass(a) / std::max(d(a) - reach, 1.0) >
               ballMass(b) / std::max(d(b) - reach, 1.0);
    });

    std::vector<Index> keep;
    keep.reserve(static_cast<std::size_t>(maxN));
    for (int i = 0; i < maxN; ++i) keep.push_back(cellIdx[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])]);
    return keep;
}

std::vector<Index> uniqueSorted(std::vector<Index> v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

std::vector<Index> setDifference(const std::vector<Index>& a, const std::vector<Index>& b) {
    const std::vector<Index> sa = uniqueSorted(a);
    const std::vector<Index> sb = uniqueSorted(b);
    std::vector<Index> out;
    std::set_difference(sa.begin(), sa.end(), sb.begin(), sb.end(), std::back_inserter(out));
    return out;
}

}  // namespace

AgentPlan planAgentSortie(const AgentSpec& agent, const SortieContext& ctx) {
    const PlannerParams& P   = *ctx.params;
    const ClusterSet&    cl  = *ctx.clusters;
    const CellSet&       cs  = *ctx.cells;
    const BudgetParams&  bo  = P.budget;

    const double B        = agent.budgetDist;
    const bool   budgeted = std::isfinite(B);

    AgentPlan plan = emptyPlan(agent.startPos, P, agent.id, B);
    if (agent.candClusters.empty()) return plan;

    const std::vector<Index> cand = uniqueSorted(agent.candClusters);
    const auto               nCand = static_cast<Index>(cand.size());

    Path2 candCentroids(nCand, 2);
    VecX  candRewards(nCand);
    for (Index i = 0; i < nCand; ++i) {
        candCentroids.row(i) = cl.centroids.row(cand[static_cast<std::size_t>(i)]);
        candRewards(i)       = cl.reward(cand[static_cast<std::size_t>(i)]);
    }

    // --- the reserve loop (outer, measured) --------------------------------
    double reserve = 0.0;
    if (budgeted && P.extendForLateralCoverage()) reserve = bo.reserveFrac0 * B;

    AgentPlan best;
    bool   haveBest  = false;
    double bestInfo  = -kInf;
    int    outerIters = 0;
    double resFeas   = kInf;  // smallest reserve measured FEASIBLE
    double resInfeas = -1.0;  // largest reserve measured INFEASIBLE (-1 = none yet)

    for (int outer = 1; outer <= std::max(1, bo.maxOuterIter); ++outer) {
        outerIters = outer;

        double eff = kInf;
        if (budgeted) {
            eff = B - reserve;
            if (eff <= 0.0) break;
        }

        // --- 1. macro route over this agent's candidate clusters -----------
        MacroRouteParams mr = bo.macroRoute;
        mr.verbose          = bo.verbose;
        OrienteeringParams oo = bo.orienteering;
        oo.verbose            = bo.verbose && oo.verbose;

        MacroRouteSolution route = planBudgetedMacroRoute(
            candCentroids, candRewards, agent.startPos, eff, P.minTurnRadius, mr, oo, P.dubins);

        std::vector<Index> selGlobal;
        selGlobal.reserve(route.visitOrder.size());
        for (const Index o : route.visitOrder) selGlobal.push_back(cand[static_cast<std::size_t>(o)]);

        // Local entity list: one entry per route row 1..n, holding the global
        // cell indices that row is responsible for.
        std::vector<std::vector<Index>> entityCells;
        entityCells.reserve(selGlobal.size());
        for (const Index g : selGlobal) entityCells.push_back(cl.cellIdx[static_cast<std::size_t>(g)]);

        // --- 2. cell-level refinement on whatever budget is left -----------
        CellRefineInfo     refineInfo;
        std::vector<Index> anchorsGlobal;

        if (budgeted && bo.cellRefine && route.routeXY.rows() > 0) {
            const std::vector<Index> droppedGlobal = setDifference(cand, selGlobal);
            std::vector<Index>       leftoverCells;
            for (const Index q : droppedGlobal) {
                const auto& lst = cl.cellIdx[static_cast<std::size_t>(q)];
                leftoverCells.insert(leftoverCells.end(), lst.begin(), lst.end());
            }

            if (!leftoverCells.empty()) {
                CellAnchorParams rf = bo.cellAnchor;
                rf.verbose = bo.verbose;
                // The reach is the SAME radius clusterCells used to guarantee a
                // cluster is sweepable from its centroid, so an anchor row and a
                // cluster row are one object to the sweep window downstream.
                if (!rf.gimbalReach.has_value()) rf.gimbalReach = P.maxClusterRadius;

                leftoverCells = pickNearestCandidates(leftoverCells, cs, route.routeXY,
                                                      bo.maxCellCandidates, *rf.gimbalReach);

                Path2 candXY(static_cast<Index>(leftoverCells.size()), 2);
                VecX  candMass(static_cast<Index>(leftoverCells.size()));
                for (std::size_t i = 0; i < leftoverCells.size(); ++i) {
                    candXY.row(static_cast<Index>(i)) = cs.centers.row(leftoverCells[i]);
                    candMass(static_cast<Index>(i))   = cs.mass(leftoverCells[i]);
                }

                const CellAnchorResult rr = refineWithCellAnchors(
                    route.routeXY, candXY, candMass, eff, P.minTurnRadius, rf, P.dubins);

                if (rr.info.nAdded > 0 || rr.info.nHarvested > 0) {
                    // Re-derive, from rowTag/rowCells, which cells each route row
                    // now owns.  rowCells holds only what THIS PASS added, so an
                    // incoming cluster row keeps its own cells and gains the free
                    // harvest on top; an anchor row owns its whole reach ball.
                    std::vector<std::vector<Index>> newEntity;
                    newEntity.reserve(rr.rowTag.size() - 1);
                    for (std::size_t i = 1; i < rr.rowTag.size(); ++i) {
                        std::vector<Index> extra;
                        extra.reserve(rr.rowCells[i].size());
                        for (const Index c : rr.rowCells[i]) extra.push_back(leftoverCells[static_cast<std::size_t>(c)]);

                        std::vector<Index> row;
                        if (rr.rowTag[i] < 0) {
                            // row (-tag) of the incoming route -> its cluster's
                            // cells, plus anything harvested onto it for free
                            const auto src = static_cast<std::size_t>(-rr.rowTag[i] - 1);
                            if (src < entityCells.size()) row = entityCells[src];
                        }
                        row.insert(row.end(), extra.begin(), extra.end());
                        newEntity.push_back(std::move(row));
                        anchorsGlobal.insert(anchorsGlobal.end(), extra.begin(), extra.end());
                    }
                    entityCells  = std::move(newEntity);
                    route.routeXY = rr.routeXY;
                }
                refineInfo            = rr.info;
                refineInfo.candidates = leftoverCells;
            }
        }

        // --- 3. sensor path over the route, in route order -----------------
        Path2              sensorTargets;
        std::vector<Index> sensorMacroRow, sensorCellIdx;
        routing::buildSensorPathOrdered(cs.centers, entityCells, route.routeXY, sensorTargets,
                                        sensorMacroRow, sensorCellIdx);

        // --- 4. trajectories ------------------------------------------------
        if (route.routeXY.rows() < 2 || sensorTargets.rows() == 0) {
            AgentPlan cur = emptyPlan(agent.startPos, P, agent.id, B);
            cur.routeInfo = route.info;
            best     = cur;
            haveBest = true;
            break;
        }

        trajectory::TrajectoryInputs ti;
        ti.macroRoute       = route.routeXY;
        ti.sensorTargets    = sensorTargets;
        ti.sensorMacroRow   = sensorMacroRow;
        ti.droneAltitude    = P.droneAltitude;
        ti.avgDroneSpeed    = P.avgDroneSpeed;
        ti.dt               = P.dt;
        ti.minTurnRadius    = P.minTurnRadius;
        ti.maxClusterRadius = P.maxClusterRadius;
        ti.extendForLateralCoverage = P.extendForLateralCoverage();

        const trajectory::TrajectoryResult tr =
            trajectory::generateTrajectories(ti, P.traj, P.extension, P.dubins);

        const double flown = core::polylineLength(tr.drone);

        AgentPlan cur;
        cur.agentId        = agent.id;
        cur.droneRoute     = route.routeXY;
        cur.sensorTargets  = sensorTargets;
        cur.sensorMacroRow = sensorMacroRow;
        cur.sensorCellIdx  = sensorCellIdx;
        cur.droneTraj      = tr.drone;
        cur.sensorTraj     = tr.sensor;
        cur.timeVec        = tr.time;
        cur.extInfo        = tr.extInfo;
        cur.selClusters    = selGlobal;
        cur.cellAnchors    = uniqueSorted(anchorsGlobal);
        cur.servicedCellIdx = uniqueSorted(sensorCellIdx);
        cur.droppedClusters = setDifference(cand, selGlobal);
        cur.flownLength    = flown;
        cur.flightTime     = flown / P.avgDroneSpeed;
        cur.budget         = B;
        cur.budgetUsed     = budgeted ? flown / B : 0.0;
        cur.feasible       = !budgeted || flown <= B * (1.0 + bo.tol);
        cur.routeInfo      = route.info;
        cur.refineInfo     = refineInfo;
        cur.reserve        = reserve;
        cur.outerIters     = outer;

        double curInfo = 0.0;
        for (const Index c : cur.servicedCellIdx) curInfo += cs.mass(c);

        // --- 5. keep the best plan so far -----------------------------------
        // Feasible ALWAYS beats infeasible, whatever the information: a plan the
        // aircraft cannot fly is not a better plan.  Among plans of equal
        // feasibility, more information wins.
        if (!haveBest || (cur.feasible && !best.feasible) ||
            (cur.feasible == best.feasible && curInfo > bestInfo + 1e-9)) {
            best     = cur;
            bestInfo = curInfo;
            haveBest = true;
        }

        if (!budgeted) break;

        // --- 6. bisect the reserve -------------------------------------------
        double nextRes;
        if (cur.feasible) {
            resFeas = std::min(resFeas, reserve);
            if (cur.budgetUsed >= 1.0 - bo.tol - 0.02) break;  // budget is being spent; done
            nextRes = (resInfeas < 0.0) ? std::max(0.0, 0.5 * reserve)
                                        : 0.5 * (resInfeas + resFeas);
        } else {
            resInfeas = std::max(resInfeas, reserve);
            if (std::isfinite(resFeas)) {
                nextRes = 0.5 * (resInfeas + resFeas);
            } else {
                nextRes = reserve + std::max((flown - B) * 1.05, 0.02 * B);
            }
            if (nextRes >= B) break;
        }

        if (std::isfinite(resFeas) && resInfeas >= 0.0 && (resFeas - resInfeas) < 0.01 * B)
            break;  // bracket tight enough
        if (std::abs(nextRes - reserve) < 0.005 * B) break;  // nothing left to move

        if (bo.verbose) {
            std::printf(
                "  agent %d: flown %.0f m of %.0f m (%.0f%%, feasible %d); reserve %.0f -> %.0f m, "
                "re-planning.\n",
                agent.id, flown, B, 100.0 * cur.budgetUsed, cur.feasible ? 1 : 0, reserve, nextRes);
        }
        reserve = nextRes;
    }

    if (!haveBest) best = emptyPlan(agent.startPos, P, agent.id, B);
    plan = best;
    plan.outerIters = outerIters;

    InfoScoreOptions so;
    so.budget       = B;
    so.totalMapMass = cs.totalMapMass;
    plan.score = pathInformationScore(cs.mass, plan.servicedCellIdx, plan.flownLength, so);

    if (bo.verbose) {
        std::printf(
            "  agent %d: %lld clusters + %lld cell anchors, %lld cells, info %.4g (%.1f%% of map "
            "cells), flown %.0f m",
            agent.id, static_cast<long long>(plan.selClusters.size()),
            static_cast<long long>(plan.cellAnchors.size()),
            static_cast<long long>(plan.servicedCellIdx.size()), plan.score.info,
            100.0 * plan.score.infoFraction, plan.flownLength);
        if (budgeted) {
            std::printf(" / %.0f m (%.1f%%), %.1f min\n", B, 100.0 * plan.budgetUsed,
                        plan.flightTime / 60.0);
        } else {
            std::printf(" (unbudgeted), %.1f min\n", plan.flightTime / 60.0);
        }
    }

    if (budgeted && !plan.feasible) {
        std::fprintf(stderr,
                     "[mtl] warning: agent %d could not be planned inside the %.0f m budget (best "
                     "%.0f m after %d passes). Raise budget.maxOuterIter or budget.reserveFrac0.\n",
                     agent.id, B, plan.flownLength, outerIters);
    }
    return plan;
}

}  // namespace mtl::planning
