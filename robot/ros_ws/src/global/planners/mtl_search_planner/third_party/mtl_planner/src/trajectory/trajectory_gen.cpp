#include "mtl/trajectory/trajectory_gen.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mtl/core/dubins.hpp"
#include "mtl/core/numeric.hpp"
#include "mtl/sensing/abeam.hpp"
#include "mtl/trajectory/lateral_coverage.hpp"

namespace mtl::trajectory {

TrajectoryResult generateTrajectories(const TrajectoryInputs& in, const TrajectoryParams& to,
                                      const ExtensionParams& extendOpts,
                                      const DubinsParams& dubinsOpts) {
    TrajectoryResult out;

    // --- 1. drone Dubins path ---------------------------------------------
    Path2 droneTraj2D;
    VecX  droneArc;
    core::computeDubinsWaypoints(in.macroRoute, in.minTurnRadius, dubinsOpts, droneTraj2D,
                                 droneArc);

    out.extInfo.enabled       = in.extendForLateralCoverage;
    out.extInfo.arcNominalEnd = droneArc.size() > 0 ? droneArc(droneArc.size() - 1) : 0.0;

    if (droneTraj2D.rows() < 2) {
        out.drone.resize(1, 3);
        out.drone.row(0) << in.macroRoute(0, 0), in.macroRoute(0, 1), in.droneAltitude;
        out.sensor = out.drone.leftCols(2);
        out.time   = VecX::Zero(1);
        return out;
    }

    // --- 1b. optional: extend the ground track so the tail cells come abeam --
    const Path2 nominalXY = droneTraj2D;
    if (in.extendForLateralCoverage && in.sensorTargets.rows() > 0) {
        const Index MAll = in.sensorTargets.rows();

        // Which centres are allowed to drive the extension.  Default 'tail':
        // only the final macro-cluster(s), the ones the route abandons.  Cells
        // missed mid-route are a different, much more expensive problem -
        // covering them means re-flying the middle of the sortie - so they are
        // opt-in via Scope::All.
        std::vector<Index> candIdx;
        if (extendOpts.scope == ExtensionParams::Scope::All || in.sensorMacroRow.empty()) {
            for (Index i = 0; i < MAll; ++i) candIdx.push_back(i);
        } else {
            const Index mLast = *std::max_element(in.sensorMacroRow.begin(), in.sensorMacroRow.end());
            const Index floorRow = mLast - (std::max(1, extendOpts.tailClusters) - 1);
            for (Index i = 0; i < MAll; ++i)
                if (in.sensorMacroRow[static_cast<std::size_t>(i)] >= floorRow) candIdx.push_back(i);
        }

        Path2 candPts(static_cast<Index>(candIdx.size()), 2);
        for (std::size_t i = 0; i < candIdx.size(); ++i)
            candPts.row(static_cast<Index>(i)) = in.sensorTargets.row(candIdx[i]);

        const ExtensionResult ext = extendTrajForLateralCoverage(
            nominalXY, candPts, in.droneAltitude, in.minTurnRadius, extendOpts);

        out.extInfo.scope = (extendOpts.scope == ExtensionParams::Scope::All) ? "all" : "tail";
        out.extInfo.candidateIdx = candIdx;
        for (const Index r : ext.info.residualBefore)
            out.extInfo.residualBefore.push_back(candIdx[static_cast<std::size_t>(r)]);
        for (const Index r : ext.info.residualAfter)
            out.extInfo.residualAfter.push_back(candIdx[static_cast<std::size_t>(r)]);
        out.extInfo.extendDist = ext.info.extendDist;
        out.extInfo.extraDist = ext.info.extraDist;
        out.extInfo.reachEff  = ext.info.reachEff;
        out.extInfo.alongTol  = ext.info.alongTol;
        out.extInfo.passes    = ext.info.passes;
        out.extInfo.note      = ext.info.note;

        if (ext.extXY.rows() > 0) {
            Path2 merged(nominalXY.rows() + ext.extXY.rows(), 2);
            merged.topRows(nominalXY.rows())    = nominalXY;
            merged.bottomRows(ext.extXY.rows()) = ext.extXY;

            // Same hygiene as computeDubinsWaypoints: a repeated sample makes
            // the arc length non-monotonic and the interpolation cannot use it.
            std::vector<Vec2> kept;
            kept.reserve(static_cast<std::size_t>(merged.rows()));
            kept.emplace_back(merged(0, 0), merged(0, 1));
            for (Index i = 1; i < merged.rows(); ++i) {
                const Vec2 p(merged(i, 0), merged(i, 1));
                if ((p - kept.back()).norm() > 1e-9) kept.push_back(p);
            }
            droneTraj2D.resize(static_cast<Index>(kept.size()), 2);
            for (std::size_t i = 0; i < kept.size(); ++i)
                droneTraj2D.row(static_cast<Index>(i)) = kept[i].transpose();
            VecX ds;
            core::arcOf(droneTraj2D, ds, droneArc);
            out.extInfo.applied = true;
        }

        // Book-keeping over ALL this agent's centres, not just the ones the
        // scope let us plan for, so the cost/benefit of Scope::All is visible.
        if (ext.info.reachEff > 0.0) {
            const sensing::AbeamResult before = sensing::abeamObservable(
                nominalXY, in.sensorTargets, ext.info.reachEff, ext.info.alongTol,
                extendOpts.alongOffset);
            const sensing::AbeamResult after = sensing::abeamObservable(
                droneTraj2D, in.sensorTargets, ext.info.reachEff, ext.info.alongTol,
                extendOpts.alongOffset);
            out.extInfo.nNeverAbeamAllBefore = MAll - before.nObservable;
            out.extInfo.nNeverAbeamAllAfter  = MAll - after.nObservable;
            if (extendOpts.verbose) {
                std::printf("  extendTraj: over all %lld centres, never abeam %lld -> %lld (scope '%s').\n",
                            static_cast<long long>(MAll),
                            static_cast<long long>(out.extInfo.nNeverAbeamAllBefore),
                            static_cast<long long>(out.extInfo.nNeverAbeamAllAfter),
                            out.extInfo.scope.c_str());
            }
        }
    }

    // --- time-parameterise at constant ground speed ------------------------
    const double arcEnd    = droneArc(droneArc.size() - 1);
    const double totalTime = std::max(arcEnd / in.avgDroneSpeed, to.minSimTime);
    const auto   movingSteps =
        static_cast<Index>(std::floor(totalTime / in.dt)) + 1;

    VecX targetArc(movingSteps);
    if (movingSteps == 1) {
        targetArc(0) = 0.0;
    } else {
        for (Index i = 0; i < movingSteps; ++i)
            targetArc(i) = arcEnd * static_cast<double>(i) / static_cast<double>(movingSteps - 1);
    }

    Path3 movingTraj(movingSteps, 3);
    movingTraj.col(0) = core::interp1(droneArc, droneTraj2D.col(0), targetArc);
    movingTraj.col(1) = core::interp1(droneArc, droneTraj2D.col(1), targetArc);
    movingTraj.col(2).setConstant(in.droneAltitude);

    // A fixed wing cannot hover, and with the single-axis gimbal every centre is
    // observed in passing, so hoverTime is 0 by default.
    const auto hoverSteps = static_cast<Index>(std::lround(to.hoverTime / in.dt));
    if (hoverSteps > 0) {
        out.drone.resize(movingSteps + hoverSteps, 3);
        out.drone.topRows(movingSteps) = movingTraj;
        out.drone.bottomRows(hoverSteps).rowwise() = movingTraj.row(movingSteps - 1);
    } else {
        out.drone = movingTraj;
    }

    const Index nSteps = out.drone.rows();
    out.time.resize(nSteps);
    for (Index i = 0; i < nSteps; ++i) out.time(i) = static_cast<double>(i) * in.dt;

    // --- 2. default state: nadir transit -----------------------------------
    out.sensor = out.drone.leftCols(2);

    // Where the appended coverage extension begins, in trajectory steps.  The
    // macro-cluster sweep windows below must stay inside the NOMINAL part of the
    // route: the extension flies back past the last cluster, and without this
    // the closest-approach search would drag the last cluster's window into it.
    Index extStartIdx = -1;
    if (out.extInfo.applied) {
        extStartIdx = nSteps - 1;
        for (Index i = 0; i < movingSteps; ++i) {
            if (targetArc(i) >= out.extInfo.arcNominalEnd - 1e-6) {
                extStartIdx = i;
                break;
            }
        }
        extStartIdx = std::min(std::max<Index>(extStartIdx, 2), nSteps - 1);
    }
    const Index nSearch = (extStartIdx < 0) ? nSteps : std::max<Index>(3, extStartIdx);

    // --- 3. active state: micro-TSP sweeps over contiguous time windows -----
    const double activationRadius = in.maxClusterRadius * to.activationRadiusFactor;
    const Index  numMacro         = in.macroRoute.rows();

    // Pre-calculate all peak approach indices first, so the windows below are
    // strictly ordered in time and cannot overlap.
    std::vector<Index> peak(static_cast<std::size_t>(numMacro), 0);
    Index searchIdx = 0;
    for (Index m = 0; m < numMacro; ++m) {
        double best = kInf;
        Index  bestI = searchIdx;
        for (Index i = searchIdx; i < nSearch; ++i) {
            const double d =
                (out.drone.block(i, 0, 1, 2).transpose() - in.macroRoute.row(m).transpose()).norm();
            if (d < best) {
                best  = d;
                bestI = i;
            }
        }
        peak[static_cast<std::size_t>(m)] = bestI;
        searchIdx = bestI;  // the next peak must chronologically follow this one
    }

    Index prevTEnd = 0;
    for (Index m = 1; m < numMacro; ++m) {
        std::vector<Index> idxM;
        for (std::size_t s = 0; s < in.sensorMacroRow.size(); ++s)
            if (in.sensorMacroRow[s] == m) idxM.push_back(static_cast<Index>(s));
        if (idxM.empty()) continue;

        // A. expand the window backwards, without overwriting the previous cluster
        Index tStart = peak[static_cast<std::size_t>(m)];
        while (tStart > prevTEnd &&
               (out.drone.block(tStart, 0, 1, 2).transpose() - in.macroRoute.row(m).transpose())
                       .norm() < activationRadius) {
            --tStart;
        }

        // B. expand forwards.  If clusters are close, split the difference so
        //    they do not fight for time.
        Index tEnd = peak[static_cast<std::size_t>(m)];
        Index nextLimit = nSearch - 1;
        if (m + 1 < numMacro)
            nextLimit = (peak[static_cast<std::size_t>(m)] + peak[static_cast<std::size_t>(m + 1)]) / 2;
        while (tEnd < nextLimit &&
               (out.drone.block(tEnd, 0, 1, 2).transpose() - in.macroRoute.row(m).transpose())
                       .norm() < activationRadius) {
            ++tEnd;
        }

        const Index segmentSteps = tEnd - tStart + 1;
        if (segmentSteps <= 5) continue;

        // C. anchor the start and end of the sweep to the drone's own position,
        //    so the gimbal smoothly deploys and retracts.
        Path2 microPath(static_cast<Index>(idxM.size()) + 2, 2);
        microPath.row(0) = out.drone.block(tStart, 0, 1, 2);
        for (std::size_t i = 0; i < idxM.size(); ++i)
            microPath.row(static_cast<Index>(i) + 1) = in.sensorTargets.row(idxM[i]);
        microPath.row(microPath.rows() - 1) = out.drone.block(tEnd, 0, 1, 2);

        out.sensor.block(tStart, 0, segmentSteps, 2) =
            core::resampleAlongPath(microPath, segmentSteps);
        prevTEnd = tEnd;
    }

    // --- 4. advisory sweep over the coverage extension ----------------------
    // The scheduler treats the sensor path as advisory (the cell centres are the
    // hard constraint), but leaving the extension at nadir would make the plot
    // look as if nothing were being looked at out there.  Order the previously
    // unobservable centres by WHEN the extension draws level with them, and run
    // the advisory boresight through them in that order.
    if (out.extInfo.applied && !out.extInfo.residualBefore.empty() &&
        extStartIdx >= 0 && extStartIdx < nSteps - 5) {
        const Index nWin = nSteps - extStartIdx;
        const auto& resid = out.extInfo.residualBefore;

        std::vector<std::pair<Index, Index>> abeam;  // (step, cell)
        abeam.reserve(resid.size());
        for (const Index j : resid) {
            double best = kInf;
            Index  bestK = 0;
            for (Index k = 0; k < nWin; ++k) {
                const double d = (out.drone.block(extStartIdx + k, 0, 1, 2).transpose() -
                                  in.sensorTargets.row(j).transpose())
                                     .norm();
                if (d < best) {
                    best  = d;
                    bestK = k;
                }
            }
            abeam.emplace_back(bestK, j);
        }
        std::stable_sort(abeam.begin(), abeam.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });

        Path2 microPath(static_cast<Index>(abeam.size()) + 2, 2);
        microPath.row(0) = out.drone.block(extStartIdx, 0, 1, 2);
        for (std::size_t i = 0; i < abeam.size(); ++i)
            microPath.row(static_cast<Index>(i) + 1) = in.sensorTargets.row(abeam[i].second);
        microPath.row(microPath.rows() - 1) = out.drone.block(nSteps - 1, 0, 1, 2);

        out.sensor.block(extStartIdx, 0, nWin, 2) = core::resampleAlongPath(microPath, nWin);
    }

    return out;
}

}  // namespace mtl::trajectory
