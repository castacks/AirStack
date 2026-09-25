#include "mtl/planning/cell_anchors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

#include "mtl/core/dubins.hpp"
#include "mtl/core/numeric.hpp"

namespace mtl::planning {
namespace {

/// Cheapest waypoint bringing the route within `standoff` of `target`.
///
/// For every insertion slot we take the point of that leg closest to the target
/// and then walk BACK OUT along the line toward the target only as far as the
/// stand-off allows.  Three regimes fall out of the same two lines of algebra:
///   standoff = 0        -> the waypoint is the target itself (overfly)
///   0 < standoff < gap  -> the track is shifted toward the target until it is
///                          in gimbal reach; the detour is ~2*(gap - standoff)
///   standoff >= gap     -> the leg already passes inside reach; detour 0, and
///                          all the waypoint does is declare a sweep window
struct AnchorInsertion {
    double      delta = kInf;  ///< Euclidean detour
    std::size_t pos   = 0;     ///< insert AFTER this row
    Vec2        pt    = Vec2::Zero();
};

AnchorInsertion bestAnchorInsertion(const Path2& routeXY, const Vec2& target, double standoff) {
    const auto      n = static_cast<std::size_t>(routeXY.rows());
    AnchorInsertion best;
    best.pos = n > 0 ? n - 1 : 0;
    best.pt  = target;

    for (std::size_t p = 0; p < n; ++p) {
        const Vec2 a = routeXY.row(static_cast<Index>(p)).transpose();
        Vec2       q = a;
        double     base = 0.0;
        bool       hasNext = (p + 1 < n);
        Vec2       b = Vec2::Zero();
        if (hasNext) {
            b    = routeXY.row(static_cast<Index>(p + 1)).transpose();
            q    = core::projectPointOnSegment(target, a, b);
            base = (b - a).norm();
        }

        const double gap = (target - q).norm();
        Vec2 pt;
        if (gap <= standoff + 1e-9) {
            pt = q;  // already in reach
        } else {
            pt = target + (standoff / gap) * (q - target);
        }

        double d = (pt - a).norm() - base;
        if (hasNext) d += (b - pt).norm();
        d = std::max(d, 0.0);

        if (d < best.delta - 1e-12) {
            best.delta = d;
            best.pos   = p;
            best.pt    = pt;
        }
    }
    return best;
}

}  // namespace

CellAnchorResult refineWithCellAnchors(const Path2& routeXY_in, const Path2& cellsXY,
                                       const VecX& cellRewards, double budgetDist,
                                       double minTurnRadius, const CellAnchorParams& opts,
                                       const DubinsParams& dubinsOpts,
                                       const LengthFcn& lengthFcn) {
    CellAnchorResult out;
    out.routeXY = routeXY_in;

    LengthFcn measure = lengthFcn;
    if (!measure) {
        measure = [minTurnRadius, dubinsOpts](const Path2& p) {
            return core::dubinsLength(p, minTurnRadius, dubinsOpts);
        };
    }

    const Index C = cellsXY.rows();

    // --- the gimbal reach and the stand-off ladder -------------------------
    const double R = std::max(0.0, opts.gimbalReach.value_or(0.0));
    std::vector<double> standoffs;
    if (R > 0.0) {
        std::set<double, std::greater<>> tiers;  // cheapest detour first
        for (const double f : opts.standoffTiers)
            if (f >= 0.0 && f <= 1.0) tiers.insert(R * f);
        if (tiers.empty()) tiers.insert(0.0);
        standoffs.assign(tiers.begin(), tiers.end());
    } else {
        standoffs.push_back(0.0);  // overfly the cell
    }

    const double L0     = measure(out.routeXY);
    const auto   nRows0 = static_cast<std::size_t>(out.routeXY.rows());

    out.rowTag.assign(nRows0, 0);
    for (std::size_t i = 1; i < nRows0; ++i) out.rowTag[i] = -static_cast<int>(i);
    out.rowCells.assign(nRows0, {});

    out.info.lengthBefore = L0;
    out.info.flownLength  = L0;
    out.info.gimbalReach  = R;
    out.info.note         = "no candidates";

    if (C == 0 || !std::isfinite(budgetDist) || nRows0 < 1) return out;
    if (L0 > budgetDist + 1e-6) {
        out.info.feasible = false;
        out.info.note     = "route already over budget";
        return out;
    }

    // --- Dubins inflation ratio of the route we already have ---------------
    double ratio = 1.3;  // nothing to measure yet; a turn-heavy guess is the safe one
    {
        const double eL = core::polylineLength(out.routeXY);
        if (eL > 1e-6) ratio = std::max(1.0, L0 / eL);
    }

    // --- who is eligible ----------------------------------------------------
    // Every cell with mass is eligible to be SERVICED (it may ride along in
    // someone else's reach ball for nothing).  minGainFrac is applied to the
    // reach BALL an anchor would buy, not to the individual cell: a modest cell
    // surrounded by others is worth the detour even when the cell alone is not.
    std::vector<char> alive(static_cast<std::size_t>(C), 0);
    double meanPositive = 0.0;
    Index  nPositive    = 0;
    for (Index j = 0; j < C; ++j) {
        if (cellRewards(j) > 0.0) {
            alive[static_cast<std::size_t>(j)] = 1;
            meanPositive += cellRewards(j);
            ++nPositive;
        }
    }
    const double gainFloor =
        (opts.minGainFrac > 0.0 && nPositive > 0)
            ? opts.minGainFrac * (meanPositive / static_cast<double>(nPositive))
            : 0.0;

    std::vector<Index> served;
    int                nHarvest = 0;

    // =====================================================================
    // 1. FREE HARVEST - cells already inside gimbal reach of a flown waypoint
    // =====================================================================
    if (R > 0.0 && opts.harvestFree && nRows0 >= 2) {
        for (Index j = 0; j < C; ++j) {
            if (!alive[static_cast<std::size_t>(j)]) continue;
            double      dm    = kInf;
            std::size_t bestR = 1;
            for (std::size_t m = 1; m < nRows0; ++m) {
                const double d =
                    (out.routeXY.row(static_cast<Index>(m)).transpose() - cellsXY.row(j).transpose())
                        .norm();
                if (d < dm) {
                    dm    = d;
                    bestR = m;
                }
            }
            if (dm <= R + 1e-9) {
                out.rowCells[bestR].push_back(j);
                alive[static_cast<std::size_t>(j)] = 0;
                served.push_back(j);
                ++nHarvest;
            }
        }
    }

    // =====================================================================
    // 2. ANCHOR WAYPOINTS - greedy reach-ball reward per metre of detour
    // =====================================================================
    int                nAnchors = 0;
    int                nTrials  = 0;
    double             Lcur     = L0;
    std::vector<Index> anchorSeeds;
    std::vector<Index> rejected;
    // Detour at which each candidate was refused by the real measurement.
    // Anything at least that dear is hopeless; strictly cheaper rungs are not.
    VecX rejDetour = VecX::Constant(C, kInf);

    auto anyAlive = [&]() {
        return std::any_of(alive.begin(), alive.end(), [](char c) { return c != 0; });
    };

    while (anyAlive() && static_cast<double>(nAnchors) < opts.maxAdd && nTrials < opts.maxTrials) {
        std::vector<Index> idxAlive;
        for (Index j = 0; j < C; ++j)
            if (alive[static_cast<std::size_t>(j)]) idxAlive.push_back(j);

        double bestSc = -kInf;
        bool   haveBest = false;
        struct Best {
            Index              j = 0;
            double             delta = 0.0;
            std::size_t        pos = 0;
            Vec2               pt = Vec2::Zero();
            std::vector<Index> grp;
            double             reward = 0.0;
        } bestOpt;

        for (const Index j : idxAlive) {
            for (const double so : standoffs) {
                const AnchorInsertion ins =
                    bestAnchorInsertion(out.routeXY, cellsXY.row(j).transpose(), so);

                if (ins.delta >= rejDetour(j) - 1e-9) continue;  // already refused this dear
                // optimistic feasibility screen on the scaled Euclidean detour
                if (Lcur + ratio * ins.delta > budgetDist + 1e-6) continue;

                std::vector<Index> grp;
                if (R > 0.0) {
                    for (const Index q : idxAlive)
                        if ((cellsXY.row(q).transpose() - ins.pt).norm() <= R + 1e-9)
                            grp.push_back(q);
                    if (std::find(grp.begin(), grp.end(), j) == grp.end()) grp.push_back(j);
                } else {
                    grp.push_back(j);
                }

                double rew = 0.0;
                for (const Index q : grp) rew += cellRewards(q);
                if (rew < gainFloor - 1e-12) continue;

                const double sc = rew / std::max(ins.delta, 1e-6);
                if (sc > bestSc + 1e-12) {
                    bestSc        = sc;
                    haveBest      = true;
                    bestOpt.j     = j;
                    bestOpt.delta = ins.delta;
                    bestOpt.pos   = ins.pos;
                    bestOpt.pt    = ins.pt;
                    bestOpt.grp   = grp;
                    bestOpt.reward = rew;
                }
            }
        }

        if (!haveBest) break;

        const std::size_t p  = bestOpt.pos;
        const Vec2        pt = bestOpt.pt;

        // --- would this anchor duplicate a row the route already flies? ----
        // It can, when the track already passes inside gimbal reach at a corner.
        // Merging into that row keeps the macro list clean and the sweep windows
        // from fighting each other over the same stretch of track.
        std::size_t attachRow = 0;
        if (bestOpt.delta <= 1e-9) {
            const auto n = static_cast<std::size_t>(out.routeXY.rows());
            if (p >= 1 &&
                (pt - out.routeXY.row(static_cast<Index>(p)).transpose()).norm() <= opts.snapTol) {
                attachRow = p;
            } else if (p + 1 < n && (pt - out.routeXY.row(static_cast<Index>(p + 1)).transpose())
                                            .norm() <= opts.snapTol) {
                attachRow = p + 1;
            }
        }
        if (attachRow > 0) {
            for (const Index q : bestOpt.grp) {
                out.rowCells[attachRow].push_back(q);
                alive[static_cast<std::size_t>(q)] = 0;
                served.push_back(q);
                ++nHarvest;
            }
            continue;
        }

        // --- tentative splice, then MEASURE -------------------------------
        Path2 trial(out.routeXY.rows() + 1, 2);
        trial.topRows(static_cast<Index>(p) + 1) = out.routeXY.topRows(static_cast<Index>(p) + 1);
        trial.row(static_cast<Index>(p) + 1)     = pt.transpose();
        if (static_cast<Index>(p) + 1 < out.routeXY.rows()) {
            trial.bottomRows(out.routeXY.rows() - static_cast<Index>(p) - 1) =
                out.routeXY.bottomRows(out.routeXY.rows() - static_cast<Index>(p) - 1);
        }

        const double Ltrial = measure(trial);
        ++nTrials;

        if (Ltrial <= budgetDist + 1e-6) {
            out.routeXY = trial;
            out.rowTag.insert(out.rowTag.begin() + static_cast<long>(p) + 1,
                              static_cast<int>(bestOpt.j) + 1);
            out.rowCells.insert(out.rowCells.begin() + static_cast<long>(p) + 1, bestOpt.grp);
            Lcur = Ltrial;
            ++nAnchors;
            anchorSeeds.push_back(bestOpt.j);
            for (const Index q : bestOpt.grp) {
                alive[static_cast<std::size_t>(q)] = 0;
                served.push_back(q);
            }
            const double eL = core::polylineLength(out.routeXY);
            if (eL > 1e-6) ratio = std::max(1.0, Lcur / eL);
        } else {
            // Refused at this cost.  Cheaper rungs of the same ladder - a bigger
            // stand-off, i.e. shifting the track toward the cell instead of over
            // it - are still allowed, which is the whole point of the ladder.
            rejDetour(bestOpt.j) = std::min(rejDetour(bestOpt.j), bestOpt.delta);
            rejected.push_back(bestOpt.j);
            if (bestOpt.delta > 1e-9)
                ratio = std::max(ratio, (Ltrial - Lcur) / bestOpt.delta);
        }
    }

    // --- pack, de-duplicating `served` but keeping acceptance order --------
    std::vector<char>  seen(static_cast<std::size_t>(C), 0);
    std::vector<Index> uniqueServed;
    for (const Index j : served) {
        if (seen[static_cast<std::size_t>(j)]) continue;
        seen[static_cast<std::size_t>(j)] = 1;
        uniqueServed.push_back(j);
    }

    out.addedCells        = uniqueServed;
    out.info.reward       = 0.0;
    for (const Index j : uniqueServed) out.info.reward += cellRewards(j);
    out.info.flownLength  = Lcur;
    out.info.nAdded       = nAnchors;
    out.info.nHarvested   = nHarvest;
    out.info.nCellsServed = static_cast<int>(uniqueServed.size());
    out.info.anchorSeeds  = anchorSeeds;
    out.info.nTrials      = nTrials;
    out.info.ratio        = ratio;
    out.info.rejected     = rejected;
    out.info.feasible     = Lcur <= budgetDist + 1e-6;
    out.info.note         = "refined";

    if (opts.verbose && (nAnchors > 0 || nHarvest > 0 || !rejected.empty())) {
        std::printf(
            "  cell refinement (gimbal reach %.0f m): %d anchor waypoint(s) + %d free, %lld "
            "cells, reward +%.4g, flown %.0f -> %.0f m of %.0f m (%lld rejected, %d "
            "measurements).\n",
            R, nAnchors, nHarvest, static_cast<long long>(uniqueServed.size()), out.info.reward,
            L0, Lcur, budgetDist, static_cast<long long>(rejected.size()), nTrials);
    }
    return out;
}

}  // namespace mtl::planning
