// =============================================================================
//  mtl/planning/info_aware.hpp
//
//  THE INFORMATION-AWARE ABSTRACTION SEARCH (PlannerParams::infoAware).
//
//  The plain pipeline commits to ONE abstraction - k-means clusters of radius
//  maxClusterRadius - and then optimises the route over it against its own
//  proxy objective.  This searches over abstractions instead, and judges every
//  one by what its flown plan would actually detect:
//
//    candidates  the plain plan ("baseline"); k-means and peak/level clusters
//                (mapping::clusterByPeaks, every infoAware.levelSets entry) at
//                the plain radius and at the detection reach(es); each also
//                re-planned with infoAware.restarts extra orienteering seeds;
//    score       CoverageModel::detectedMass of the scheduled trajectories;
//    moves       on the best candidate: PEEL a chosen cluster (keep its densest
//                cells, hand the rest to a new cluster the route may drop),
//                SPLIT it in two, or MERGE it with its nearest unchosen
//                neighbour when the union still fits the reach - re-planned,
//                kept only if the score rises.
//
//  Every plan is a full, budget-verified plan of the ordinary pipeline
//  (Planner::planFromClusters), so the budget guarantee, the gimbal schedule
//  and the geometry are exactly those of the plain planner.  The search only
//  changes WHICH abstraction the route is planned over.
// =============================================================================
#ifndef MTL_PLANNING_INFO_AWARE_HPP
#define MTL_PLANNING_INFO_AWARE_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

/// Cross-track ground offset [m] at which the boresight's slant range reaches
/// slantMargin * sensor.beta (single-axis: along the tilted swept line;
/// multi-axis: straight out).  0 when even nadir is out of range.
double detectionReach(const PlannerParams& P);

/// Run the search.  `P.infoAware.enabled` is ignored (the sub-plans are plain).
PlanningResult planInfoAware(const PlannerParams& P, const CellSet& cells,
                             const std::vector<Vec2>& agentStarts);

}  // namespace mtl::planning

#endif  // MTL_PLANNING_INFO_AWARE_HPP
