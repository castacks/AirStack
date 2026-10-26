// =============================================================================
//  mtl_curve/curve_planning/info_aware.hpp
//
//  THE INFORMATION-AWARE CLUSTER SEARCH, CURVE-PLANNER SIDE
//  (PlannerParams::infoAware).
//
//  The plain curve planner commits to ONE abstraction - k-means clusters of
//  radius maxClusterRadius, grouped by proximity alone - and seeds its curves
//  from it.  This builds several abstractions that follow the prior instead
//  (mapping::clusterByPeaks: peaks -> mass levels -> split under a reach) and
//  lets the planner pick the one whose seeds search best.  It is the curve
//  twin of cpp_planner's planning::planInfoAware (AirStack mtl_search_planner);
//  the abstraction code is the same, the SCORE differs: there a candidate is a
//  full plan scored by a coverage model, here a candidate is scored by the
//  fast-model team objective of its seed curves (team_curves seedOnly), which
//  costs one seed construction instead of an optimisation.
//
//  This header holds the abstraction side (candidates, moves); the loop that
//  scores them lives in Planner (planner.cpp), which owns the team solve.
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_INFO_AWARE_HPP
#define MTLC_CURVE_PLANNING_INFO_AWARE_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

struct AbstractionCandidate {
    std::string label;
    double      reach = 0.0;   ///< [m] cluster radius it was built with
    ClusterSet  clusters;      ///< reward and cellIdx filled
};

/// The candidate abstractions, in a fixed order:
///   "plain"  the clusters given (the planner's own maxClusterRadius k-means);
///   for each reach r in {maxClusterRadius} U {reachScales * swathHalfWidth}:
///     "kmeans@r"  (only for the extra reaches, with includeKmeans)
///     "peaks[f..]@r" for every levelSets entry.
/// Reaches within 1 m of an earlier one are skipped.  `basins` is filled.
std::vector<AbstractionCandidate> infoAwareCandidates(const CellSet& cells, const ClusterSet& plain,
                                                      double swathHalfWidth, const PlannerParams& P,
                                                      Index* basins = nullptr);

/// PEEL / SPLIT / MERGE neighbourhood of an abstraction, on the clusters the
/// seeds actually visited (`visited`), richest first - the cpp_planner moves
/// verbatim.  PEEL keeps the densest cells holding peelKeep of a cluster's mass
/// and hands the rest (split under `reach`) to new clusters the route may
/// drop; SPLIT 2-means a cluster; MERGE joins it with its nearest unvisited
/// neighbour when the union still fits `reach`.
std::vector<std::pair<std::string, ClusterSet>> infoAwareMoves(const ClusterSet& cl,
                                                               const std::vector<Index>& visited,
                                                               const CellSet& cells, double reach,
                                                               const InfoAwareParams& ia,
                                                               const ClusterParams& cp, std::uint64_t seed);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_INFO_AWARE_HPP
