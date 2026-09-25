// =============================================================================
//  mtl/mapping/cells.hpp
//
//  The abstraction ladder the whole planner stands on:
//
//      belief grid  --extractValidCells-->  cells (points to aim at, with mass)
//      cells        --clusterCells------->  macro-clusters (points to fly to)
//      clusters     --computeClusterRewards->  prizes for the orienteering problem
//
//  It exists because the budgeted route problem is NP-hard and its search space
//  is the power set of the nodes times their orderings: hopeless over hundreds
//  of cells, easy over tens of clusters.  The abstraction is EXACT IN REWARD -
//  mass is additive and a drone at a centroid does sweep every cell of that
//  cluster, because clusterCells guarantees each cell is within maxClusterRadius
//  of its centroid - and APPROXIMATE IN COST, charging only centroid-to-centroid
//  travel.  That error is bounded by the cluster radius and absorbed by the
//  Dubins re-costing in planning::planBudgetedMacroRoute, which measures the
//  real flown arc.
// =============================================================================
#ifndef MTL_MAPPING_CELLS_HPP
#define MTL_MAPPING_CELLS_HPP

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::mapping {

// -----------------------------------------------------------------------------
/// Dice the belief field into targetCellSize blocks and keep the centre of every
/// block whose MEAN belief beats the threshold.  Those centres are the points
/// the sensor must be aimed at.
///
/// The mass carried back is the AGGREGATE belief in the cell
/// (sum(belief) * pixelArea), not the mean, because the quantity that matters
/// operationally is expected targets looked at, and that is additive over
/// disjoint cells.  A big weakly-believed cell and a small strongly-believed one
/// then trade off correctly.
///
/// @param belief          the prior grid
/// @param targetCellSize  [m] side of the blocks the map is diced into
/// @param meanInfoThresh  a block is kept when its mean belief exceeds this
// -----------------------------------------------------------------------------
CellSet extractValidCells(const BeliefField& belief, double targetCellSize,
                          double meanInfoThresh, bool verbose = false);

// -----------------------------------------------------------------------------
/// Macro-cluster the valid cells so every cell is within reach of its centroid.
///
/// K is raised until the largest cell-to-centroid distance is at most
/// maxRadius.  The centroids become the macro waypoints the route planner
/// reasons about, and maxRadius is what guarantees the gimbal sweep of a cluster
/// actually covers the cells that cluster owns.
///
/// Fills `centroids`, `cellCluster` and `maxRadius`; reward and cellIdx are
/// filled by computeClusterRewards.
// -----------------------------------------------------------------------------
ClusterSet clusterCells(const Path2& validCenters, double maxRadius,
                        const ClusterParams& opts, std::uint64_t seed,
                        bool verbose = false);

// -----------------------------------------------------------------------------
/// Lift per-cell belief mass onto the macro-clusters: each centroid inherits the
/// summed mass of the cells it owns, which is the prize the orienteering solver
/// maximises.  Fills `reward` and `cellIdx` in place.
// -----------------------------------------------------------------------------
void computeClusterRewards(const VecX& cellMass, ClusterSet& clusters);

}  // namespace mtl::mapping

#endif  // MTL_MAPPING_CELLS_HPP
