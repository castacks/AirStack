// =============================================================================
//  mtl/mapping/peak_clusters.hpp
//
//  AN ABSTRACTION THAT FOLLOWS THE BELIEF, NOT JUST THE MAP.
//
//  mapping::clusterCells groups cells by proximity alone.  On a peaked prior a
//  proximity cluster straddles a peak's dense core and its thin tail, and the
//  route can only take both or neither.  This builds the clusters from the
//  shape of the prior instead:
//
//    BASINS.  The valid cells sit on a lattice.  Every cell points at its
//    highest-mass lattice neighbour (itself if it is a local maximum); the
//    cells that climb to the same top form that peak's basin - a discrete
//    watershed.  Two basins are merged when the best saddle between them (the
//    lower of the two cells on the highest edge that crosses) is at least
//    `persistence` times the lower peak: a shallow dip is one peak with two
//    bumps, a deep one is two peaks.
//
//    LEVELS.  Each basin's cells are sorted by mass, densest first, and cut at
//    the cumulative-mass fractions `levelFracs` (e.g. {0.5, 1}: the core that
//    holds half the basin's mass, then the rest).  A core is compact and dense,
//    a shoulder is the ring around it, so the orienteering problem over these
//    nodes chooses HOW FAR DOWN each peak to go: the levels are a discretised
//    profit curve, with diminishing returns built in.
//
//    REACH.  Every (basin, level) set is then split spatially exactly like
//    clusterCells - k raised until every cell is within `maxRadius` of its
//    centroid - so the gimbal guarantee a cluster rests on still holds.
//
//  The result is an ordinary ClusterSet (the route planner needs nothing new),
//  with ClusterSet::basin / level / parent filled in.
// =============================================================================
#ifndef MTL_MAPPING_PEAK_CLUSTERS_HPP
#define MTL_MAPPING_PEAK_CLUSTERS_HPP

#include <cstdint>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::mapping {

/// The prior's peaks, as a partition of the valid cells.
struct PeakBasins {
    std::vector<int>    basinOfCell;  ///< M-by-1 basin id (0-based, dense)
    std::vector<Index>  peakCell;     ///< per basin: the cell at its top
    std::vector<double> mass;         ///< per basin: summed cell mass
    Index size() const { return static_cast<Index>(peakCell.size()); }
};

/// Discrete watershed of the cell masses on the cell lattice, with
/// persistence merging.  Cells closer than 1.5 * cellSize are neighbours
/// (8-connectivity on the regular lattice).
PeakBasins findPeakBasins(const CellSet& cells, double persistence);

/// Build the information-aware abstraction: basins -> mass levels -> spatial
/// split under maxRadius.  `levelFracs` must be increasing and end at 1.
/// Fills reward and cellIdx as computeClusterRewards does.
ClusterSet clusterByPeaks(const CellSet& cells, const PeakBasins& basins,
                          const std::vector<double>& levelFracs, double maxRadius,
                          const ClusterParams& opts, std::uint64_t seed);

/// Split an arbitrary set of cells (global indices) under maxRadius, the rule
/// clusterCells applies to the whole map.  Returns one list of global indices
/// per resulting cluster.
std::vector<std::vector<Index>> splitUnderRadius(const CellSet& cells,
                                                 const std::vector<Index>& members,
                                                 double maxRadius, const ClusterParams& opts,
                                                 std::uint64_t seed);

/// Rebuild centroids, reward, cellIdx, cellCluster and maxRadius of a
/// ClusterSet from a list of member lists (global cell indices).  basin /
/// level / parent are carried over from the per-cluster vectors given (may be
/// empty).
ClusterSet clusterSetFromMembers(const CellSet& cells,
                                 const std::vector<std::vector<Index>>& members,
                                 const std::vector<int>& basin = {},
                                 const std::vector<int>& level = {});

}  // namespace mtl::mapping

#endif  // MTL_MAPPING_PEAK_CLUSTERS_HPP
