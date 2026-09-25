// =============================================================================
//  mtl/mapgen/scenario.hpp   (library: mtl_mapgen)
//
//  SCENARIO GENERATION - deliberately NOT part of the planner.
//
//  A host simulation normally brings its own prior and its own ground truth;
//  this library exists so the package can be run, tested and benchmarked
//  standalone without the planner depending on any of it.  Link mtl_mapgen only
//  if you want the reference scenario.
//
//  The prior is a sum of Gaussian bumps at uniformly random positions, capped
//  from above and floored from below.  The targets are sampled FROM that prior
//  by inverse-CDF, so they concentrate where the belief does - and the planner
//  never sees them, only the belief.
// =============================================================================
#ifndef MTL_MAPGEN_SCENARIO_HPP
#define MTL_MAPGEN_SCENARIO_HPP

#include <cstdint>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::mapgen {

/// Spatial grid and the Gaussian prior belief map.
///
/// @param mapSize  [m] side of the square search area
/// @param cellSize [m] grid resolution; the grid is (mapSize/cellSize + 1)^2
/// @param seed     RNG seed for the bump placement
BeliefField generateBeliefMap(double mapSize, double cellSize, const BeliefMapParams& opts,
                              std::uint64_t seed);

/// Sample `numTargets` ground-truth target positions from the belief map by
/// inverse-CDF over the flattened grid.
std::vector<Target> generateTargetPoses(const BeliefField& belief, int numTargets,
                                        std::uint64_t seed);

}  // namespace mtl::mapgen

#endif  // MTL_MAPGEN_SCENARIO_HPP
