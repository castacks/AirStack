// =============================================================================
//  mtl/routing/tsp.hpp
//
//  The two open-path TSP heuristics the pipeline needs, and the sensor path that
//  threads them together.
//
//    calculateTSPPath      the UNBUDGETED macro route: greedy nearest-neighbour
//                          plus 2-opt over every cluster.  Used only when the
//                          budget is infinite, where choosing WHICH clusters to
//                          visit is not a question.
//    calculateMicroTSPPath the in-cluster sweep order, with the entry node
//                          pinned to the incoming leg and the exit node pinned
//                          to the next cluster, so the gimbal does not have to
//                          fly back across the cluster to leave it.
//    buildSensorPathOrdered the two combined: walk the macro route in order and
//                          emit each row's cells in sweep order, carrying the
//                          global cell indices along so a scheduler miss can be
//                          mapped back onto the information report.
// =============================================================================
#ifndef MTL_ROUTING_TSP_HPP
#define MTL_ROUTING_TSP_HPP

#include <optional>
#include <vector>

#include "mtl/types.hpp"

namespace mtl::routing {

/// Open-path TSP over `points` starting from `start`: greedy nearest neighbour
/// then 2-opt.  Returns [start; points in visit order].
Path2 calculateTSPPath(const Path2& points, const Vec2& start);

/// Visit order (indices into `points`) of the same route, without the start row.
std::vector<Index> tspVisitOrder(const Path2& points, const Vec2& start);

/// In-cluster sweep order.  The entry node is the cell closest to `current`, the
/// exit node the cell closest to `next` (when there is a next cluster); 2-opt
/// then shortens the middle without ever moving those two.
/// Returns the permutation of rows of `cells`.
std::vector<Index> calculateMicroTSPOrder(const Path2& cells, const Vec2& current,
                                          const std::optional<Vec2>& next);

// -----------------------------------------------------------------------------
/// Hierarchical sensor path from an EXPLICIT macro-route order.
///
/// @param validCenters   M-by-2 global list of valid-cell centres
/// @param entityCells    one list per route row 1..n of the GLOBAL cell indices
///                       that row is responsible for.  A singleton anchor is
///                       just an entity holding one cell's reach ball.
/// @param routeXY        (1+n)-by-2 macro route, row 0 = launch point
/// @param[out] sensorTargets   S-by-2 cell centres in service order
/// @param[out] sensorMacroRow  S-by-1 route row each centre belongs to (1..n)
/// @param[out] sensorCellIdx   S-by-1 global cell index of each centre
// -----------------------------------------------------------------------------
void buildSensorPathOrdered(const Path2& validCenters,
                            const std::vector<std::vector<Index>>& entityCells,
                            const Path2& routeXY, Path2& sensorTargets,
                            std::vector<Index>& sensorMacroRow,
                            std::vector<Index>& sensorCellIdx);

}  // namespace mtl::routing

#endif  // MTL_ROUTING_TSP_HPP
