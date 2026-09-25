// =============================================================================
//  mtl/planning/orienteering.hpp
//
//  MAX INFORMATION UNDER A PATH-LENGTH BUDGET.
//
//  Given nodes with prizes, a fixed origin and a maximum path length, choose
//  WHICH nodes to visit and IN WHAT ORDER so the collected prize is maximal and
//  the path fits.  This is the ORIENTEERING PROBLEM (selective TSP / TSP with
//  profits), and it is the correct formalisation of "don't just truncate the TSP
//  tour": truncation fixes the ORDER first and then cuts, which is exactly the
//  mistake - the order that is optimal for visiting everything is generally a
//  poor prefix.  Here the subset and the order are chosen together.
//
//  Open path by default: the drone stops at its last serviced cluster and does
//  not fly home (set OrienteeringParams::endPos to close it).
//
//  THE HEURISTIC (orienteering is NP-hard; this is a strong, cheap heuristic)
//  -------------------------------------------------------------------------
//  Multi-start, each start driven to a local optimum under four move families
//  that attack the four ways a solution can be bad:
//
//    1. GREEDY RATIO INSERTION - repeatedly insert the unvisited node with the
//       best prize per metre of DETOUR it causes.  Cheapest-insertion, not
//       nearest-neighbour: a node is judged by what it costs where it best fits.
//    2. SHORTENING - 2-opt plus Or-opt (segment relocation, length 1-3, forward
//       and reversed) on the CHOSEN subset.  It does not change the prize; it
//       frees budget, which then lets step 1 add more nodes.  The two alternate.
//    3. REPLACEMENT - swap a chosen node for an unchosen one of higher prize
//       wherever the length allows.  This fixes a greedy run that committed
//       early to cheap, low-value nodes.
//    4. RUIN AND RECREATE - tear 1-3 chosen nodes out of the incumbent at random
//       and rebuild.  The only move family that can trade a FEW expensive
//       high-prize nodes for MANY cheap ones, which 1-for-1 replacement
//       structurally cannot see.
//
//  Starts differ by insertion criterion and randomisation: start 1 is pure
//  greedy on prize/detour (deterministic), start 2 is greedy on prize^2/detour,
//  and the rest sample from a restricted candidate list - the classic GRASP
//  construction.  Best over all starts wins, ties broken by shorter path.
//
//  Cost is EUCLIDEAN here, deliberately: 2-opt and cheapest insertion need the
//  triangle inequality.  The flown track is a Dubins path and is longer;
//  planBudgetedMacroRoute wraps this solver in a re-costing loop that measures
//  the real arc and shrinks the budget handed here until the real arc fits.
// =============================================================================
#ifndef MTL_PLANNING_ORIENTEERING_HPP
#define MTL_PLANNING_ORIENTEERING_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

struct OrienteeringSolution {
    Path2              routeXY;     ///< [startPos; nodes(visitOrder); endPos?]
    std::vector<Index> visitOrder;  ///< indices into `nodes`, in visit order
    OrienteeringInfo   info;
};

/// @param nodes    K-by-2 candidate positions (macro-cluster centroids)
/// @param rewards  K-by-1 prize per node
/// @param startPos origin.  Always visited, never counted as prize.
/// @param budget   maximum path length [m].  Inf selects every node.
OrienteeringSolution solveBudgetedOrienteering(const Path2& nodes, const VecX& rewards,
                                               const Vec2& startPos, double budget,
                                               const OrienteeringParams& opts);

}  // namespace mtl::planning

#endif  // MTL_PLANNING_ORIENTEERING_HPP
