// =============================================================================
//  mtl/planning/cell_anchors.hpp
//
//  Spend the budget crumbs on leftover cells, gimbal-aware.
//
//  WHY
//  ---
//  The cluster abstraction is what makes the budgeted route tractable, but it is
//  coarse: clusters are all-or-nothing, so a cluster whose centroid detour costs
//  900 m is refused whole even when 300 m of budget is going spare and one of
//  its cells sits 120 m off the route.  This pass reclaims exactly that.
//
//  THE GIMBAL, WHICH A NAIVE VERSION OF THIS PASS IGNORES
//  -----------------------------------------------------
//  Pricing a leftover cell as if the AIRCRAFT had to overfly it, and paying it
//  the mass of that ONE cell, is wrong on both halves for a gimballed payload:
//
//    (a) the aircraft does not have to reach the cell centre.  It only has to
//        come within GIMBALREACH of it - the very guarantee clusterCells
//        enforces when it grows K until every cell is inside maxClusterRadius of
//        its centroid.  A cell 620 m off the track with a 500 m reach costs a
//        120 m stand-off detour, not a 620 m one.
//    (b) once the aircraft IS at an anchor point, the gimbal reaches every
//        leftover cell within GIMBALREACH of that point, not just the one that
//        motivated the detour.  An accepted anchor is therefore a NEW ONE-OFF
//        CLUSTER, priced by the whole ball's mass.
//
//  Downstream this needs no special casing: generateTrajectories opens a micro
//  sweep window around every macro row, so an anchor row behaves exactly like a
//  real cluster row.
//
//  THE THREE MOVES, CHEAPEST FIRST
//  -------------------------------
//   1. FREE HARVEST - a leftover cell already inside GIMBALREACH of a waypoint
//      the route flies anyway is pure profit, attached for zero extra metres.
//   2. ANCHOR AT FULL REACH - splice a new waypoint and sweep the ball around
//      it.  Placement is a stand-off ladder (standoffTiers, as fractions of
//      GIMBALREACH): tier 1 stops the aircraft the instant the seed cell enters
//      reach, tier 0 flies right over it and centres a bigger ball on it.
//   3. SHIFT THE TRACK - falls out of the ladder for free: when the full detour
//      busts the budget the cheaper rungs are still on the table, i.e. the route
//      is nudged toward the cell rather than over it.
//
//  HOW THE BUDGET STAYS HARD
//  -------------------------
//  Insertion is RANKED on the Euclidean detour scaled by the route's measured
//  Dubins inflation ratio (fast, and enough to rank), but every accepted
//  insertion is then RE-MEASURED with the real length function.  One that busts
//  the budget is rolled back, and the detour it was refused at becomes that
//  candidate's ceiling, so only strictly cheaper rungs are retried.  The route
//  returned has been measured feasible.
// =============================================================================
#ifndef MTL_PLANNING_CELL_ANCHORS_HPP
#define MTL_PLANNING_CELL_ANCHORS_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/planning/macro_route.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

struct CellAnchorResult {
    Path2              routeXY;     ///< route with the accepted anchors spliced in
    std::vector<Index> addedCells;  ///< indices into `cellsXY` this pass got serviced
    CellRefineInfo     info;

    /// One entry per row of the RETURNED route:
    ///    0   the launch point (row 0)
    ///   -q   row q of the route that came IN (q >= 1)
    ///   +j   an anchor waypoint seeded by cellsXY.row(j)
    /// Without it the caller cannot tell which route row owns which cells after
    /// the splices have shuffled the rows.
    std::vector<int> rowTag;
    /// One entry per row of the RETURNED route: the indices into `cellsXY` that
    /// THIS PASS made that row responsible for.  For an anchor row that is its
    /// whole reach ball; for an incoming row it is the free harvest, which the
    /// caller must APPEND to that row's existing cells, not replace them with.
    std::vector<std::vector<Index>> rowCells;
};

/// @param routeXY       (1+n)-by-2 route so far, row 0 = launch point.  Existing
///                      rows are never reordered or removed.
/// @param cellsXY       C-by-2 candidate cell centres
/// @param cellRewards   C-by-1 information mass of each candidate
/// @param budgetDist    max flown distance [m] (Inf disables the pass)
CellAnchorResult refineWithCellAnchors(const Path2& routeXY, const Path2& cellsXY,
                                       const VecX& cellRewards, double budgetDist,
                                       double minTurnRadius, const CellAnchorParams& opts,
                                       const DubinsParams& dubinsOpts,
                                       const LengthFcn& lengthFcn = nullptr);

}  // namespace mtl::planning

#endif  // MTL_PLANNING_CELL_ANCHORS_HPP
