// =============================================================================
//  mtl/planning/info_score.hpp
//
//  How good is a route?  The scalar the budgeted planner maximises:
//
//      J(route) = sum over cells serviced by the route of mass(cell)
//
//  Aggregate, not mean, because the quantity that matters operationally is
//  expected targets looked at, and that is additive over disjoint cells.  Two
//  consequences make the budgeted route non-trivial and are worth being explicit
//  about:
//
//    * J is MODULAR (a plain sum), not submodular.  Servicing a cell twice earns
//      nothing extra, and the serviced set is determined by which clusters the
//      route reaches, so the optimisation is exactly an orienteering problem -
//      maximise an additive prize under a path-length budget - and not a general
//      adaptive informative-path problem.
//    * J ignores WHEN a cell is seen and ignores detection physics.  It is the
//      "you were pointed at it" score.  Pass `observed` and the same sum is
//      reported over the cells the gimbal scheduler actually managed to service,
//      so the gap between planned and realized is visible rather than hidden.
// =============================================================================
#ifndef MTL_PLANNING_INFO_SCORE_HPP
#define MTL_PLANNING_INFO_SCORE_HPP

#include <vector>

#include "mtl/types.hpp"

namespace mtl::planning {

struct InfoScoreOptions {
    double budget       = std::numeric_limits<double>::quiet_NaN();
    double totalMapMass = std::numeric_limits<double>::quiet_NaN();
    const std::vector<Index>* observed = nullptr;  ///< cells actually observed
};

/// @param cellMass     M-by-1 per-cell mass
/// @param serviced     global indices of the cells this route services
/// @param routeLength  flown track length [m] (NaN if not known yet)
InfoScore pathInformationScore(const VecX& cellMass, const std::vector<Index>& serviced,
                               double routeLength, const InfoScoreOptions& opts = {});

}  // namespace mtl::planning

#endif  // MTL_PLANNING_INFO_SCORE_HPP
