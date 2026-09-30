// =============================================================================
//  mtl/planning/coverage_score.hpp
//
//  WHAT A FLOWN PLAN ACTUALLY DETECTS, AS SEEN FROM THE CELLS.
//
//  The route planner's own objective is "the boresight was aimed at this
//  cell's centre".  The metric a search is judged by (eval::computeResidualBelief)
//  is different: every look Bayes-updates every point its footprint covers
//  with the detection sigmoid in slant range, so it credits the footprint's
//  whole disc, discounts range, and does not pay twice for a point seen twice.
//
//  This is that metric computed from what the planner has - the cells - rather
//  than the prior raster the planner never sees: each cell is spread uniformly
//  over a subsample x subsample lattice of points inside its block, and the
//  looks of the scheduled trajectory are integrated over those points exactly
//  as computeResidualBelief integrates them over pixels (same footprint rule,
//  same sigmoid, same run-length weighting; one look in `lookStride` is taken
//  and weighted by the stride).  It is what planning::planInfoAware ranks
//  candidate plans by.  It lives in the planner library (not mtl_eval) so a
//  host that links mtl::planner alone still gets the search.
// =============================================================================
#ifndef MTL_PLANNING_COVERAGE_SCORE_HPP
#define MTL_PLANNING_COVERAGE_SCORE_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

class CoverageModel {
public:
    CoverageModel(const CellSet& cells, int subsample, double fov, const SensorModelParams& sensor);

    /// Expected belief mass detected by the team, sum over cells of
    /// mass * (1 - P(every look missed)), in the cells' units (a probability
    /// for a normalised prior).  Higher is better.
    double detectedMass(const std::vector<AgentTrajectory>& trajectories, int lookStride) const;

    /// Per-point miss log-likelihood after the looks (for tests / diagnostics).
    VecX logMiss(const std::vector<AgentTrajectory>& trajectories, int lookStride) const;

    Index numPoints() const { return pts_.rows(); }
    double totalMass() const { return w_.sum(); }

private:
    Path2 pts_;        ///< sample points
    VecX  w_;          ///< their mass
    double fov_;
    SensorModelParams sensor_;
    // uniform bucket grid over the points
    double bx0_ = 0.0, by0_ = 0.0, bs_ = 1.0;
    Index  nbx_ = 0, nby_ = 0;
    std::vector<std::vector<Index>> buckets_;
};

}  // namespace mtl::planning

#endif  // MTL_PLANNING_COVERAGE_SCORE_HPP
