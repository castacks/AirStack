// =============================================================================
//  mtl/eval/report.hpp   (library: mtl_eval)
//
//  What the budget bought, per agent and team-wide.
//
//  Supplying the REALIZED cells (the planned cells minus the scheduler's misses,
//  which PlanningResult::realizedCellIdx already holds) makes the report show
//  PLANNED versus REALIZED information, which separates two failure modes that
//  otherwise look the same:
//    * information not collected because the BUDGET refused the route, and
//    * information not collected because the single-axis gimbal could not bring
//      a planned cell abeam in time.
// =============================================================================
#ifndef MTL_EVAL_REPORT_HPP
#define MTL_EVAL_REPORT_HPP

#include <ostream>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::eval {

/// Print the budget summary table.  `realized` may be empty.
void reportBudgetSummary(std::ostream& os, const PlanningResult& result,
                         const PlannerParams& params);

/// Print the target-detection summary (requires mtl_eval's detection pass first).
void reportDetectionSummary(std::ostream& os, const std::vector<Target>& targets,
                            const PlannerParams& params);

/// Print the per-agent gimbal-coverage summary (single-axis mode only).
void reportGimbalCoverage(std::ostream& os, const PlanningResult& result);

struct ResidualBelief;  // mtl/eval/detection.hpp

/// Print the post-search residual belief - the planner-comparison metric,
/// lower is better (see computeResidualBelief).
void reportResidualBelief(std::ostream& os, const ResidualBelief& residual);

}  // namespace mtl::eval

#endif  // MTL_EVAL_REPORT_HPP
