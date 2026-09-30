// =============================================================================
//  mtl_curve/eval/report.hpp   (library: mtl_curve_eval)
//
//  Console reports: the per-agent curve / kinematic acceptance table (the
//  report main_curve_planner.m prints), the target detections and the residual
//  belief.  reportDetectionSummary and reportResidualBelief follow cpp_planner's
//  mtl/eval/report.hpp.
// =============================================================================
#ifndef MTLC_EVAL_REPORT_HPP
#define MTLC_EVAL_REPORT_HPP

#include <ostream>
#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::eval {

/// Per-agent curve table + acceptance checks 1-3 (curvature, length,
/// endpoints) + the team solve's history.  Returns true when every agent passes.
bool reportCurveSummary(std::ostream& os, const PlanningResult& result, const PlannerParams& params);

void reportDetectionSummary(std::ostream& os, const std::vector<Target>& targets, const PlannerParams& params);

struct ResidualBelief;  // mtl_curve/eval/detection.hpp

/// The post-search residual belief - the planner-comparison metric, lower is better.
void reportResidualBelief(std::ostream& os, const ResidualBelief& residual);

}  // namespace mtl::curve::eval

#endif  // MTLC_EVAL_REPORT_HPP
