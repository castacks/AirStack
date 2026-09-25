// =============================================================================
//  mtl/eval/detection.hpp   (library: mtl_eval)
//
//  SCORING - deliberately NOT part of the planner.
//
//  The planner's own objective is "was the sensor pointed at this cell", which
//  is modular, cheap and exactly what makes the budgeted route an orienteering
//  problem.  THIS is the detection physics: the probability that a target inside
//  the footprint was actually seen, accumulated over time and over agents.  A
//  host simulation with its own sensor model should use that instead - which is
//  why this lives in a separate library the planner does not link.
//
//      f(r) = 1 / (a + exp(b*(r - c)))    for r <= beta,  else pOutOfRange
//
//  with r the SLANT RANGE from the aircraft to the target.  A target is inside
//  the footprint when it is within slantRange*tan(FOV/2) of the boresight ground
//  point - the footprint follows the slant range to the point the boresight is
//  on, not the altitude; the two are equal only at nadir.
// =============================================================================
#ifndef MTL_EVAL_DETECTION_HPP
#define MTL_EVAL_DETECTION_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::eval {

struct DetectionSummary {
    Index  nDetected = 0;
    Index  nMissed   = 0;
    double meanDetectionProb = 0.0;
    std::vector<Index> missedIdx;
};

/// Accumulate detection probability across every agent and every time step,
/// latching each target's detectionTime the first step its cumulative
/// probability crosses `threshold`.  `targets` is updated in place.
///
/// @param trajectories one entry per agent, all padded onto `timeVec`
DetectionSummary updateTargetDetectionProbs(const std::vector<AgentTrajectory>& trajectories,
                                            const VecX& timeVec, std::vector<Target>& targets,
                                            double fov, const SensorModelParams& sensor,
                                            double threshold);

/// Single-agent variant, for a host stepping one aircraft at a time.
DetectionSummary updateTargetDetectionProbs(const Path3& droneTraj, const Path2& sensorTraj,
                                            const VecX& timeVec, std::vector<Target>& targets,
                                            double fov, const SensorModelParams& sensor,
                                            double threshold);

// -----------------------------------------------------------------------------
/// The belief the search leaves behind: a post-search Bayes update of the prior,
/// pixel by pixel, against every look every agent took.
///
///     residual(x) = prior(x) * prod_t prod_agents (1 - P_t(Z|x))
///                 = P(target is at x AND every look at x missed it)
///
/// with P_t(Z|x) exactly the model updateTargetDetectionProbs applies to the
/// targets: f(slant range) inside the footprint (sensor.pOutOfRangeMulti past
/// beta), zero outside it.  The prior is normalised first, so
///
///     residualMass = residual.sum() = P(the search missed the target)
///
/// which is 1 for a search that looked at nothing and falls toward 0 as the
/// search sees more of the belief, better.  LOWER IS BETTER: this is the number
/// to compare planners on.
// -----------------------------------------------------------------------------
struct ResidualBelief {
    MatX   residual;            ///< same grid as the prior: P(target here AND missed)
    double priorMass    = 0.0;  ///< sum of the normalised prior (1)
    double residualMass = 0.0;  ///< residual.sum(): P(target missed) - the metric
    double detectedMass = 0.0;  ///< priorMass - residualMass
    Index  nLooks = 0;          ///< distinct sensor states integrated
    Index  nSteps = 0;          ///< trajectory steps integrated, all agents

    /// The Bayes posterior given that nothing was found: residual / residualMass.
    MatX posterior() const {
        return residualMass > 0.0 ? (residual / residualMass).eval()
                                  : MatX::Zero(residual.rows(), residual.cols());
    }
};

/// Compute the residual belief over every pixel of `prior` (1 m on the
/// reference map) from the delivered, padded team trajectories.  Runs of
/// identical consecutive states (hover padding, a grounded agent) are
/// integrated in one go, raised to the run length - exactly equivalent to
/// stepping them one by one - each look only touches its footprint, and
/// log(1 - P) is tabulated in the squared slant range (interpolation error far
/// below round-off).  ~12 s single-core on the 5001 x 5001 reference grid;
/// build with MTL_ENABLE_OPENMP to split each footprint across threads.
ResidualBelief computeResidualBelief(const BeliefField& prior,
                                     const std::vector<AgentTrajectory>& trajectories,
                                     double fov, const SensorModelParams& sensor);

/// Circular ground footprint of the camera at one instant: the radius follows
/// the SLANT RANGE to the ground point the boresight is on.
double footprintRadius(const Vec3& dronePos, const Vec2& sensorPos, double fov);

/// Vertices of that footprint, for plotting.
Path2 projectedFootprint(const Vec3& dronePos, const Vec2& sensorPos, double fov, int numPoints = 30);

}  // namespace mtl::eval

#endif  // MTL_EVAL_DETECTION_HPP
