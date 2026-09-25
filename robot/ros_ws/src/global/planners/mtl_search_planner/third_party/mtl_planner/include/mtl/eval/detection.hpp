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

/// Circular ground footprint of the camera at one instant: the radius follows
/// the SLANT RANGE to the ground point the boresight is on.
double footprintRadius(const Vec3& dronePos, const Vec2& sensorPos, double fov);

/// Vertices of that footprint, for plotting.
Path2 projectedFootprint(const Vec3& dronePos, const Vec2& sensorPos, double fov, int numPoints = 30);

}  // namespace mtl::eval

#endif  // MTL_EVAL_DETECTION_HPP
