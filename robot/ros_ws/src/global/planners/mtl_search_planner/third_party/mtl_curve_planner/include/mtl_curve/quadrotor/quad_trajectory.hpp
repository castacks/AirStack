// =============================================================================
//  mtl_curve/quadrotor/quad_trajectory.hpp
//
//  Quadrotor (body-fixed camera) version of a curve-planner plan - the port of
//  quadReference.m, optimizeQuadFlatTrajectory.m, generateQuadTrajectories.m
//  and quadMatchedLooks.m.
//
//  The fixed wing flies the curve while its 1-DOF gimbal sweeps the boresight
//  across track.  A quadrotor with a fixed camera has no gimbal: to look left
//  it must accelerate right (the body banks, the camera under it swings), to
//  look further ahead it must pitch up, i.e. brake; yaw is free and swings a
//  forward-tilted camera across track without moving the vehicle.  So the
//  trajectory itself is re-planned.  The flat outputs x(t), y(t), psi(t) are
//  uniform quintic B-splines in time, the altitude is held, and their
//  coefficients solve
//
//    min  sum_k |b(t_k) - b_ref(t_k)|^2/sb^2                boresight on the plan's ground points
//       + (n.(p - p_ref))^2/sc^2 + (t.(p - p_ref))^2/sa^2    on the curve, on time
//       + (psi - psi_track)^2/spsi^2 + |p''''|^2/ssnap^2 + psi''^2/spsidd^2
//       + start state + w * hinge^2(tilt, |(p,q)|, |r|, |r'|, |v|, look)
//
//  by Levenberg-Marquardt on the sparse banded normal equations (Jacobian =
//  B-spline basis chained with per-sample finite-difference derivatives of the
//  flatness map), escalating the hinge weights until every TRUE limit holds.
// =============================================================================
#ifndef MTLC_QUADROTOR_QUAD_TRAJECTORY_HPP
#define MTLC_QUADROTOR_QUAD_TRAJECTORY_HPP

#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::quadrotor {

/// The fixed-wing plan re-timed for the quad's cruise speed: arc length
/// s(t) = Vq t, path p_ref(t) = curve(s(t)), boresight b_ref(t) = the gimbal's
/// ground point at that arc length (same ground pattern, stretched timeline).
struct QuadReference {
    VecX  s;        ///< arc length of the kept fixed-wing samples
    Path2 P, B;     ///< curve and gimbal ground points
    VecX  psi;      ///< unwrapped track heading
    double h = 0.0, Vq = 0.0, T = 0.0;
};

struct QuadReferenceSample {
    Path2 P, B, tan;
    VecX  psi, bearing, s;   ///< bearing: drone -> ground point, on psi's branch
};

/// @param nSteps unpadded sample count (AgentTrajectory::steps)
QuadReference quadReference(const Path3& drone, const Path2& sensor, Index nSteps, double Vq);
QuadReferenceSample sampleReference(const QuadReference& ref, const VecX& t);

/// One agent.  `q` must be resolved (resolveQuadParams); `dtOut` the output step.
QuadTrajectory optimizeQuadFlatTrajectory(const QuadReference& ref, double beta, const QuadParams& q, double dtOut);

/// The team: every agent of a generateCurveTrajectories output, padded to the
/// longest quad trajectory; timeVec = the team quad timeline.
std::vector<QuadTrajectory> generateQuadTrajectories(const std::vector<AgentTrajectory>& fixedWing, double Vfw,
                                                     double dt, double fwTiltAngle, double beta,
                                                     const QuadParams& params, VecX& timeVec);

/// The AgentTrajectory views of a quad team (what eval:: takes).
std::vector<AgentTrajectory> quadAgentTrajectories(const std::vector<QuadTrajectory>& quad);

/// The quad team sampled at the fixed wing's look instants (same arc lengths,
/// same look count): the evaluators count every step as a look, so this
/// isolates the camera GEOMETRY from the slower quad's extra looks.
std::vector<AgentTrajectory> quadMatchedLooks(const std::vector<QuadTrajectory>& quad,
                                              const std::vector<AgentTrajectory>& fixedWing, double Vfw,
                                              double dt, double Vq, VecX& timeVec);

}  // namespace mtl::curve::quadrotor

#endif  // MTLC_QUADROTOR_QUAD_TRAJECTORY_HPP
