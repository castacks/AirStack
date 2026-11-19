// attitude_feedforward.hpp - the pid_controller's optional attitude feed-forward (header-only,
// no ROS types, so it is unit-testable on its own).
//
// A 6-DOF reference (e.g. mtl_trajectory_follower flying a quadrotor plan whose camera is
// bolted to the airframe) knows the ATTITUDE the vehicle must have at every instant: for a
// multirotor that attitude is what produces the planned acceleration. The cascade
// position -> velocity -> attitude only reaches it through its error terms (and clamps the
// velocity loop's attitude to a few degrees), so the planned roll / pitch are added to the
// velocity loop's output as a feed-forward.
//
// Conventions (those of the command the PID publishes, mav_msgs/RollPitchYawrateThrust in
// base_link_stabilized, and of the reference quaternion, map frame, FLU body):
//   ZYX Euler, roll > 0 = right side down (accelerates toward -y), pitch > 0 = nose DOWN
//   (accelerates toward +x). The reference may be at a different yaw than the vehicle, so
// the thrust DIRECTION of the reference (body z axis in the world) is re-expressed in the
// vehicle's yaw frame before it is turned back into roll / pitch: what matters for the
// translational dynamics is where the thrust points, not the Euler numbers.
#ifndef PID_CONTROLLER_ATTITUDE_FEEDFORWARD_HPP
#define PID_CONTROLLER_ATTITUDE_FEEDFORWARD_HPP

#include <algorithm>
#include <cmath>

namespace pid_controller {

struct TiltAngles {
  double roll = 0.0;   ///< [rad] + right side down
  double pitch = 0.0;  ///< [rad] + nose down
};

/// Roll / pitch, in the frame yawed by `vehicle_yaw`, that tilt the body z axis like the
/// reference attitude quaternion (x, y, z, w; world frame) does.
inline TiltAngles feedforward_tilt(double qx, double qy, double qz, double qw, double vehicle_yaw) {
  const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (!(n > 1e-9)) return {};
  qx /= n; qy /= n; qz /= n; qw /= n;
  // third column of R(q): the body z axis in the world
  const double zx = 2.0 * (qx * qz + qy * qw);
  const double zy = 2.0 * (qy * qz - qx * qw);
  const double zz = 1.0 - 2.0 * (qx * qx + qy * qy);
  // into the vehicle's yaw frame: Rz(-yaw)
  const double c = std::cos(vehicle_yaw), s = std::sin(vehicle_yaw);
  const double bx = c * zx + s * zy;
  const double by = -s * zx + c * zy;
  // z_b = (cos(roll) sin(pitch), -sin(roll), cos(roll) cos(pitch))
  TiltAngles out;
  out.pitch = std::atan2(bx, zz);
  out.roll = std::atan2(-by, std::hypot(bx, zz));
  return out;
}

/// Total tilt-limited command: feedback + feed-forward, each axis clamped to +-max_tilt.
inline TiltAngles combine_tilt(const TiltAngles& feedback, const TiltAngles& ff, double max_tilt) {
  TiltAngles out;
  out.roll = std::max(-max_tilt, std::min(max_tilt, feedback.roll + ff.roll));
  out.pitch = std::max(-max_tilt, std::min(max_tilt, feedback.pitch + ff.pitch));
  return out;
}

/// Hover-normalised collective thrust kept vertical-equivalent when tilted:
/// thrust / (cos(roll) cos(pitch)), the tilt term floored at `min_cos`.
inline double tilt_compensated_thrust(double thrust, double roll, double pitch, double min_cos = 0.5) {
  return thrust / std::max(std::cos(roll) * std::cos(pitch), min_cos);
}

}  // namespace pid_controller

#endif  // PID_CONTROLLER_ATTITUDE_FEEDFORWARD_HPP
