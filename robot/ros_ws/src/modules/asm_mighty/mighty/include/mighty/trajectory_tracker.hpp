#pragma once

#include <cmath>
#include <limits>
#include <mutex>

#include <dynus_interfaces/msg/state.hpp>
#include <dynus_interfaces/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

/** @brief Time-based feedforward + feedback trajectory tracker for ground robots.
 *
 *  Receives the full Trajectory from MIGHTY (on each replan), stores it,
 *  and interpolates at its own servo rate to produce (p_ref, v_ref, yaw_ref,
 *  dyaw_ref).  Feedforward from v_ref/dyaw_ref plus proportional feedback
 *  on position and heading error produces cmd_vel.
 *
 *  This follows the standard robotics architecture (PX4, MoveIt, Nav2):
 *  planner publishes full trajectory, controller owns the clock.
 */
class TrajectoryTracker : public rclcpp::Node {
 public:
  TrajectoryTracker();

 private:
  // Callbacks
  void trajectoryCallback(const dynus_interfaces::msg::Trajectory::SharedPtr msg);
  void stateCallback(const dynus_interfaces::msg::State::SharedPtr msg);
  void controlCallback();

  // Interpolation: given elapsed time, produce reference setpoint
  struct RefPoint {
    double x, y, vx, vy, yaw, dyaw;
  };
  RefPoint interpolateTrajectory(double t_elapsed);

  // Helpers
  double wrapPi(double angle);
  double yawFromQuat(const geometry_msgs::msg::Quaternion& q);

  // Publishers and Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ref_marker_;
  rclcpp::Subscription<dynus_interfaces::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<dynus_interfaces::msg::State>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  dynus_interfaces::msg::State current_state_;
  dynus_interfaces::msg::Trajectory trajectory_;
  bool state_initialized_ = false;
  bool trajectory_initialized_ = false;

  // Trajectory timing
  rclcpp::Time traj_start_time_;     // wall time when current trajectory was received
  uint32_t current_traj_id_ = 0;    // detect replans via trajectory_id

  // Parameters
  double control_rate_;
  double max_velocity_;
  double max_angular_velocity_;
  double stopping_radius_;
  double Kp_along_;
  double Kp_cross_;
  double Kp_yaw_;
  double w_smoothing_;
  bool use_hardware_;
  std::string map_frame_id_;

  // Control state
  double prev_v_cmd_ = 0.0;
  double prev_w_cmd_ = 0.0;
};
