// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <takeoff_landing_planner/takeoff_landing_task.hpp>

#include <airstack_common/ros2_helper.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <cmath>
#include <thread>

bool TakeoffLandingTaskNode::fresh_authority(std::chrono::steady_clock::time_point after)
{
  std::lock_guard<std::mutex> lock(authority_mutex_);
  const auto now = std::chrono::steady_clock::now();
  return is_armed_.load() && has_control_.load() && armed_received_ > after && control_received_ > after &&
    std::chrono::duration<double>(now - armed_received_).count() <= control_state_max_age_s_ &&
    std::chrono::duration<double>(now - control_received_).count() <= control_state_max_age_s_;
}

TakeoffLandingTaskNode::TakeoffLandingTaskNode()
: rclcpp::Node("takeoff_landing_task")
{
  // parameters
  default_takeoff_velocity_ = airstack::get_param(this, "takeoff_velocity", 1.0);
  default_landing_velocity_ = airstack::get_param(this, "landing_velocity", 0.3);
  takeoff_acceptance_distance_ = airstack::get_param(this, "takeoff_acceptance_distance", 0.3);
  takeoff_acceptance_time_ = airstack::get_param(this, "takeoff_acceptance_time", 1.0);
  takeoff_max_horizontal_displacement_ =
    airstack::get_param(this, "takeoff_max_horizontal_displacement", 0.0);
  takeoff_max_altitude_overshoot_ =
    airstack::get_param(this, "takeoff_max_altitude_overshoot", 0.0);
  takeoff_max_vertical_speed_ =
    airstack::get_param(this, "takeoff_max_vertical_speed", 0.0);
  preflight_hold_max_position_error_ =
    airstack::get_param(this, "preflight_hold_max_position_error", 0.1);
  preflight_hold_confirmation_samples_ =
    airstack::get_param(this, "preflight_hold_confirmation_samples", 3);
  preflight_hold_timeout_ = airstack::get_param(this, "preflight_hold_timeout", 2.0);
  control_acquisition_timeout_s_ = airstack::get_param(this, "control_acquisition_timeout_s", 2.0);
  control_state_max_age_s_ = airstack::get_param(this, "control_state_max_age_s", 0.5);
  landing_stationary_distance_ = airstack::get_param(this, "landing_stationary_distance", 0.02);
  landing_acceptance_time_ = airstack::get_param(this, "landing_acceptance_time", 5.0);
  landing_tracking_point_ahead_time_ =
    airstack::get_param(this, "landing_tracking_point_ahead_time", 5.0);
  landing_stall_timeout_s_ = airstack::get_param(this, "landing_stall_timeout_s", 5.0);
  landing_max_duration_s_ = airstack::get_param(this, "landing_max_duration_s", 60.0);
  takeoff_path_roll_ = airstack::get_param(this, "takeoff_path_roll", 0.0) * M_PI / 180.0;
  takeoff_path_pitch_ = airstack::get_param(this, "takeoff_path_pitch", 0.0) * M_PI / 180.0;
  takeoff_path_relative_to_orientation_ =
    airstack::get_param(this, "takeoff_path_relative_to_orientation", false);

  // subscribers
  auto sensor_qos = rclcpp::QoS(1);
  sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  robot_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry", sensor_qos,
    std::bind(&TakeoffLandingTaskNode::odom_callback, this, std::placeholders::_1));

  tracking_point_sub_ = this->create_subscription<airstack_msgs::msg::Odometry>(
    "tracking_point", 1,
    std::bind(&TakeoffLandingTaskNode::tracking_point_callback, this, std::placeholders::_1));

  completion_percentage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "trajectory_completion_percentage", 1,
    std::bind(
      &TakeoffLandingTaskNode::completion_percentage_callback, this, std::placeholders::_1));

  is_armed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "is_armed", 1,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(authority_mutex_);
      is_armed_ = msg->data; armed_received_ = std::chrono::steady_clock::now();
    });

  has_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "has_control", 1,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(authority_mutex_);
      has_control_ = msg->data; control_received_ = std::chrono::steady_clock::now();
    });

  state_estimate_timed_out_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "state_estimate_timed_out", 1,
    [this](std_msgs::msg::Bool::SharedPtr msg) { state_estimate_timed_out_ = msg->data; });

  extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "extended_state", 1,
    [this](mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      landed_state_ = msg->landed_state;
      bool in_air = (msg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR);
      if (!in_air) {
        // Definitive non-airborne state from MAVROS — always publish false.
        std_msgs::msg::Bool airborne_msg;
        airborne_msg.data = false;
        is_airborne_pub_->publish(airborne_msg);
      } else if (!landed_) {
        // IN_AIR and no confirmed landing — publish true.
        std_msgs::msg::Bool airborne_msg;
        airborne_msg.data = true;
        is_airborne_pub_->publish(airborne_msg);
      }
      // If landed_ is set and MAVROS reports IN_AIR, publish nothing:
      // land_execute already published false; MAVROS is transiently wrong.
    });

  // publishers
  traj_override_pub_ =
    this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_override", 1);
  is_airborne_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_airborne", 1);

  // service clients
  traj_mode_client_ =
    this->create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode");
  robot_command_client_ =
    this->create_client<airstack_msgs::srv::RobotCommand>("robot_command");

  // action servers
  takeoff_server_ = rclcpp_action::create_server<TakeoffTask>(
    this, "~/takeoff_task",
    std::bind(&TakeoffLandingTaskNode::takeoff_handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&TakeoffLandingTaskNode::takeoff_handle_cancel, this, std::placeholders::_1),
    std::bind(&TakeoffLandingTaskNode::takeoff_handle_accepted, this, std::placeholders::_1));

  land_server_ = rclcpp_action::create_server<LandTask>(
    this, "~/land_task",
    std::bind(&TakeoffLandingTaskNode::land_handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&TakeoffLandingTaskNode::land_handle_cancel, this, std::placeholders::_1),
    std::bind(&TakeoffLandingTaskNode::land_handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TakeoffLandingTaskNode started");
}

// ─────────────────────────── subscription callbacks ───────────────────────────

void TakeoffLandingTaskNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  robot_odom_ = *msg;
  got_robot_odom_ = true;
}

void TakeoffLandingTaskNode::tracking_point_callback(
  const airstack_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tracking_point_mutex_);
  tracking_point_odom_ = *msg;
  got_tracking_point_ = true;
  ++tracking_point_sequence_;
}

void TakeoffLandingTaskNode::completion_percentage_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(completion_mutex_);
  completion_percentage_ = msg->data;
  got_completion_percentage_ = true;
}

// ─────────────────────────── helper ───────────────────────────────────────────

bool TakeoffLandingTaskNode::set_trajectory_mode(int32_t mode)
{
  if (!traj_mode_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "set_trajectory_mode service not available");
    return false;
  }
  auto request = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
  request->mode = mode;
  auto future = traj_mode_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    traj_mode_client_->remove_pending_request(future);
    RCLCPP_ERROR(this->get_logger(), "set_trajectory_mode response timed out");
    return false;
  }
  return future.get()->success;
}

bool TakeoffLandingTaskNode::send_robot_command(uint8_t command)
{
  return robot_command_disposition(command) == CommandDisposition::ACCEPTED;
}

TakeoffLandingTaskNode::CommandDisposition
TakeoffLandingTaskNode::robot_command_disposition(uint8_t command)
{
  if (!robot_command_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "robot_command service not available");
    return CommandDisposition::NOT_SENT;
  }
  auto request = std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
  request->command = command;
  auto future = robot_command_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    robot_command_client_->remove_pending_request(future);
    RCLCPP_ERROR(this->get_logger(), "robot_command response timed out (outcome unknown)");
    return CommandDisposition::UNCONFIRMED;
  }
  return future.get()->success ? CommandDisposition::ACCEPTED : CommandDisposition::REJECTED;
}

std::string TakeoffLandingTaskNode::contain_takeoff_breach()
{
  const bool hold = set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
  // Still request LAND if the trajectory service is unavailable/unconfirmed.
  // Do not wait for the higher-level recovery action while ascent continues.
  const auto disposition = robot_command_disposition(airstack_msgs::srv::RobotCommand::Request::LAND);
  // The bool interface cannot distinguish an inner MAVROS timeout from refusal.
  // Any sent LAND may still take effect; only NOT_SENT permits trajectory fallback.
  abort_land_handover_ = disposition != CommandDisposition::NOT_SENT;
  const char *token = disposition == CommandDisposition::ACCEPTED ? "ACCEPTED" :
    disposition == CommandDisposition::REJECTED ? "FAILED_OR_UNCONFIRMED" :
    disposition == CommandDisposition::NOT_SENT ? "NOT_SENT" : "UNCONFIRMED";
  RCLCPP_ERROR(this->get_logger(), "takeoff_abort hold=%s abort_land=%s grounding=UNVERIFIED",
    hold ? "ACCEPTED" : "UNCONFIRMED", token);
  return std::string("; abort_hold=") + (hold ? "ACCEPTED" : "UNCONFIRMED") +
    "; abort_land=" + token + "; grounding=UNVERIFIED";
}

bool TakeoffLandingTaskNode::confirm_tracking_point_hold()
{
  if (preflight_hold_max_position_error_ <= 0.0 ||
    preflight_hold_confirmation_samples_ <= 0 || preflight_hold_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid preflight hold configuration");
    return false;
  }

  uint64_t last_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(tracking_point_mutex_);
    last_sequence = tracking_point_sequence_;
  }
  int matching_samples = 0;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(preflight_hold_timeout_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    nav_msgs::msg::Odometry odom;
    airstack_msgs::msg::Odometry tracking;
    uint64_t sequence;
    bool have_tracking;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = robot_odom_;
    }
    {
      std::lock_guard<std::mutex> lock(tracking_point_mutex_);
      tracking = tracking_point_odom_;
      sequence = tracking_point_sequence_;
      have_tracking = got_tracking_point_;
    }
    if (have_tracking && sequence != last_sequence) {
      last_sequence = sequence;
      const double position_error = std::sqrt(
        std::pow(tracking.pose.position.x - odom.pose.pose.position.x, 2) +
        std::pow(tracking.pose.position.y - odom.pose.pose.position.y, 2) +
        std::pow(tracking.pose.position.z - odom.pose.pose.position.z, 2));
      const bool frame_matches = tracking.header.frame_id == odom.header.frame_id;
      matching_samples = frame_matches && position_error <= preflight_hold_max_position_error_ ?
        matching_samples + 1 : 0;
      if (matching_samples >= preflight_hold_confirmation_samples_) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_ERROR(this->get_logger(),
    "Preflight hold not confirmed: tracking point did not match current odometry");
  return false;
}

// ─────────────────────────── TakeoffTask ──────────────────────────────────────

rclcpp_action::GoalResponse TakeoffLandingTaskNode::takeoff_handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const TakeoffTask::Goal> goal)
{
  if (task_active_) {
    RCLCPP_WARN(this->get_logger(), "TakeoffTask rejected: another task is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state_estimate_timed_out_) {
    RCLCPP_WARN(this->get_logger(), "TakeoffTask rejected: state estimate timed out");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->target_altitude_m <= 0.0f) {
    RCLCPP_WARN(this->get_logger(), "TakeoffTask rejected: target_altitude_m must be positive");
    return rclcpp_action::GoalResponse::REJECT;
  }
  task_active_ = true;
  cancel_requested_ = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TakeoffLandingTaskNode::takeoff_handle_cancel(
  std::shared_ptr<TakeoffGoalHandle>)
{
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TakeoffLandingTaskNode::takeoff_handle_accepted(std::shared_ptr<TakeoffGoalHandle> goal_handle)
{
  std::thread{
    std::bind(&TakeoffLandingTaskNode::takeoff_execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void TakeoffLandingTaskNode::takeoff_execute(std::shared_ptr<TakeoffGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  float target_altitude = goal->target_altitude_m;
  float velocity = (goal->velocity_m_s > 0.0f) ? goal->velocity_m_s : default_takeoff_velocity_;

  auto result = std::make_shared<TakeoffTask::Result>();
  auto feedback = std::make_shared<TakeoffTask::Feedback>();
  feedback->target_altitude_m = target_altitude;

  landed_ = false;  // clear latch — a new takeoff is starting
  abort_land_handover_ = false;

  // wait for odometry
  rclcpp::Rate wait_rate(10);
  int wait_count = 0;
  while (!got_robot_odom_ && rclcpp::ok()) {
    if (wait_count++ > 50) {
      RCLCPP_ERROR(this->get_logger(), "TakeoffTask aborted: no odometry received");
      result->success = false;
      result->message = "no odometry received";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }
    wait_rate.sleep();
  }

  // Neutralize any setpoint retained across a simulator reset before arming or
  // requesting offboard control. Do not proceed until fresh controller output tracks
  // the current physical pose for several consecutive samples.
  if (!set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE) ||
    !confirm_tracking_point_hold())
  {
    result->success = false;
    result->message = "preflight hold not confirmed";
    goal_handle->abort(result);
    task_active_ = false;
    return;
  }

  if (!std::isfinite(control_acquisition_timeout_s_) || control_acquisition_timeout_s_ <= 0.0 ||
    !std::isfinite(control_state_max_age_s_) || control_state_max_age_s_ <= 0.0)
  {
    result->success = false;
    result->message = "invalid control authority timing configuration";
    goal_handle->abort(result); task_active_ = false; return;
  }

  // arm the robot
  if (!is_armed_) {
    RCLCPP_INFO(this->get_logger(), "TakeoffTask: arming robot");
    const auto arm_disposition = robot_command_disposition(airstack_msgs::srv::RobotCommand::Request::ARM);
    if (arm_disposition != CommandDisposition::ACCEPTED) {
      RCLCPP_ERROR(this->get_logger(), "TakeoffTask aborted: failed to arm");
      result->success = false;
      result->message = arm_disposition == CommandDisposition::NOT_SENT ?
        "arm request not sent" : "failed or unconfirmed arm request; " + contain_takeoff_breach();
      if (cancel_requested_) { goal_handle->canceled(result); }
      else { goal_handle->abort(result); }
      task_active_ = false;
      return;
    }
  }

  // request offboard control
  // Always make a fresh control request after arming. The cached has_control
  // flag can describe a previous disarmed OFFBOARD session.
  RCLCPP_INFO(this->get_logger(), "TakeoffTask: requesting offboard control");
  const auto control_requested_at = std::chrono::steady_clock::now();
  if (!send_robot_command(airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL)) {
    RCLCPP_ERROR(this->get_logger(), "TakeoffTask aborted: failed to request offboard control");
    result->success = false;
    // The boolean interface may hide an inner MAVROS timeout. A request can
    // have reached PX4 even when its response was not observed.
    result->message = "failed or unconfirmed offboard control request; " + contain_takeoff_breach();
    if (cancel_requested_) { goal_handle->canceled(result); }
    else { goal_handle->abort(result); }
    task_active_ = false;
    return;
  }

  // Require post-request armed/control observations, not service acceptance.
  const auto authority_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(control_acquisition_timeout_s_);
  while (rclcpp::ok() && !cancel_requested_ && !fresh_authority(control_requested_at) &&
    std::chrono::steady_clock::now() < authority_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!rclcpp::ok() || cancel_requested_ || !fresh_authority(control_requested_at)) {
    result->success = false;
    result->message = "control authority not observed before ascent; " + contain_takeoff_breach();
    if (cancel_requested_) { goal_handle->canceled(result); }
    else { goal_handle->abort(result); }
    task_active_ = false; return;
  }

  // set trajectory mode to TRACK
  if (!set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK)) {
    result->success = false;
    result->message = "failed or unconfirmed TRACK transition; " + contain_takeoff_breach();
    if (cancel_requested_) { goal_handle->canceled(result); }
    else { goal_handle->abort(result); }
    task_active_ = false;
    return;
  }

  if (!fresh_authority()) {
    result->success = false;
    result->message = "control authority lost before trajectory; " + contain_takeoff_breach();
    goal_handle->abort(result); task_active_ = false; return;
  }

  // generate and publish takeoff trajectory
  double takeoff_start_x;
  double takeoff_start_y;
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);

    // A tracking point can survive a simulator /clock reset in a long-lived
    // controller process. A takeoff must begin at the current physical pose;
    // otherwise that stale setpoint becomes an unintended horizontal command.
    airstack_msgs::msg::Odometry start_point;
    start_point.header = robot_odom_.header;
    start_point.pose.position.x = robot_odom_.pose.pose.position.x;
    start_point.pose.position.y = robot_odom_.pose.pose.position.y;
    start_point.pose.position.z = robot_odom_.pose.pose.position.z;
    start_point.pose.orientation = robot_odom_.pose.pose.orientation;
    takeoff_start_x = robot_odom_.pose.pose.position.x;
    takeoff_start_y = robot_odom_.pose.pose.position.y;

    // height is relative offset; target_altitude_m is absolute
    float current_z = robot_odom_.pose.pose.position.z;
    float relative_height = target_altitude - current_z;
    if (relative_height <= 0.0f) {
      RCLCPP_WARN(this->get_logger(),
        "TakeoffTask: target_altitude_m (%.2f) is not above current altitude (%.2f)",
        target_altitude, current_z);
      relative_height = 0.5f;  // minimum ascent
    }

    if (takeoff_path_relative_to_orientation_) {
      start_point.pose.orientation = robot_odom_.pose.pose.orientation;
    }

    TakeoffTrajectory traj_gen(
      relative_height, velocity,
      takeoff_path_roll_, takeoff_path_pitch_,
      takeoff_path_relative_to_orientation_);
    traj_override_pub_->publish(traj_gen.get_trajectory(start_point));
  }

  RCLCPP_INFO(this->get_logger(), "TakeoffTask: ascending to %.2fm at %.2f m/s",
    target_altitude, velocity);

  // monitor completion
  rclcpp::Rate rate(10);  // 10 Hz feedback
  rclcpp::Time acceptance_start;
  bool in_acceptance_window = false;

  while (rclcpp::ok()) {
    if (cancel_requested_) {
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      result->success = false;
      result->message = "canceled";
      goal_handle->canceled(result);
      task_active_ = false;
      return;
    }

    if (!fresh_authority()) {
      result->success = false;
      result->message = "control authority lost during ascent; " + contain_takeoff_breach();
      goal_handle->abort(result); task_active_ = false; return;
    }

    float current_x;
    float current_y;
    float current_z;
    float current_vertical_speed;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      current_x = robot_odom_.pose.pose.position.x;
      current_y = robot_odom_.pose.pose.position.y;
      current_z = robot_odom_.pose.pose.position.z;
      current_vertical_speed = robot_odom_.twist.twist.linear.z;
    }

    const float dist = std::abs(current_z - target_altitude);
    feedback->current_altitude_m = current_z;
    feedback->status =
      dist <= takeoff_acceptance_distance_ ? "stabilizing_at_target" : "ascending";
    goal_handle->publish_feedback(feedback);

    const double horizontal_displacement =
      std::hypot(current_x - takeoff_start_x, current_y - takeoff_start_y);
    if (takeoff_max_horizontal_displacement_ > 0.0 &&
      horizontal_displacement > takeoff_max_horizontal_displacement_)
    {
      RCLCPP_ERROR(this->get_logger(),
        "TakeoffTask aborted: horizontal displacement %.2fm exceeds %.2fm",
        horizontal_displacement, takeoff_max_horizontal_displacement_);
      result->success = false;
      result->message = "horizontal displacement limit exceeded" + contain_takeoff_breach();
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    if (takeoff_max_altitude_overshoot_ > 0.0 &&
      current_z > target_altitude + takeoff_max_altitude_overshoot_)
    {
      RCLCPP_ERROR(this->get_logger(),
        "TakeoffTask aborted: altitude %.2fm exceeds target %.2fm plus %.2fm overshoot limit",
        current_z, target_altitude, takeoff_max_altitude_overshoot_);
      result->success = false;
      result->message = "altitude overshoot limit exceeded" + contain_takeoff_breach();
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    if (takeoff_max_vertical_speed_ > 0.0 &&
      current_vertical_speed > takeoff_max_vertical_speed_)
    {
      RCLCPP_ERROR(this->get_logger(),
        "TakeoffTask aborted: vertical speed %.2fm/s exceeds %.2fm/s",
        current_vertical_speed, takeoff_max_vertical_speed_);
      result->success = false;
      result->message = "vertical speed limit exceeded" + contain_takeoff_breach();
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    // check completion: within acceptance distance of target for acceptance_time
    if (dist <= takeoff_acceptance_distance_) {
      if (!in_acceptance_window) {
        in_acceptance_window = true;
        acceptance_start = this->now();
      }
      if ((this->now() - acceptance_start).seconds() >= takeoff_acceptance_time_) {
        RCLCPP_INFO(this->get_logger(), "TakeoffTask: complete at %.2fm", current_z);
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
        result->success = true;
        result->message = "takeoff complete";
        goal_handle->succeed(result);
        task_active_ = false;
        return;
      }
    } else {
      in_acceptance_window = false;
    }

    rate.sleep();
  }

  set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
  result->success = false;
  result->message = "node shutting down";
  goal_handle->abort(result);
  task_active_ = false;
}

// ─────────────────────────── LandTask ─────────────────────────────────────────

rclcpp_action::GoalResponse TakeoffLandingTaskNode::land_handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const LandTask::Goal>)
{
  if (task_active_) {
    RCLCPP_WARN(this->get_logger(), "LandTask rejected: another task is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  task_active_ = true;
  cancel_requested_ = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TakeoffLandingTaskNode::land_handle_cancel(
  std::shared_ptr<LandGoalHandle>)
{
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TakeoffLandingTaskNode::land_handle_accepted(std::shared_ptr<LandGoalHandle> goal_handle)
{
  std::thread{
    std::bind(&TakeoffLandingTaskNode::land_execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void TakeoffLandingTaskNode::land_execute(std::shared_ptr<LandGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  float velocity = (goal->velocity_m_s > 0.0f) ? goal->velocity_m_s : default_landing_velocity_;

  auto result = std::make_shared<LandTask::Result>();
  auto feedback = std::make_shared<LandTask::Feedback>();

  // wait for odometry
  rclcpp::Rate wait_rate(10);
  int wait_count = 0;
  while (!got_robot_odom_ && rclcpp::ok()) {
    if (wait_count++ > 50) {
      RCLCPP_ERROR(this->get_logger(), "LandTask aborted: no odometry received");
      result->success = false;
      result->message = "no odometry received";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }
    wait_rate.sleep();
  }

  const bool observing_abort_land = abort_land_handover_.load();
  if (!observing_abort_land) {
  // Stop following any prior trajectory before constructing the landing path. Recovery
  // must never inherit the takeoff tracking point that preceded an abort.
  if (!set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE)) {
    result->success = false;
    result->message = "failed to set landing hold mode";
    goal_handle->abort(result);
    task_active_ = false;
    return;
  }

  // set trajectory mode to TRACK
  if (!set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK)) {
    result->success = false;
    result->message = "failed to set trajectory mode";
    goal_handle->abort(result);
    task_active_ = false;
    return;
  }

  // generate and publish landing trajectory
  {
    // A recovery landing starts at measured vehicle state, not a controller tracking
    // point that may still represent the aborted ascent.
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    airstack_msgs::msg::Odometry start_point;
    start_point.header = robot_odom_.header;
    start_point.pose.position.x = robot_odom_.pose.pose.position.x;
    start_point.pose.position.y = robot_odom_.pose.pose.position.y;
    start_point.pose.position.z = robot_odom_.pose.pose.position.z;
    start_point.pose.orientation = robot_odom_.pose.pose.orientation;
    // Descend to just below the current altitude plus a small margin. The prior
    // value of -10000.0 sent the tracking point to z=-9999, which only worked
    // because PX4 auto-disarms on ground contact.  A bounded value keeps the
    // trajectory controller from commanding an unbounded descent.
    const double landing_descent = -(start_point.pose.position.z + 1.0);
    TakeoffTrajectory land_traj(landing_descent, velocity);
    traj_override_pub_->publish(land_traj.get_trajectory(start_point));
  }
  }  // Accepted/uncertain abort LAND is observation-only; no conflicting trajectory.

  if (observing_abort_land) {
    RCLCPP_WARN(this->get_logger(), "LandTask: observing prior autopilot LAND request; grounding unverified");
  } else {
    RCLCPP_INFO(this->get_logger(), "LandTask: descending at %.2f m/s", velocity);
  }

  rclcpp::Rate rate(10);
  const auto landing_started = std::chrono::steady_clock::now();
  auto last_descent_progress = landing_started;
  float progress_altitude;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    progress_altitude = robot_odom_.pose.pose.position.z;
  }
  bool px4_land_requested = observing_abort_land;
  while (rclcpp::ok()) {
    if (cancel_requested_) {
      if (!observing_abort_land) {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      }
      result->success = false;
      result->message = "canceled";
      goal_handle->canceled(result);
      task_active_ = false;
      return;
    }

    float current_z;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      current_z = robot_odom_.pose.pose.position.z;
    }

    feedback->current_altitude_m = current_z;
    feedback->status = observing_abort_land ? "observing_autopilot_handover" : "landing";
    goal_handle->publish_feedback(feedback);

    // check if mavros reports on-ground
    if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
      RCLCPP_INFO(this->get_logger(), "LandTask: landed (mavros landed_state = ON_GROUND) at %.2fm",
        current_z);
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      send_robot_command(airstack_msgs::srv::RobotCommand::Request::DISARM);
      landed_ = true;
      std_msgs::msg::Bool airborne_msg;
      airborne_msg.data = false;
      is_airborne_pub_->publish(airborne_msg);
      result->success = true;
      result->message = "landing complete";
      goal_handle->succeed(result);
      task_active_ = false;
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (current_z <= progress_altitude - 0.05f) {
      progress_altitude = current_z;
      last_descent_progress = now;
    }
    // A controller can reach the floor while PX4's landed flag stays IN_AIR.
    // A sustained lack of descent is a better trigger than a map-Z threshold:
    // it also works when another scene's floor is not at z=0.
    if (!px4_land_requested && landing_stall_timeout_s_ > 0.0 &&
      std::chrono::duration<double>(now - last_descent_progress).count() >=
      landing_stall_timeout_s_)
    {
      px4_land_requested = true;
      RCLCPP_WARN(this->get_logger(),
        "LandTask descent stalled at %.2fm; requesting autopilot LAND mode", current_z);
      if (send_robot_command(airstack_msgs::srv::RobotCommand::Request::LAND)) {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "LandTask autopilot LAND mode request failed; retaining descent trajectory");
      }
    }
    if (landing_max_duration_s_ > 0.0 &&
      std::chrono::duration<double>(now - landing_started).count() >= landing_max_duration_s_)
    {
      RCLCPP_ERROR(this->get_logger(), "LandTask timed out without confirmed ground state");
      if (!observing_abort_land) {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      }
      result->success = false;
      result->message = "landing confirmation timed out";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    rate.sleep();
  }

  if (!observing_abort_land) {
    set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
  }
  result->success = false;
  result->message = "node shutting down";
  goal_handle->abort(result);
  task_active_ = false;
}

// ─────────────────────────── main ─────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffLandingTaskNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
