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
#include <thread>

TakeoffLandingTaskNode::TakeoffLandingTaskNode()
: rclcpp::Node("takeoff_landing_task")
{
  // parameters
  default_takeoff_velocity_ = airstack::get_param(this, "takeoff_velocity", 1.0);
  default_landing_velocity_ = airstack::get_param(this, "landing_velocity", 0.3);
  takeoff_acceptance_distance_ = airstack::get_param(this, "takeoff_acceptance_distance", 0.3);
  takeoff_acceptance_time_ = airstack::get_param(this, "takeoff_acceptance_time", 1.0);
  landing_stationary_distance_ = airstack::get_param(this, "landing_stationary_distance", 0.02);
  landing_acceptance_time_ = airstack::get_param(this, "landing_acceptance_time", 5.0);
  landing_tracking_point_ahead_time_ =
    airstack::get_param(this, "landing_tracking_point_ahead_time", 5.0);
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
    [this](std_msgs::msg::Bool::SharedPtr msg) { is_armed_ = msg->data; });

  has_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "has_control", 1,
    [this](std_msgs::msg::Bool::SharedPtr msg) { has_control_ = msg->data; });

  state_estimate_timed_out_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "state_estimate_timed_out", 1,
    [this](std_msgs::msg::Bool::SharedPtr msg) { state_estimate_timed_out_ = msg->data; });

  extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "extended_state", 1,
    [this](mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      landed_state_ = msg->landed_state;
    });

  // publishers
  traj_override_pub_ =
    this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_override", 1);

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
  future.wait();
  return future.get()->success;
}

bool TakeoffLandingTaskNode::send_robot_command(uint8_t command)
{
  if (!robot_command_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "robot_command service not available");
    return false;
  }
  auto request = std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
  request->command = command;
  auto future = robot_command_client_->async_send_request(request);
  future.wait();
  return future.get()->success;
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

  // arm the robot
  if (!is_armed_) {
    RCLCPP_INFO(this->get_logger(), "TakeoffTask: arming robot");
    if (!send_robot_command(airstack_msgs::srv::RobotCommand::Request::ARM)) {
      RCLCPP_ERROR(this->get_logger(), "TakeoffTask aborted: failed to arm");
      result->success = false;
      result->message = "failed to arm";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }
  }

  // request offboard control
  if (!has_control_) {
    RCLCPP_INFO(this->get_logger(), "TakeoffTask: requesting offboard control");
    if (!send_robot_command(airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL)) {
      RCLCPP_ERROR(this->get_logger(), "TakeoffTask aborted: failed to request offboard control");
      result->success = false;
      result->message = "failed to request offboard control";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }
  }

  // set trajectory mode to TRACK
  if (!set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK)) {
    result->success = false;
    result->message = "failed to set trajectory mode";
    goal_handle->abort(result);
    task_active_ = false;
    return;
  }

  // generate and publish takeoff trajectory
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    std::lock_guard<std::mutex> tp_lock(tracking_point_mutex_);

    airstack_msgs::msg::Odometry start_point;
    if (got_tracking_point_) {
      start_point = tracking_point_odom_;
    } else {
      // fall back to current robot odom as start point
      start_point.header = robot_odom_.header;
      start_point.pose.position.x = robot_odom_.pose.pose.position.x;
      start_point.pose.position.y = robot_odom_.pose.pose.position.y;
      start_point.pose.position.z = robot_odom_.pose.pose.position.z;
      start_point.pose.orientation = robot_odom_.pose.pose.orientation;
    }

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
      result->message = "cancelled";
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
    goal_handle->publish_feedback(feedback);

    // check completion: within acceptance distance of target for acceptance_time
    float dist = std::abs(current_z - target_altitude);
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
    std::lock_guard<std::mutex> tp_lock(tracking_point_mutex_);
    airstack_msgs::msg::Odometry start_point;
    if (got_tracking_point_) {
      start_point = tracking_point_odom_;
    } else {
      std::lock_guard<std::mutex> odom_lock(odom_mutex_);
      start_point.header = robot_odom_.header;
      start_point.pose.position.x = robot_odom_.pose.pose.position.x;
      start_point.pose.position.y = robot_odom_.pose.pose.position.y;
      start_point.pose.position.z = robot_odom_.pose.pose.position.z;
      start_point.pose.orientation = robot_odom_.pose.pose.orientation;
    }
    TakeoffTrajectory land_traj(-10000.0, velocity);
    traj_override_pub_->publish(land_traj.get_trajectory(start_point));
  }

  RCLCPP_INFO(this->get_logger(), "LandTask: descending at %.2f m/s", velocity);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    if (cancel_requested_) {
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      result->success = false;
      result->message = "cancelled";
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
    feedback->status = "landing";
    goal_handle->publish_feedback(feedback);

    // check if mavros reports on-ground
    if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
      RCLCPP_INFO(this->get_logger(), "LandTask: landed (mavros landed_state = ON_GROUND) at %.2fm",
        current_z);
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
      result->success = true;
      result->message = "landing complete";
      goal_handle->succeed(result);
      task_active_ = false;
      return;
    }

    rate.sleep();
  }

  set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
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
