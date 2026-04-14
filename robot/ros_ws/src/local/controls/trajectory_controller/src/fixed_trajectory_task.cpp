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
//
// Ports fixed_trajectory_generator.py to a ROS 2 action server.

#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <airstack_msgs/msg/fixed_trajectory.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/msg/waypoint_xyzv_yaw.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>

#include "task_msgs/action/fixed_trajectory_task.hpp"

using WaypointXYZVYaw = airstack_msgs::msg::WaypointXYZVYaw;
using TrajectoryXYZVYaw = airstack_msgs::msg::TrajectoryXYZVYaw;

// ─────────────────────────── trajectory generation helpers ───────────────────

static void get_velocities(TrajectoryXYZVYaw & traj, double velocity, double max_acc)
{
  auto & wps = traj.waypoints;
  double v_prev = 0.0;
  for (size_t i = 0; i < wps.size(); ++i) {
    size_t j = (i + 1) % wps.size();
    double dx = wps[j].position.x - wps[i].position.x;
    double dy = wps[j].position.y - wps[i].position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double v_limit = std::sqrt(v_prev * v_prev + 2.0 * max_acc * dist);
    wps[i].velocity = std::min(velocity, v_limit);
    v_prev = wps[i].velocity;
  }
}

static void get_velocities_dual(TrajectoryXYZVYaw & traj, double velocity, double max_acc)
{
  auto & wps = traj.waypoints;
  double v_prev = 0.0;
  for (size_t i = 0; i < wps.size(); ++i) {
    size_t j = (i + 1) % wps.size();
    double dx = wps[j].position.x - wps[i].position.x;
    double dy = wps[j].position.y - wps[i].position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double v_limit = std::sqrt(v_prev * v_prev + 2.0 * max_acc * dist);
    wps[i].velocity = std::min(velocity, v_limit);
    v_prev = wps[i].velocity;
  }

  v_prev = 0.0;
  size_t tail_start = static_cast<size_t>(wps.size() * 0.85);
  for (size_t i = wps.size() - 1; i > tail_start; --i) {
    size_t j = (i - 1 + wps.size()) % wps.size();
    double dx = wps[j].position.x - wps[i].position.x;
    double dy = wps[j].position.y - wps[i].position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double v_limit = std::sqrt(v_prev * v_prev + 2.0 * max_acc * dist);
    wps[i].velocity = std::min(velocity, v_limit);
    v_prev = wps[i].velocity;
  }
  wps.back().velocity = 0.0;
}

static void get_accelerations(TrajectoryXYZVYaw & traj)
{
  auto & wps = traj.waypoints;
  if (wps.size() < 3) {return;}

  for (size_t i = 0; i + 2 < wps.size(); ++i) {
    size_t j = i + 1;
    size_t k = i + 2;

    double dx1 = wps[j].position.x - wps[i].position.x;
    double dy1 = wps[j].position.y - wps[i].position.y;
    double dist1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    if (dist1 < 1e-9) {
      wps[i].acceleration.x = 0.0;
      wps[i].acceleration.y = 0.0;
      wps[i].acceleration.z = 0.0;
      continue;
    }

    double v_cx = wps[i].velocity * dx1 / dist1;
    double v_cy = wps[i].velocity * dy1 / dist1;

    double dx2 = wps[k].position.x - wps[j].position.x;
    double dy2 = wps[k].position.y - wps[j].position.y;
    double dist2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    if (dist2 < 1e-9) {
      wps[i].acceleration.x = 0.0;
      wps[i].acceleration.y = 0.0;
      wps[i].acceleration.z = 0.0;
      continue;
    }

    double v_nx = wps[j].velocity * dx2 / dist2;
    double v_ny = wps[j].velocity * dy2 / dist2;

    constexpr double acc_limit = 50.0;
    wps[i].acceleration.x = (std::abs(dx1) < 1e-6)
      ? 0.0 : std::min((v_nx * v_nx - v_cx * v_cx) / (2.0 * dx1), acc_limit);
    wps[i].acceleration.y = (std::abs(dy1) < 1e-6)
      ? 0.0 : std::min((v_ny * v_ny - v_cy * v_cy) / (2.0 * dy1), acc_limit);
    wps[i].acceleration.z = 0.0;
  }

  // zero out last two
  if (wps.size() >= 2) {
    wps[wps.size() - 2].acceleration.x = 0.0;
    wps[wps.size() - 2].acceleration.y = 0.0;
    wps[wps.size() - 2].acceleration.z = 0.0;
  }
  wps.back().acceleration.x = 0.0;
  wps.back().acceleration.y = 0.0;
  wps.back().acceleration.z = 0.0;
}

static WaypointXYZVYaw make_wp(double x, double y, double z, double yaw, double v = 0.0)
{
  WaypointXYZVYaw wp;
  wp.position.x = x;
  wp.position.y = y;
  wp.position.z = z;
  wp.yaw = yaw;
  wp.velocity = v;
  return wp;
}

static TrajectoryXYZVYaw generate_circle(const std::map<std::string, std::string> & attrs)
{
  std::string frame_id = attrs.count("frame_id") ? attrs.at("frame_id") : "base_link";
  double radius = std::stod(attrs.at("radius"));
  double velocity = std::stod(attrs.at("velocity"));

  TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame_id;

  traj.waypoints.push_back(make_wp(0.0, 0.0, 0.0, 0.0, velocity));
  traj.waypoints.push_back(make_wp(radius, 0.0, 0.0, 0.0, velocity));

  for (double angle = 0.0; angle < 2.0 * M_PI; angle += 10.0 * M_PI / 180.0) {
    traj.waypoints.push_back(
      make_wp(radius * std::cos(angle), radius * std::sin(angle), 0.0, 0.0, velocity));
  }

  traj.waypoints.push_back(make_wp(radius, 0.0, 0.0, 0.0, velocity));
  traj.waypoints.push_back(make_wp(0.0, 0.0, 0.0, 0.0, velocity));
  return traj;
}

static TrajectoryXYZVYaw generate_figure8(const std::map<std::string, std::string> & attrs)
{
  std::string frame_id = attrs.count("frame_id") ? attrs.at("frame_id") : "base_link";
  double length = std::stod(attrs.at("length"));
  double width = std::stod(attrs.at("width"));
  double height = std::stod(attrs.at("height"));
  double velocity = std::stod(attrs.at("velocity"));
  double max_acc = std::stod(attrs.at("max_acceleration"));

  TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame_id;

  int n = 600;
  for (int i = 0; i < n - 1; ++i) {
    double t = 2.0 * M_PI * i / n;
    double x = std::cos(t) * length - length;
    double y = std::cos(t) * std::sin(t) * 2.0 * width;
    double xd = -std::sin(t) * length;
    double yd = (std::cos(t) * std::cos(t) - std::sin(t) * std::sin(t)) * 2.0 * width;
    double yaw = std::atan2(yd, xd);
    traj.waypoints.push_back(make_wp(x, y, height, yaw));
  }

  get_velocities_dual(traj, velocity, max_acc);
  get_accelerations(traj);
  return traj;
}

static TrajectoryXYZVYaw generate_racetrack(const std::map<std::string, std::string> & attrs)
{
  std::string frame_id = attrs.count("frame_id") ? attrs.at("frame_id") : "base_link";
  double length = std::stod(attrs.at("length"));
  double width = std::stod(attrs.at("width"));
  double height = std::stod(attrs.at("height"));
  double velocity = std::stod(attrs.at("velocity"));
  double turn_velocity = std::stod(attrs.at("turn_velocity"));
  double max_acc = std::stod(attrs.at("max_acceleration"));

  TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame_id;

  double sl = length - width;  // straightaway length

  // first straightaway
  for (int i = 0; i <= 79; ++i) {
    double x = sl * i / 79.0;
    traj.waypoints.push_back(make_wp(x, 0.0, height, 0.0, velocity));
  }

  // first turn: semi-circle at (sl, width/2), from -pi/2 to pi/2
  int turn_n = 48;  // 50 - 2 endpoints already covered
  for (int i = 1; i < turn_n + 1; ++i) {
    double t = -M_PI / 2.0 + M_PI * i / (turn_n + 1);
    double x = width / 2.0 * std::cos(t) + sl;
    double y = width / 2.0 * std::sin(t) + width / 2.0;
    double xd = -width / 2.0 * std::sin(t);
    double yd = width / 2.0 * std::cos(t);
    double yaw = std::atan2(yd, xd);
    traj.waypoints.push_back(make_wp(x, y, height, yaw, turn_velocity));
  }

  // second straightaway
  for (int i = 0; i <= 79; ++i) {
    double x = sl * (1.0 - i / 79.0);
    traj.waypoints.push_back(make_wp(x, width, height, M_PI, velocity));
  }

  // second turn: semi-circle at (0, width/2), from pi/2 to 3pi/2
  for (int i = 1; i < turn_n + 1; ++i) {
    double t = M_PI / 2.0 + M_PI * i / (turn_n + 1);
    double x = width / 2.0 * std::cos(t);
    double y = width / 2.0 * std::sin(t) + width / 2.0;
    double xd = -width / 2.0 * std::sin(t);
    double yd = width / 2.0 * std::cos(t);
    double yaw = std::atan2(yd, xd) + M_PI;
    traj.waypoints.push_back(make_wp(x, y, height, yaw, turn_velocity));
  }

  if (!traj.waypoints.empty()) {
    traj.waypoints.front().velocity = 0.0;
    traj.waypoints.back().velocity = 0.0;
  }

  get_accelerations(traj);
  (void)max_acc;
  return traj;
}

static TrajectoryXYZVYaw generate_line(const std::map<std::string, std::string> & attrs)
{
  std::string frame_id = attrs.count("frame_id") ? attrs.at("frame_id") : "base_link";
  double length = std::stod(attrs.at("length"));
  double height = std::stod(attrs.at("height"));
  double velocity = std::stod(attrs.at("velocity"));
  double max_acc = std::stod(attrs.at("max_acceleration"));

  TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame_id;

  for (double y = 0.0; y > -length; y -= 0.5) {
    traj.waypoints.push_back(make_wp(-y, 0.0, height, 0.0));
  }

  get_velocities(traj, velocity, max_acc);
  return traj;
}

static TrajectoryXYZVYaw generate_point(const std::map<std::string, std::string> & attrs)
{
  std::string frame_id = attrs.count("frame_id") ? attrs.at("frame_id") : "base_link";
  double x = std::stod(attrs.at("x"));
  double y = std::stod(attrs.at("y"));
  double height = std::stod(attrs.at("height"));
  double velocity = std::stod(attrs.at("velocity"));
  double max_acc = std::stod(attrs.at("max_acceleration"));

  TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame_id;
  traj.waypoints.push_back(make_wp(x, y, height, 0.0));

  get_velocities(traj, velocity, max_acc);
  return traj;
}

static TrajectoryXYZVYaw generate_lawnmower(const std::map<std::string, std::string> & attrs)
{
  std::string frame_id = attrs.count("frame_id") ? attrs.at("frame_id") : "base_link";
  double length = std::stod(attrs.at("length"));
  double width = std::stod(attrs.at("width"));
  double height = std::stod(attrs.at("height"));
  double velocity = std::stod(attrs.at("velocity"));
  bool vertical = (attrs.count("vertical") && std::stoi(attrs.at("vertical")) != 0);

  TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame_id;

  int strips = static_cast<int>(std::abs(height / width));
  double h_sign = (height >= 0) ? 1.0 : -1.0;

  for (int i = 0; i < strips; ++i) {
    double z = h_sign * (i + 1) * width;
    double yaw = std::atan2(length, 0.0);  // pointing along y

    if (i % 2 == 0) {
      // y goes 0 → length
      traj.waypoints.push_back(make_wp(0.0, 0.0, z, vertical ? 0.0 : yaw, 0.1));
      traj.waypoints.push_back(make_wp(0.0, 0.5, z, vertical ? 0.0 : yaw, 0.1));
      traj.waypoints.push_back(make_wp(0.0, 0.5, z, vertical ? 0.0 : yaw, velocity));
      traj.waypoints.push_back(make_wp(0.0, length - 0.5, z, vertical ? 0.0 : yaw, velocity));
      traj.waypoints.push_back(make_wp(0.0, length, z, vertical ? 0.0 : yaw, 0.1));
    } else {
      // y goes length → 0
      traj.waypoints.push_back(make_wp(0.0, length, z, vertical ? 0.0 : -yaw, 0.1));
      traj.waypoints.push_back(make_wp(0.0, length - 0.5, z, vertical ? 0.0 : -yaw, 0.1));
      traj.waypoints.push_back(make_wp(0.0, length - 0.5, z, vertical ? 0.0 : -yaw, velocity));
      traj.waypoints.push_back(make_wp(0.0, 0.5, z, vertical ? 0.0 : -yaw, velocity));
      traj.waypoints.push_back(make_wp(0.0, 0.0, z, vertical ? 0.0 : -yaw, 0.1));
    }
  }

  // return waypoint
  traj.waypoints.push_back(make_wp(0.0, 0.0, 0.0, 0.0, 0.5));

  // if not vertical, swap x and z
  if (!vertical) {
    for (auto & wp : traj.waypoints) {
      std::swap(wp.position.x, wp.position.z);
    }
  }

  return traj;
}

// ─────────────────────────── action server node ───────────────────────────────

class FixedTrajectoryTaskNode : public rclcpp::Node
{
public:
  using FixedTrajectoryTask = task_msgs::action::FixedTrajectoryTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FixedTrajectoryTask>;

  FixedTrajectoryTaskNode()
  : rclcpp::Node("fixed_trajectory_task")
  {
    completion_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "trajectory_completion_percentage", 1,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(completion_mutex_);
        completion_percentage_ = msg->data;
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(1).best_effort(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_odom_ = *msg;
      });

    traj_override_pub_ =
      this->create_publisher<TrajectoryXYZVYaw>("trajectory_override", 1);

    traj_mode_client_ =
      this->create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode");

    action_server_ = rclcpp_action::create_server<FixedTrajectoryTask>(
      this, "~/fixed_trajectory_task",
      std::bind(&FixedTrajectoryTaskNode::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&FixedTrajectoryTaskNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FixedTrajectoryTaskNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "FixedTrajectoryTaskNode started");
  }

private:
  std::atomic<bool> task_active_{false};
  std::atomic<bool> cancel_requested_{false};

  std::mutex completion_mutex_;
  float completion_percentage_{0.0f};

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry current_odom_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr completion_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<TrajectoryXYZVYaw>::SharedPtr traj_override_pub_;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_client_;
  rclcpp_action::Server<FixedTrajectoryTask>::SharedPtr action_server_;

  bool set_trajectory_mode(int32_t mode)
  {
    if (!traj_mode_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "set_trajectory_mode service not available");
      return false;
    }
    auto req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    req->mode = mode;
    auto future = traj_mode_client_->async_send_request(req);
    future.wait();
    return future.get()->success;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FixedTrajectoryTask::Goal> goal)
  {
    if (task_active_) {
      RCLCPP_WARN(this->get_logger(), "FixedTrajectoryTask rejected: task already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // validate trajectory type
    const std::string & type = goal->trajectory_spec.type;
    if (type != "Figure8" && type != "Circle" && type != "Racetrack" &&
      type != "Line" && type != "Point" && type != "Lawnmower")
    {
      RCLCPP_WARN(this->get_logger(),
        "FixedTrajectoryTask rejected: unknown trajectory type '%s'", type.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    task_active_ = true;
    cancel_requested_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>)
  {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{
      std::bind(&FixedTrajectoryTaskNode::execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute(std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FixedTrajectoryTask::Result>();
    auto feedback = std::make_shared<FixedTrajectoryTask::Feedback>();

    // parse attributes into a map
    std::map<std::string, std::string> attrs;
    for (const auto & kv : goal->trajectory_spec.attributes) {
      attrs[kv.key] = kv.value;
    }

    // generate trajectory
    TrajectoryXYZVYaw traj;
    const std::string & type = goal->trajectory_spec.type;
    try {
      if (type == "Circle") {
        traj = generate_circle(attrs);
      } else if (type == "Figure8") {
        traj = generate_figure8(attrs);
      } else if (type == "Racetrack") {
        traj = generate_racetrack(attrs);
      } else if (type == "Line") {
        traj = generate_line(attrs);
      } else if (type == "Point") {
        traj = generate_point(attrs);
      } else if (type == "Lawnmower") {
        traj = generate_lawnmower(attrs);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "FixedTrajectoryTask: trajectory generation failed: %s",
        e.what());
      result->success = false;
      result->message = std::string("trajectory generation failed: ") + e.what();
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    if (traj.waypoints.empty()) {
      result->success = false;
      result->message = "trajectory generation produced no waypoints";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    traj.header.stamp = this->now();

    // set trajectory mode to TRACK and publish
    if (!set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK)) {
      result->success = false;
      result->message = "failed to set trajectory mode";
      goal_handle->abort(result);
      task_active_ = false;
      return;
    }

    traj_override_pub_->publish(traj);
    RCLCPP_INFO(this->get_logger(), "FixedTrajectoryTask: executing %s (%zu waypoints)",
      type.c_str(), traj.waypoints.size());

    // reset completion percentage after publishing
    {
      std::lock_guard<std::mutex> lock(completion_mutex_);
      completion_percentage_ = 0.0f;
    }

    // monitor completion
    rclcpp::Rate rate(5);
    while (rclcpp::ok()) {
      if (cancel_requested_) {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
        result->success = false;
        result->message = "cancelled";
        goal_handle->canceled(result);
        task_active_ = false;
        return;
      }

      float completion;
      {
        std::lock_guard<std::mutex> lock(completion_mutex_);
        completion = completion_percentage_;
      }

      nav_msgs::msg::Odometry odom;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom = current_odom_;
      }

      feedback->progress = completion / 100.0f;
      feedback->status = type + " trajectory running";
      feedback->current_position.x = odom.pose.pose.position.x;
      feedback->current_position.y = odom.pose.pose.position.y;
      feedback->current_position.z = odom.pose.pose.position.z;
      goal_handle->publish_feedback(feedback);

      if (completion >= 100.0f) {
        if (goal->loop) {
          // republish and reset
          traj.header.stamp = this->now();
          traj_override_pub_->publish(traj);
          {
            std::lock_guard<std::mutex> lock(completion_mutex_);
            completion_percentage_ = 0.0f;
          }
          RCLCPP_INFO(this->get_logger(), "FixedTrajectoryTask: looping %s", type.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "FixedTrajectoryTask: %s complete", type.c_str());
          set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
          result->success = true;
          result->message = type + " trajectory complete";
          goal_handle->succeed(result);
          task_active_ = false;
          return;
        }
      }

      rate.sleep();
    }

    set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
    result->success = false;
    result->message = "node shutting down";
    goal_handle->abort(result);
    task_active_ = false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FixedTrajectoryTaskNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
