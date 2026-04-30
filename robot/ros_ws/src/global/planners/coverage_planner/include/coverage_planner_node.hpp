// Copyright (c) 2026 Carnegie Mellon University
//
// Licensed under the Apache License, Version 2.0 (the "License").
// See coverage_planner_logic.hpp for full license text.

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "airstack_msgs/msg/odometry.hpp"
#include "airstack_msgs/msg/trajectory_xyzv_yaw.hpp"
#include "airstack_msgs/srv/trajectory_mode.hpp"
#include "coverage_planner_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_msgs/action/coverage_task.hpp"
#include "task_msgs/action/navigate_task.hpp"

namespace coverage_planner {

class CoveragePlannerNode : public rclcpp::Node {
public:
  using CoverageTask = task_msgs::action::CoverageTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CoverageTask>;
  using NavigateTask = task_msgs::action::NavigateTask;

  CoveragePlannerNode();
  ~CoveragePlannerNode() override = default;

private:
  // ---- Subscriber callbacks -----------------------------------------
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ---- Action server callbacks --------------------------------------
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const CoverageTask::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void execute(std::shared_ptr<GoalHandle> goal_handle);

  // ---- Planning helpers ---------------------------------------------
  std::optional<std::vector<Waypoint>>
  plan_coverage(const CoverageTask::Goal &goal) const;
  nav_msgs::msg::Path build_ros_path(const std::vector<Waypoint> &waypoints,
                                     const rclcpp::Time &stamp) const;
  void publish_visualization(const nav_msgs::msg::Path &path);
  void send_navigate_goal(const nav_msgs::msg::Path &path,
                          double goal_tolerance_m);
  void send_trajectory_override(const std::vector<Waypoint> &waypoints,
                                double velocity);
  bool set_trajectory_mode(int32_t mode);
  void
  tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg);

  // ---- Configuration parameters -------------------------------------
  std::string world_frame_id_;
  std::string pub_global_plan_topic_;
  std::string pub_path_viz_topic_;
  std::string pub_coverage_area_viz_topic_;
  std::string sub_odometry_topic_;

  double default_altitude_m_{5.0};
  double default_line_spacing_m_{5.0};
  double default_boundary_inset_m_{0.0};
  double waypoint_tolerance_m_{2.0};
  bool publish_visualizations_{true};

  // ---- Runtime state ------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_path_viz_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pub_coverage_area_viz_;

  rclcpp_action::Server<CoverageTask>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateTask>::SharedPtr navigate_client_;
  rclcpp_action::ClientGoalHandle<NavigateTask>::SharedPtr
      navigate_goal_handle_;
  std::atomic<bool> navigate_goal_done_{true};
  std::atomic<bool> navigate_goal_succeeded_{false};

  // Direct trajectory override (bypasses local planner)
  rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr
      pub_traj_override_;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr
      traj_mode_client_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr
      sub_tracking_point_;
  airstack_msgs::msg::Odometry tracking_point_odom_;
  std::atomic<bool> got_tracking_point_{false};
  bool direct_mode_{false};

  geometry_msgs::msg::Pose current_pose_;
  bool received_odometry_{false};

  std::atomic<bool> task_active_{false};
  std::atomic<bool> cancel_requested_{false};
  rclcpp::Time task_start_time_;
  double polygon_area_m2_{0.0};
  std::size_t total_waypoints_{0};
};

} // namespace coverage_planner
