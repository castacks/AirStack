// Copyright (c) 2026 Carnegie Mellon University
//
// Licensed under the Apache License, Version 2.0 (the "License").
// See coverage_planner_logic.hpp for full license text.

#include "coverage_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>

namespace coverage_planner {

using std::placeholders::_1;
using std::placeholders::_2;

CoveragePlannerNode::CoveragePlannerNode() : rclcpp::Node("coverage_planner_node") {
    // ---- Topic / frame parameters -------------------------------------
    world_frame_id_ = this->declare_parameter<std::string>("world_frame_id", "map");
    pub_global_plan_topic_ =
        this->declare_parameter<std::string>("pub_global_plan_topic", "~/global_plan");
    pub_path_viz_topic_ =
        this->declare_parameter<std::string>("pub_path_viz_topic", "~/coverage_path_viz");
    pub_coverage_area_viz_topic_ = this->declare_parameter<std::string>(
        "pub_coverage_area_viz_topic", "~/coverage_area_viz");
    sub_odometry_topic_ = this->declare_parameter<std::string>("sub_odometry_topic", "odometry");

    // ---- Coverage defaults --------------------------------------------
    default_altitude_m_ = this->declare_parameter<double>("default_altitude_m", 5.0);
    default_line_spacing_m_ = this->declare_parameter<double>("default_line_spacing_m", 5.0);
    default_boundary_inset_m_ =
        this->declare_parameter<double>("default_boundary_inset_m", 0.0);
    waypoint_tolerance_m_ = this->declare_parameter<double>("waypoint_tolerance_m", 2.0);
    publish_visualizations_ =
        this->declare_parameter<bool>("publish_visualizations", true);

    // ---- Pub / sub ----------------------------------------------------
    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odometry_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CoveragePlannerNode::odometry_callback, this, _1));
    pub_global_plan_ =
        this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
    pub_path_viz_ =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_path_viz_topic_, 10);
    pub_coverage_area_viz_ = this->create_publisher<visualization_msgs::msg::Marker>(
        pub_coverage_area_viz_topic_, 10);

    // ---- Action server & client --------------------------------------
    action_server_ = rclcpp_action::create_server<CoverageTask>(
        this, "~/coverage_task",
        std::bind(&CoveragePlannerNode::handle_goal, this, _1, _2),
        std::bind(&CoveragePlannerNode::handle_cancel, this, _1),
        std::bind(&CoveragePlannerNode::handle_accepted, this, _1));

    navigate_client_ = rclcpp_action::create_client<NavigateTask>(this, "navigate_task");

    // ---- Direct trajectory override (bypasses local planner) ----------
    pub_traj_override_ =
        this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_override", 1);
    traj_mode_client_ =
        this->create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode");
    sub_tracking_point_ = this->create_subscription<airstack_msgs::msg::Odometry>(
        "tracking_point", 10,
        std::bind(&CoveragePlannerNode::tracking_point_callback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "coverage_planner initialized. Waiting for CoverageTask goals on %s",
                (std::string(this->get_fully_qualified_name()) + "/coverage_task").c_str());
}

// ---------------------------------------------------------------------------
// Subscriber callbacks
// ---------------------------------------------------------------------------

void CoveragePlannerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    if (!received_odometry_) {
        received_odometry_ = true;
        RCLCPP_INFO(this->get_logger(), "Received first odometry message");
    }
}

// ---------------------------------------------------------------------------
// Action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CoveragePlannerNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const CoverageTask::Goal> goal) {
    if (task_active_) {
        RCLCPP_WARN(this->get_logger(), "Rejecting CoverageTask goal: task already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->coverage_area.points.size() < 3) {
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting CoverageTask goal: polygon needs at least 3 vertices (got %zu)",
                    goal->coverage_area.points.size());
        return rclcpp_action::GoalResponse::REJECT;
    }
    const float line_spacing =
        goal->line_spacing_m > 0.0f ? goal->line_spacing_m
                                    : static_cast<float>(default_line_spacing_m_);
    if (line_spacing <= 0.0f) {
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting CoverageTask goal: line_spacing_m must be > 0");
        return rclcpp_action::GoalResponse::REJECT;
    }
    task_active_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Accepted CoverageTask: %zu-vertex polygon, spacing=%.2fm, "
                "heading=%.1fdeg, alt=[%.1f,%.1f]m AGL",
                goal->coverage_area.points.size(), line_spacing, goal->heading_deg,
                goal->min_altitude_agl, goal->max_altitude_agl);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoveragePlannerNode::handle_cancel(
    std::shared_ptr<GoalHandle> /*goal_handle*/) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request for CoverageTask");
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CoveragePlannerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&CoveragePlannerNode::execute, this, _1), goal_handle}.detach();
}

// ---------------------------------------------------------------------------
// Task execution (runs in its own thread)
// ---------------------------------------------------------------------------

void CoveragePlannerNode::execute(std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    task_start_time_ = this->now();
    cancel_requested_ = false;
    navigate_goal_done_ = true;
    navigate_goal_succeeded_ = false;

    auto finalize = [&](bool success, const std::string& msg, float coverage_pct) {
        if (navigate_goal_handle_ && !navigate_goal_done_) {
            navigate_client_->async_cancel_goal(navigate_goal_handle_);
        }
        auto result = std::make_shared<CoverageTask::Result>();
        result->success = success;
        result->message = msg;
        result->coverage_percentage = coverage_pct;
        task_active_ = false;
        if (success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->canceled(result);
        }
    };

    // Wait briefly for an odometry fix before planning.
    {
        rclcpp::Rate wait_rate(10.0);
        const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
        while (rclcpp::ok() && !received_odometry_ && this->now() < deadline) {
            if (cancel_requested_) {
                finalize(false, "Cancelled before planning", 0.0f);
                return;
            }
            wait_rate.sleep();
        }
    }
    if (!received_odometry_) {
        RCLCPP_WARN(this->get_logger(),
                    "No odometry received — planning from (0, 0) as start");
    }

    // ---- Plan the coverage path --------------------------------------
    const auto waypoints_opt = plan_coverage(*goal);
    if (!waypoints_opt || waypoints_opt->empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate coverage path");
        finalize(false, "Failed to generate coverage path", 0.0f);
        return;
    }
    const auto& waypoints = *waypoints_opt;
    total_waypoints_ = waypoints.size();

    const auto ros_path = build_ros_path(waypoints, this->now());
    pub_global_plan_->publish(ros_path);
    if (publish_visualizations_) publish_visualization(ros_path);

    const double path_len_m = path_length(waypoints);
    RCLCPP_INFO(this->get_logger(),
                "Generated coverage path: %zu waypoints, %.1fm total length",
                waypoints.size(), path_len_m);

    // ---- Try local planner first, fall back to direct mode -----------
    direct_mode_ = false;
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(),
                    "NavigateTask action server not available; using direct trajectory override");
        direct_mode_ = true;
    }

    if (direct_mode_) {
        // Bypass local planner: publish trajectory override directly
        const double velocity = 2.0;  // m/s cruise speed
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK);
        send_trajectory_override(waypoints, velocity);
    } else {
        send_navigate_goal(ros_path, waypoint_tolerance_m_);
    }

    // ---- Monitor loop (publish feedback ~1 Hz) -----------------------
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok()) {
        if (cancel_requested_) {
            if (direct_mode_) {
                set_trajectory_mode(
                    airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
            }
            finalize(false, "Task cancelled", 0.0f);
            return;
        }

        // Feedback -----------------------------------------------------
        auto feedback = std::make_shared<CoverageTask::Feedback>();
        feedback->current_position.x = current_pose_.position.x;
        feedback->current_position.y = current_pose_.position.y;
        feedback->current_position.z = current_pose_.position.z;

        // Rough progress: nearest waypoint index
        std::size_t nearest = 0;
        double nearest_d2 = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < waypoints.size(); ++i) {
            const double dx = waypoints[i].x - current_pose_.position.x;
            const double dy = waypoints[i].y - current_pose_.position.y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < nearest_d2) {
                nearest_d2 = d2;
                nearest = i;
            }
        }
        const float frac = waypoints.empty()
                               ? 0.0f
                               : static_cast<float>(nearest) /
                                     static_cast<float>(waypoints.size() - 1);
        feedback->progress = std::clamp(frac, 0.0f, 1.0f);
        feedback->coverage_percentage = feedback->progress * 100.0f;

        if (direct_mode_) {
            // In direct mode, check if tracking point is near the last waypoint
            feedback->status = "surveying (direct)";
            if (got_tracking_point_ && !waypoints.empty()) {
                const double dx = tracking_point_odom_.pose.position.x - waypoints.back().x;
                const double dy = tracking_point_odom_.pose.position.y - waypoints.back().y;
                const double dz = tracking_point_odom_.pose.position.z - waypoints.back().z;
                const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < waypoint_tolerance_m_) {
                    set_trajectory_mode(
                        airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
                    finalize(true, "Coverage complete (direct mode)", 100.0f);
                    RCLCPP_INFO(this->get_logger(), "CoverageTask succeeded (direct mode)");
                    return;
                }
            }
        } else {
            feedback->status = navigate_goal_done_ ? "complete" : "surveying";
            // Completion via NavigateTask
            if (navigate_goal_done_) {
                if (navigate_goal_succeeded_) {
                    finalize(true, "Coverage complete", 100.0f);
                    RCLCPP_INFO(this->get_logger(), "CoverageTask succeeded");
                } else {
                    finalize(false, "Local planner failed to complete coverage path",
                             feedback->coverage_percentage);
                    RCLCPP_WARN(this->get_logger(),
                                "CoverageTask aborted: NavigateTask did not succeed");
                }
                return;
            }
        }

        goal_handle->publish_feedback(feedback);
        rate.sleep();
    }

    // Node shutdown
    if (direct_mode_) {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
    }
    finalize(false, "Node shutting down", 0.0f);
}

// ---------------------------------------------------------------------------
// Planning helpers
// ---------------------------------------------------------------------------

std::optional<std::vector<Waypoint>> CoveragePlannerNode::plan_coverage(
    const CoverageTask::Goal& goal) const {
    Polygon2D polygon;
    polygon.reserve(goal.coverage_area.points.size());
    for (const auto& pt : goal.coverage_area.points) {
        polygon.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y)});
    }

    CoverageParams params;
    params.line_spacing_m = goal.line_spacing_m > 0.0f
                                ? goal.line_spacing_m
                                : default_line_spacing_m_;
    params.heading_deg = goal.heading_deg;
    params.boundary_inset_m = default_boundary_inset_m_;

    // Pick the midpoint of the requested altitude band. CoverageTask
    // specifies AGL; this planner currently treats the values as world-z.
    // Ground-relative conversion can be added once terrain elevation is
    // available here.
    const double lo = std::max(0.0, static_cast<double>(goal.min_altitude_agl));
    const double hi = std::max(lo, static_cast<double>(goal.max_altitude_agl));
    if (hi > 0.0) {
        params.altitude_m = 0.5 * (lo + hi);
    } else {
        params.altitude_m = default_altitude_m_;
    }

    Point2D start{current_pose_.position.x, current_pose_.position.y};
    return generate_coverage_path(polygon, start, params);
}

nav_msgs::msg::Path CoveragePlannerNode::build_ros_path(
    const std::vector<Waypoint>& waypoints, const rclcpp::Time& stamp) const {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = world_frame_id_;
    path.poses.reserve(waypoints.size());
    for (const auto& wp : waypoints) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = wp.x;
        ps.pose.position.y = wp.y;
        ps.pose.position.z = wp.z;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wp.yaw);
        q.normalize();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path.poses.push_back(ps);
    }
    return path;
}

void CoveragePlannerNode::publish_visualization(const nav_msgs::msg::Path& path) {
    visualization_msgs::msg::Marker line;
    line.header = path.header;
    line.ns = "coverage_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.15;
    line.color.a = 1.0f;
    line.color.r = 0.1f;
    line.color.g = 0.8f;
    line.color.b = 0.1f;
    line.pose.orientation.w = 1.0;
    line.points.reserve(path.poses.size());
    for (const auto& ps : path.poses) line.points.push_back(ps.pose.position);
    pub_path_viz_->publish(line);
}

void CoveragePlannerNode::send_navigate_goal(const nav_msgs::msg::Path& path,
                                              double goal_tolerance_m) {
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(),
                    "NavigateTask action server not available; publishing global_plan only");
        return;
    }

    NavigateTask::Goal goal;
    goal.global_plan = path;
    goal.goal_tolerance_m = static_cast<float>(goal_tolerance_m);

    navigate_goal_done_ = false;
    navigate_goal_succeeded_ = false;

    rclcpp_action::Client<NavigateTask>::SendGoalOptions opts;
    opts.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<NavigateTask>::SharedPtr& gh) {
            navigate_goal_handle_ = gh;
            if (!gh) {
                RCLCPP_WARN(this->get_logger(), "NavigateTask goal rejected by server");
                navigate_goal_done_ = true;
            }
        };
    opts.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<NavigateTask>::WrappedResult& r) {
            navigate_goal_succeeded_ =
                (r.code == rclcpp_action::ResultCode::SUCCEEDED) && r.result && r.result->success;
            navigate_goal_done_ = true;
        };

    navigate_client_->async_send_goal(goal, opts);
    RCLCPP_INFO(this->get_logger(), "Dispatched NavigateTask with %zu waypoints",
                path.poses.size());
}

// ---------------------------------------------------------------------------
// Tracking point callback (for direct mode completion detection)
// ---------------------------------------------------------------------------

void CoveragePlannerNode::tracking_point_callback(
    const airstack_msgs::msg::Odometry::SharedPtr msg) {
    tracking_point_odom_ = *msg;
    got_tracking_point_ = true;
}

// ---------------------------------------------------------------------------
// Trajectory mode helper
// ---------------------------------------------------------------------------

bool CoveragePlannerNode::set_trajectory_mode(int32_t mode) {
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

// ---------------------------------------------------------------------------
// Direct trajectory override (bypasses local planner entirely)
// ---------------------------------------------------------------------------

void CoveragePlannerNode::send_trajectory_override(
    const std::vector<Waypoint>& waypoints, double velocity) {
    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    traj.header.stamp = this->now();
    traj.header.frame_id = world_frame_id_;

    // Time between waypoints based on spacing and velocity
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        airstack_msgs::msg::WaypointXYZVYaw wp;
        wp.position.x = waypoints[i].x;
        wp.position.y = waypoints[i].y;
        wp.position.z = waypoints[i].z;
        wp.velocity = velocity;
        wp.yaw = waypoints[i].yaw;
        traj.waypoints.push_back(wp);
    }

    pub_traj_override_->publish(traj);
    RCLCPP_INFO(this->get_logger(),
                "Published trajectory override with %zu waypoints at %.1f m/s",
                waypoints.size(), velocity);
}

}  // namespace coverage_planner

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<coverage_planner::CoveragePlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
