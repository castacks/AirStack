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

#include "../include/integrated_node.hpp"
#include "../include/vis_tools.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/create_timer_ros.h>

#include <vdb_edt/frontier_cluster.h>

PlannerNode::PlannerNode()
{
    node_handle_ = std::make_shared<rclcpp::Node>("integrated_plan_node");

    initialize();
}

void PlannerNode::initialize()
{
    // TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_handle_->get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(node_handle_->get_node_base_interface(),
                                                                                  node_handle_->get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_handle_);

    // Initialize vdb-edt. It contains openvdb::initialize()
    map_manager_ = std::make_shared<VDBMap>(node_handle_, tf_buffer_, tf_listener_);
    RCLCPP_INFO(node_handle_->get_logger(), "Initialized VDB grids");

    // Initialize astar planner.
    astar_planner_.initialize(map_manager_);

    setup_parameters();

    sub_tracking_point = node_handle_->create_subscription<geometry_msgs::msg::PoseStamped>("/tracking_point", 1,
                                                                                            std::bind(&PlannerNode::tracking_point_callback, this, std::placeholders::_1));

    pub_global_plan =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>(pub_global_plan_topic_, 10);
    pub_trajectory_ =
        node_handle_->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/cmd_trajectory", 10);
    pub_trajectory_lines =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);
    pub_clustered_frontier_vis =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>(pub_clustered_frontier_viz_topic_, 10);
    planning_astar_vis =
        node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("/astar_plan_vis", 10);
    planning_smoothed_vis =
        node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("/smoothed_plan_vis", 10);
    pub_goal_posestamped =
        node_handle_->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
    pub_sampled_viewpoints =
        node_handle_->create_publisher<geometry_msgs::msg::PoseArray>("/sampled_viewpoints", 10);
    pub_coll_point_vis =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>("/collision_point", 10);

    pub_valid_viewpoints =
        node_handle_->create_publisher<geometry_msgs::msg::PoseArray>("/valid_viewpoints", 10);

    // Set up the timer
    plan_timer_ = node_handle_->create_wall_timer(std::chrono::seconds(1),
                                                  std::bind(&PlannerNode::timerCallback, this));
    // Set up the service
    srv_exploration_toggle = node_handle_->create_service<std_srvs::srv::Trigger>(
        srv_exploration_toggle_topic_, std::bind(&PlannerNode::ExplorationToggleCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_handle_->get_logger(), "Exploration node initialized");

    has_tracking_point_ = false;
}

void PlannerNode::setup_parameters()
{
    auto log = node_handle_->get_logger();

    auto to_str = [](const auto &v) -> std::string
    {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::string>)
            return v;
        else if constexpr (std::is_same_v<T, bool>)
            return v ? "true" : "false";
        else
            return std::to_string(v);
    };

    auto param_set = [&](std::string_view name, const auto &def, auto &out)
    {
        using T = std::decay_t<decltype(out)>;
        const std::string key{name};
        T val{};

        if (!node_handle_->has_parameter(key))
        {
            val = node_handle_->template declare_parameter<T>(key, static_cast<T>(def));
        }
        else
        {
            node_handle_->get_parameter(key, val);
        }
        out = val;

        if (val == static_cast<T>(def))
        {
            RCLCPP_INFO(log, "Parameter %s uses default: %s", key.c_str(), to_str(val).c_str());
        }
        else
        {
            RCLCPP_INFO(log, "Parameter %s loaded from config: %s", key.c_str(), to_str(val).c_str());
        }
    };

    // ROS2: declare parameters with defaults
    param_set("world_frame_id", "map", world_frame_id_);
    param_set("robot_frame_id", "base_link", robot_frame_id_);

    param_set("sub_target_path_topic", "/perspective_goals", sub_goal_viewpoints_);

    param_set("pub_global_plan_topic", "/global_plan_viz", pub_global_plan_topic_);
    param_set("pub_trajectory_viz_topic", "/traj_viz", pub_trajectory_viz_topic_);
    param_set("pub_clustered_frontier_viz_topic", "/clustered_frontier_viz", pub_clustered_frontier_viz_topic_);

    param_set("srv_exploration_toggle_topic", "/robot_1/behavior/global_plan_toggle", srv_exploration_toggle_topic_);

    param_set("safe_robot_r", 0.8, safe_robot_r_);
    param_set("vox_size", 0.2, voxel_size_);

    safe_index_dist_ = safe_robot_r_ / voxel_size_;
    safe_sq_idx_dist_ = safe_index_dist_ * safe_index_dist_;
}

rclcpp::Node::SharedPtr PlannerNode::get_node_handle() const
{
    return node_handle_;
}

void PlannerNode::ExplorationToggleCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (enable_exploration == false)
    {
        enable_exploration = true;
        response->success = true;
        response->message = "Exploration enabled";
        RCLCPP_INFO(node_handle_->get_logger(), "Exploration enabled");
    }
    else
    {
        enable_exploration = false;
        response->success = true;
        response->message = "Exploration disabled";
        RCLCPP_INFO(node_handle_->get_logger(), "Exploration disabled");
    }
}

void PlannerNode::tracking_point_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    has_tracking_point_ = true;
    current_tracking_point_ = *msg;
}

void PlannerNode::generate_plan()
{
    RCLCPP_INFO(node_handle_->get_logger(), "Starting to generate plan...");

    Eigen::Vector3d start_point;
    if (current_path_dense_.empty())
    {
        start_point.x() = current_location_.translation.x;
        start_point.y() = current_location_.translation.y;
        start_point.z() = current_location_.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(current_location_.rotation, q);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        next_start_yaw_ = yaw;
    }
    else
    {
        start_point = current_path_dense_.back().pos_;
        next_start_yaw_ = current_path_dense_.back().yaw_;
    }

    openvdb::math::Transform::ConstPtr tf = map_manager_->get_grid_transform();

    openvdb::Vec3d start_world(start_point.x(), start_point.y(), start_point.z());
    openvdb::Coord start_coord(openvdb::Coord::round(tf->worldToIndex(start_world)));

    std::vector<ScoredViewpoint> goal_candidates;
    map_manager_->frontier_manager_.collect_ranked_best_viewpoints(start_point, goal_candidates);

    if (goal_candidates.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No viewpoint available from the list");
        return;
    }

    generated_path_raw_.clear();

    geometry_msgs::msg::PoseArray valid_vp_pose_array;
    valid_vp_pose_array.header.frame_id = "map";
    for (auto &vp : goal_candidates)
    {
        // hacky: isaac-sim has lidar point underground, cannot go down.
        if (vp.pos_.z() < -1.7 || vp.pos_.z() > -0.5 ||
            vp.pos_.x() < -3.0 || vp.pos_.x() > 4.0 ||
            vp.pos_.y() < -2.0 || vp.pos_.y() > 2.0)
        {
            continue;
        }

        geometry_msgs::msg::Pose pose;

        // position
        pose.position.x = vp.pos_.x();
        pose.position.y = vp.pos_.y();
        pose.position.z = vp.pos_.z();

        // yaw -> quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, vp.yaw_);
        q.normalize();
        pose.orientation = tf2::toMsg(q);

        valid_vp_pose_array.poses.push_back(pose);
    }
    valid_vp_pose_array.header.stamp = node_handle_->get_clock()->now();
    pub_valid_viewpoints->publish(valid_vp_pose_array);

    for (auto &vp : goal_candidates)
    {
        // hacky: isaac-sim has lidar point underground, cannot go down.
        if (vp.pos_.z() < -1.7 || vp.pos_.z() > -0.5 ||
            vp.pos_.x() < -3.0 || vp.pos_.x() > 4.0 ||
            vp.pos_.y() < -2.0 || vp.pos_.y() > 2.0)
        {
            continue;
        }
        openvdb::Vec3d goal_world(vp.pos_.x(), vp.pos_.y(), vp.pos_.z());
        openvdb::Coord goal_coord(openvdb::Coord::round(tf->worldToIndex(goal_world)));

        astar_planner_.reset();
        int plan_result;
        plan_result = astar_planner_.search(start_coord, goal_coord);

        if (plan_result == Astar::REACH_END)
        {
            // astar_planner_.pathSmooth(generated_path_raw_);
            astar_planner_.pathShorten(generated_path_raw_);
            next_goal_yaw_ = vp.yaw_;
            break;
        }
    }

    if (generated_path_raw_.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No path generated from all viewpoints");
        return;
    }

    // visualize
    std::vector<openvdb::Vec3d> path_astar = astar_planner_.getPathAstar();
    visualization_msgs::msg::MarkerArray astar_result_vis = path_to_marker_array(path_astar, 0.7);
    for (auto &m : astar_result_vis.markers)
    {
        m.header.frame_id = world_frame_id_;
    }
    planning_astar_vis->publish(astar_result_vis);

    visualization_msgs::msg::MarkerArray smoothed_result_vis = path_to_marker_array(generated_path_raw_, 1.0);
    for (auto &m : smoothed_result_vis.markers)
    {
        m.header.frame_id = world_frame_id_;
    }
    planning_smoothed_vis->publish(smoothed_result_vis);
}

bool PlannerNode::generate_replan(TimedXYZYaw start_point, TimedXYZYaw end_point)
{
    RCLCPP_INFO(node_handle_->get_logger(), "Generating  replan...");
    openvdb::math::Transform::ConstPtr tf = map_manager_->get_grid_transform();

    openvdb::Vec3d start_world(start_point.pos_.x(), start_point.pos_.y(), start_point.pos_.z());
    openvdb::Coord start_coord(openvdb::Coord::round(tf->worldToIndex(start_world)));

    generated_path_raw_.clear();

    openvdb::Vec3d goal_world(end_point.pos_.x(), end_point.pos_.y(), end_point.pos_.z());
    openvdb::Coord goal_coord(openvdb::Coord::round(tf->worldToIndex(goal_world)));

    astar_planner_.reset();
    int plan_result;
    plan_result = astar_planner_.search(start_coord, goal_coord);

    if (plan_result == Astar::REACH_END)
    {
        // astar_planner_.pathSmooth(generated_path_raw_);
        astar_planner_.pathShorten(generated_path_raw_);
    }
    else
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No path generated for replan.");
        return false;
    }

    next_start_yaw_ = start_point.yaw_;
    next_goal_yaw_ = end_point.yaw_;

    // visualize
    std::vector<openvdb::Vec3d> path_astar = astar_planner_.getPathAstar();
    visualization_msgs::msg::MarkerArray astar_result_vis = path_to_marker_array(path_astar, 0.7);
    for (auto &m : astar_result_vis.markers)
    {
        m.header.frame_id = world_frame_id_;
    }
    planning_astar_vis->publish(astar_result_vis);

    visualization_msgs::msg::MarkerArray smoothed_result_vis = path_to_marker_array(generated_path_raw_, 1.0);
    for (auto &m : smoothed_result_vis.markers)
    {
        m.header.frame_id = world_frame_id_;
    }
    planning_smoothed_vis->publish(smoothed_result_vis);

    return true;
}

void PlannerNode::interpolate_plan(double t_offset)
{
    generated_path_dense_.clear();

    if (generated_path_raw_.empty())
    {
        return;
    }

    // Edge case: only one point t=0
    if (generated_path_raw_.size() == 1)
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Trying to interpolate a path with only one point, check generate_plan()");
        return;
    }

    if (interpolate_step_ <= 0.0 || cruise_speed_ <= 0.0)
    {
        RCLCPP_WARN(node_handle_->get_logger(), "interpolate_step_ or cruise_speed_ cannot be <= 0.");
        return;
    }

    double t = t_offset;

    // First point
    openvdb::Vec3d vdb_last = generated_path_raw_.front();
    Eigen::Vector3d last_pos(vdb_last.x(), vdb_last.y(), vdb_last.z());

    // Add desired speed for the first point
    openvdb::Vec3d vdb_1 = generated_path_raw_[1];
    Eigen::Vector3d pos_1(vdb_1.x(), vdb_1.y(), vdb_1.z());
    Eigen::Vector3d diff_1 = pos_1 - last_pos;

    TimedXYZYaw first_pt;
    first_pt.pos_ = last_pos;
    first_pt.yaw_ = 0.0;
    first_pt.time_from_start_ = t;

    // Desired Velocity
    if (diff_1.norm() > 1e-6)
    {
        first_pt.desired_vel_ = diff_1.normalized() * cruise_speed_;
    }

    generated_path_dense_.push_back(first_pt);

    // Iterate each [P_i, P_i+1]
    for (std::size_t i = 0; i + 1 < generated_path_raw_.size(); ++i)
    {
        const openvdb::Vec3d &p0_vdb = generated_path_raw_[i];
        const openvdb::Vec3d &p1_vdb = generated_path_raw_[i + 1];

        Eigen::Vector3d p0(p0_vdb.x(), p0_vdb.y(), p0_vdb.z());
        Eigen::Vector3d p1(p1_vdb.x(), p1_vdb.y(), p1_vdb.z());

        Eigen::Vector3d seg = p1 - p0;
        double seg_len = seg.norm();

        // Not likely but if almost the same
        if (seg_len < 1e-6)
        {
            continue;
        }

        Eigen::Vector3d dir = seg / seg_len;

        double dt = interpolate_step_ / cruise_speed_;
        // Interpolate (not include section end)
        for (double dist = interpolate_step_; dist < seg_len; dist += interpolate_step_)
        {
            Eigen::Vector3d pos = p0 + dir * dist;
            t += dt;

            TimedXYZYaw pt;
            pt.pos_ = pos;
            pt.yaw_ = 0.0; // 先不管 yaw
            pt.time_from_start_ = t;

            // Desired Velocity
            pt.desired_vel_ = dir * cruise_speed_;

            generated_path_dense_.push_back(pt);

            last_pos = pos;
        }

        // Section end point p1
        double ds = (p1 - last_pos).norm();
        double dt_1 = ds / cruise_speed_;
        t += dt_1;

        TimedXYZYaw pt;
        pt.pos_ = p1;
        pt.yaw_ = 0.0;
        pt.time_from_start_ = t;

        // Add desired vel if not the last point
        if (i + 2 < generated_path_raw_.size())
        {
            const openvdb::Vec3d &p2_vdb = generated_path_raw_[i + 2];
            Eigen::Vector3d p2(p2_vdb.x(), p2_vdb.y(), p2_vdb.z());
            Eigen::Vector3d seg2 = p2 - p1;
            double seg2_len = seg2.norm();
            if (seg2_len >= 1e-6)
            {
                Eigen::Vector3d dir2 = seg2 / seg2_len;
                pt.desired_vel_ = dir2 * cruise_speed_;
            }
        }

        generated_path_dense_.push_back(pt);

        last_pos = p1;
    }

    // Interpolate Yaw:
    const double t_start = generated_path_dense_.front().time_from_start_;
    const double t_end = generated_path_dense_.back().time_from_start_;
    const double duration = t_end - t_start;

    auto wrapAngle = [](double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    };

    if (duration <= 1e-6)
    {
        const double yaw = wrapAngle(next_goal_yaw_);
        for (auto &pt : generated_path_dense_)
        {
            pt.yaw_ = yaw;
        }
        return;
    }

    const double yaw_start = wrapAngle(next_start_yaw_);
    const double yaw_diff = wrapAngle(next_goal_yaw_ - yaw_start);

    auto blend = [](double s)
    {
        // S-curve: 3s^2 - 2s^3
        return 3.0 * s * s - 2.0 * s * s * s;
    };

    for (auto &pt : generated_path_dense_)
    {
        double s = (pt.time_from_start_ - t_start) / duration; // [0,1]
        if (s < 0.0)
        {
            s = 0.0;
        }
        else if (s > 1.0)
        {
            s = 1.0;
        }

        double sb = blend(s);
        double yaw = yaw_start + sb * yaw_diff;
        pt.yaw_ = wrapAngle(yaw);
    }
}

void PlannerNode::publish_plan()
{
    if (current_path_dense_.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Current path is empty after trim, no traj to publish.");
        return;
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = node_handle_->now();
    traj_msg.header.frame_id = world_frame_id_;

    traj_msg.joint_names.clear();
    traj_msg.joint_names.push_back(robot_frame_id_);

    traj_msg.points.clear();
    traj_msg.points.reserve(current_path_dense_.size());

    for (size_t i = 0; i < current_path_dense_.size(); ++i)
    {
        const TimedXYZYaw &p = current_path_dense_[i];

        // Within horizon, but at least one point
        if (p.time_from_start_ > traj_horizon_ && !traj_msg.points.empty())
        {
            break;
        }

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt_msg;

        geometry_msgs::msg::Transform tf_msg;
        tf_msg.translation.x = p.pos_(0);
        tf_msg.translation.y = p.pos_(1);
        tf_msg.translation.z = p.pos_(2);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.yaw_);
        tf_msg.rotation = tf2::toMsg(q);

        pt_msg.transforms.push_back(tf_msg);
        pt_msg.time_from_start = rclcpp::Duration::from_seconds(p.time_from_start_);

        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = p.desired_vel_.x();
        vel_msg.linear.y = p.desired_vel_.y();
        vel_msg.linear.z = p.desired_vel_.z();
        pt_msg.velocities.push_back(vel_msg);

        traj_msg.points.push_back(std::move(pt_msg));
    }

    pub_trajectory_->publish(traj_msg);
    visualize_local_traj();
    visualize_full_path();
}

void PlannerNode::visualize_local_traj()
{
    visualization_msgs::msg::Marker line;
    line.header.stamp = node_handle_->now();
    line.header.frame_id = world_frame_id_;

    line.ns = "local_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.pose.orientation.w = 1.0;

    line.scale.x = 0.08;

    line.color.r = 0.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;

    line.points.clear();

    const double horizon = traj_horizon_;

    for (size_t i = 0; i < current_path_dense_.size(); ++i)
    {
        const TimedXYZYaw &p = current_path_dense_[i];

        if (p.time_from_start_ > horizon && !line.points.empty())
        {
            break;
        }

        geometry_msgs::msg::Point pt;
        pt.x = p.pos_(0);
        pt.y = p.pos_(1);
        pt.z = p.pos_(2);

        line.points.push_back(pt);
    }

    if (line.points.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No points within horizon, skip local path visualization.");
        return;
    }

    pub_trajectory_lines->publish(line);
}

void PlannerNode::visualize_full_path()
{
    visualization_msgs::msg::Marker line;
    line.header.stamp = node_handle_->now();
    line.header.frame_id = world_frame_id_;

    line.ns = "global_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.pose.orientation.w = 1.0;

    line.scale.x = 0.05;

    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;

    line.points.clear();

    const double horizon = traj_horizon_;

    for (size_t i = 1; i < current_path_dense_.size(); ++i)
    {
        const TimedXYZYaw &p = current_path_dense_[i];

        geometry_msgs::msg::Point pt;
        pt.x = p.pos_(0);
        pt.y = p.pos_(1);
        pt.z = p.pos_(2);

        line.points.push_back(pt);
    }

    if (line.points.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No points at all, skip global path visualization.");
        return;
    }

    pub_global_plan->publish(line);
}

void PlannerNode::trim_covered_path()
{
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Trimming");
    if (has_tracking_point_)
    {
        Eigen::Vector3d p_track(current_tracking_point_.pose.position.x,
                                current_tracking_point_.pose.position.y,
                                current_tracking_point_.pose.position.z);
        RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Latest ref point is \n"
                                                           << p_track);
        double best_d2 = std::numeric_limits<double>::infinity();
        size_t best_i = 0;
        for (size_t i = 0; i < current_path_dense_.size(); ++i)
        {
            const Eigen::Vector3d &pi = current_path_dense_[i].pos_;
            Eigen::Vector3d diff = p_track - pi;
            double d2 = diff.squaredNorm();
            if (d2 < best_d2)
            {
                best_d2 = d2;
                best_i = i;

                if (d2 < 1e-6)
                {
                    break;
                }
            }
        }
        current_path_dense_.erase(current_path_dense_.begin(),
                                  current_path_dense_.begin() + best_i);

        double time_offset = current_path_dense_.front().time_from_start_;
        for (auto &pt : current_path_dense_)
        {
            pt.time_from_start_ -= time_offset;
        }
        return;
    }
    // Check current progress
    Eigen::Vector3d curr_p(current_location_.translation.x,
                           current_location_.translation.y,
                           current_location_.translation.z);

    double seg_s = 0.0;
    Eigen::Vector3d proj_p = curr_p;

    int num_points_to_pop = 0;

    for (size_t i = 0; i + 1 < current_path_dense_.size(); ++i)
    {
        const Eigen::Vector3d &Pi = current_path_dense_[i].pos_;
        const Eigen::Vector3d &Pj = current_path_dense_[i + 1].pos_;

        Eigen::Vector3d v = Pj - Pi;
        Eigen::Vector3d w = curr_p - Pi;

        double v_dot_v = v.dot(v);
        if (v_dot_v < 1e-6)
        {
            continue;
        }

        double s = w.dot(v) / v_dot_v;

        if (s < 0.0)
        {
            // Current pose is in the front of the segment[0],
            // indicating that the current pose hasn't reached the beginning of the entire current_path_ yet
            // Cannot remove anything, quit.
            seg_s = 0.0;
            proj_p = Pi;
            // num_points_to_pop = 1;
            break;
        }
        else if (s <= 1.0)
        {
            // current pose lies in this section, remove all points before.
            seg_s = s;
            proj_p = Pi + s * v;

            // 0..i-1 all complete, can remove
            num_points_to_pop = static_cast<int>(i) + 1;
            break;
        }
        else // s > 1.0
        {
            // Current pose already move past segment i.
            // current_path_dense_[i] and current_path_dense_[j] can both be removed.
            seg_s = 1.0;
            proj_p = Pj;
            num_points_to_pop = static_cast<int>(i) + 2;
        }
    }

    if (num_points_to_pop > 0)
    {
        // delete num_points_to_pop points, it's removing 0 to i (totally i+1),
        // so new current_path_dense_[0] is the first un-executed waypoint
        current_path_dense_.erase(current_path_dense_.begin(),
                                  current_path_dense_.begin() + num_points_to_pop);
    }

    if (current_path_dense_.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Trimmed all dense path points, check trim_covered_path()");
        return;
    }

    double time_offset = current_path_dense_.front().time_from_start_;
    for (auto &pt : current_path_dense_)
    {
        pt.time_from_start_ -= time_offset;
    }
}

bool PlannerNode::check_local_path_free()
{
    double check_horizon = traj_horizon_;
    for (auto &pt : current_path_dense_)
    {
        if (pt.time_from_start_ > check_horizon)
        {
            break;
        }

        double sq_dist;
        bool q = map_manager_->query_sqdist_at_world(pt.pos_, sq_dist);

        // unknown -> free, may change later, but must align with A* close set judge
        // if (q && sq_dist <= safe_sq_idx_dist_)
        if (sq_dist <= safe_sq_idx_dist_)
        {
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "having collision point at \n" << pt.pos_);
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "sq_dist is " << sq_dist << " safe sq_dist is " << safe_sq_idx_dist_);

            return false;
        }
    }
    return true;
}

void PlannerNode::timerCallback()
{
    RCLCPP_WARN(node_handle_->get_logger(), "Timer callback start");
    // get current TF to world
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(world_frame_id_,
                                                                                             robot_frame_id_,
                                                                                             rclcpp::Time(0));
        current_location_ = transform_stamped.transform;
        if (!received_first_robot_tf)
        {
            received_first_robot_tf = true;
            last_location_ = current_location_;
            possible_stuck_location_ = current_location_;
            last_position_change_ = node_handle_->now();
            RCLCPP_WARN(node_handle_->get_logger(), "Received first robot_tf");
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node_handle_->get_logger(), "Robot tf not received: %s", ex.what());
    }

    if (this->enable_exploration)
    {
        // RCLCPP_WARN(node_handle_->get_logger(), "Exploration triggered, generating plans...");
        // First plan: no generated path yet
        if (current_path_dense_.empty())
        {
            double t_start = 0;
            generate_plan();
            interpolate_plan(t_start);
            current_path_dense_.insert(current_path_dense_.end(),
                                       generated_path_dense_.begin(),
                                       generated_path_dense_.end());
        }
        // Replan: adding the next path section
        else if (current_path_dense_.back().time_from_start_ < replan_remain_time_)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "Approaching current goal, adding a replan.");
            double t_start = current_path_dense_.back().time_from_start_;
            generate_plan();
            interpolate_plan(t_start);
            current_path_dense_.insert(current_path_dense_.end(),
                                       generated_path_dense_.begin(),
                                       generated_path_dense_.end());
        }

        if (current_path_dense_.size() < 2)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "No path to execute.");
            return;
        }

        // RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Before trim, point num is "
        //                                                    << current_path_dense_.size());
        // Check current position, removed executed path, reset time: current_path_dense_[0] has 0 time from start
        // RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Before trim, front point is \n"
        //                                                    << current_path_dense_[0].pos_);
        trim_covered_path();
        // if (!current_path_dense_.empty())
        // {
        //     RCLCPP_WARN_STREAM(node_handle_->get_logger(), "After trim, front point is \n"
        //                                                        << current_path_dense_[0].pos_);
        // }
        // RCLCPP_WARN_STREAM(node_handle_->get_logger(), "After trim, point num is "
        //                                                    << current_path_dense_.size());

        // Check Collision
        if (!check_local_path_free())
        {
            double t_start = 0;
            // First try replan based on current target, if fail, plan based on all viewpoints
            RCLCPP_WARN(node_handle_->get_logger(), "Collision ahead detected, replan.");
            if (!generate_replan(current_path_dense_[1], current_path_dense_.back()))
            {
                // make the current_path_dense_ end at [1], for generate_plan() starting point.
                RCLCPP_WARN(node_handle_->get_logger(), "Replan failed, generate new plan.");
                current_path_dense_.resize(1);
                generate_plan();
            }
            interpolate_plan(t_start);
            current_path_dense_.clear();
            current_path_dense_.insert(current_path_dense_.end(),
                                       generated_path_dense_.begin(),
                                       generated_path_dense_.end());
            if (current_path_dense_.size() < 2)
            {
                RCLCPP_WARN(node_handle_->get_logger(), "After replan, no path to execute.");
                return;
            }
        }
        else
        // Stuck: stay in a position for too long
        {
            Eigen::Vector3d possible_stuck_point(possible_stuck_location_.translation.x,
                                                 possible_stuck_location_.translation.y,
                                                 possible_stuck_location_.translation.z);
            Eigen::Vector3d current_point(current_location_.translation.x,
                                          current_location_.translation.y,
                                          current_location_.translation.z);
            if ((possible_stuck_point - current_point).norm() > position_change_threshold)
            {
                possible_stuck_location_ = current_location_;
                last_position_change_ = node_handle_->now();
                RCLCPP_INFO(node_handle_->get_logger(), "Path exec normally.");
            }
            else
            {
                // Check if we've been stationary for too long
                rclcpp::Duration stall_duration = node_handle_->now() - last_position_change_;
                if (stall_duration.seconds() > stall_timeout_seconds)
                {
                    RCLCPP_INFO(node_handle_->get_logger(), "Robot stationary for %f seconds, clearing plan and replan",
                                stall_duration.seconds());
                    double t_start = 0;
                    TimedXYZYaw current_goal = current_path_dense_.back();
                    TimedXYZYaw new_start;
                    new_start.pos_ = Eigen::Vector3d(current_location_.translation.x,
                                                     current_location_.translation.y,
                                                     current_location_.translation.z);
                    new_start.yaw_ = current_path_dense_.front().yaw_;
                    current_path_dense_.clear();
                    // First try replan based on current target, if fail, plan based on all viewpoints
                    if (!generate_replan(new_start, current_goal))
                    {
                        // make the current_path_dense_ end at [1], for generate_plan() starting point.
                        RCLCPP_WARN(node_handle_->get_logger(), "Replan failed, generate new plan.");
                        generate_plan();
                    }
                    interpolate_plan(t_start);
                    current_path_dense_.insert(current_path_dense_.end(),
                                               generated_path_dense_.begin(),
                                               generated_path_dense_.end());
                    if (current_path_dense_.size() < 2)
                    {
                        RCLCPP_WARN(node_handle_->get_logger(), "After replan, no path to execute.");
                        return;
                    }
                    last_position_change_ = node_handle_->now(); // Reset timer to avoid spam
                }
            }
        }

        // Output trajectory
        publish_plan();
    }

    RCLCPP_WARN(node_handle_->get_logger(), "Timer callback end");
}