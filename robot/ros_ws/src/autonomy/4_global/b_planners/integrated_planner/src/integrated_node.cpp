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

    pub_global_plan =
        node_handle_->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
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

    // Set up the timer
    plan_timer_ = node_handle_->create_wall_timer(std::chrono::seconds(1),
                                                  std::bind(&PlannerNode::timerCallback, this));
    // Set up the service
    srv_exploration_toggle = node_handle_->create_service<std_srvs::srv::Trigger>(
        srv_exploration_toggle_topic_, std::bind(&PlannerNode::ExplorationToggleCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_handle_->get_logger(), "Exploration node initialized");
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

    param_set("pub_global_plan_topic", "/global_plan", pub_global_plan_topic_);
    param_set("pub_trajectory_viz_topic", "/traj_viz", pub_trajectory_viz_topic_);
    param_set("pub_clustered_frontier_viz_topic", "/clustered_frontier_viz", pub_clustered_frontier_viz_topic_);

    param_set("srv_exploration_toggle_topic", "/robot_1/behavior/global_plan_toggle", srv_exploration_toggle_topic_);
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

void PlannerNode::generate_plan()
{
    RCLCPP_INFO(node_handle_->get_logger(), "Starting to generate plan...");

    Eigen::Vector3d start_point;
    if (generated_paths_.size() == 0)
    {
        start_point.x() = current_location.translation.x;
        start_point.y() = current_location.translation.y;
        start_point.z() = current_location.translation.z;
    }
    else
    {
        start_point.x() = generated_paths_.back().poses.back().pose.position.x;
        start_point.y() = generated_paths_.back().poses.back().pose.position.y;
        start_point.z() = generated_paths_.back().poses.back().pose.position.z;
    }

    openvdb::math::Transform::ConstPtr tf = map_manager_->get_grid_transform();

    openvdb::Vec3d start_world(start_point.x(), start_point.y(), start_point.z());
    openvdb::Coord start_coord(openvdb::Coord::round(tf->worldToIndex(start_world)));

    Eigen::Vector3d end_point;
    double end_yaw;

    std::vector<ScoredViewpoint> goal_candidates;
    map_manager_->frontier_manager_.collect_ranked_best_viewpoints(start_point, goal_candidates);

    if (goal_candidates.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No viewpoint available from the list");
        return;
    }

    std::vector<openvdb::Vec3d> path_smooth_world;
    for (auto &vp : goal_candidates)
    {
        openvdb::Vec3d goal_world(vp.pos_.x(), vp.pos_.y(), vp.pos_.z());
        openvdb::Coord goal_coord(openvdb::Coord::round(tf->worldToIndex(goal_world)));

        astar_planner_.reset();
        int plan_result;
        plan_result = astar_planner_.search(start_coord, goal_coord);

        if (plan_result == Astar::REACH_END)
        {
            astar_planner_.pathSmooth(path_smooth_world);
            break;
        }
    }

    if (path_smooth_world.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "No path generated from all viewpoints");
        return;
    }

    // visualize
    std::vector<openvdb::Vec3d> path_astar = astar_planner_.getPathAstar();
    visualization_msgs::msg::MarkerArray astar_result_vis = path_to_marker_array(path_astar, 1.0);
    for (auto &m : astar_result_vis.markers)
    {
        m.header.frame_id = "map";
    }
    planning_astar_vis->publish(astar_result_vis);

    visualization_msgs::msg::MarkerArray smoothed_result_vis = path_to_marker_array(path_smooth_world, 1.0);
    for (auto &m : astar_result_vis.markers)
    {
        m.header.frame_id = "map";
    }
    planning_smoothed_vis->publish(smoothed_result_vis);
}

void PlannerNode::generate_replan()
{
    // ViewPoint start_point;

    // start_point.x = this->current_location.translation.x;
    // start_point.y = this->current_location.translation.y;
    // start_point.z = this->current_location.translation.z;

    // tf2::Quaternion q;
    // tf2::fromMsg(this->current_location.rotation, q);
    // q.normalize();

    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // start_point.orientation = this->current_location.rotation;
    // start_point.orientation_set = true;
    // start_point.orientation_yaw = yaw;

    // float timeout_duration = 5.0;
    // std::optional<Path> gen_path_opt = this->exploration_planner->plan_to_given_waypoint(start_point, curr_goal);
    // if (gen_path_opt.has_value() && gen_path_opt.value().size() > 0)
    // {
    //     RCLCPP_INFO(node_handle_->get_logger(), "Generated path with %ld points",
    //                 gen_path_opt.value().size());

    //     // set the current goal location
    //     this->current_goal_location = geometry_msgs::msg::Transform();
    //     this->current_goal_location.translation.x = std::get<0>(gen_path_opt.value().back());
    //     this->current_goal_location.translation.y = std::get<1>(gen_path_opt.value().back());
    //     this->current_goal_location.translation.z = std::get<2>(gen_path_opt.value().back());
    //     float z_rot = std::get<3>(gen_path_opt.value().back());
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, z_rot); // Roll = 0, Pitch = 0, Yaw = yaw
    //     q.normalize();
    //     this->current_goal_location.rotation.x = q.x();
    //     this->current_goal_location.rotation.y = q.y();
    //     this->current_goal_location.rotation.z = q.z();
    //     this->current_goal_location.rotation.w = q.w();

    //     // publish the path
    //     nav_msgs::msg::Path generated_single_path;
    //     generated_single_path = nav_msgs::msg::Path();
    //     generated_single_path.header.stamp = node_handle_->now();
    //     generated_single_path.header.frame_id = world_frame_id_;
    //     for (auto point : gen_path_opt.value())
    //     {
    //         geometry_msgs::msg::PoseStamped point_msg;
    //         point_msg.pose.position.x = std::get<0>(point);
    //         point_msg.pose.position.y = std::get<1>(point);
    //         point_msg.pose.position.z = std::get<2>(point);
    //         // convert yaw rotation to quaternion
    //         tf2::Quaternion q;
    //         q.setRPY(0, 0, std::get<3>(point)); // Roll = 0, Pitch = 0, Yaw = yaw
    //         q.normalize();
    //         point_msg.pose.orientation.x = q.x();
    //         point_msg.pose.orientation.y = q.y();
    //         point_msg.pose.orientation.z = q.z();
    //         point_msg.pose.orientation.w = q.w();
    //         point_msg.header.stamp = node_handle_->now();
    //         generated_single_path.poses.push_back(point_msg);
    //     }
    //     geometry_msgs::msg::PoseStamped last_goal_loc = generated_single_path.poses.back();
    //     this->current_goal_location.translation.x = last_goal_loc.pose.position.x;
    //     this->current_goal_location.translation.y = last_goal_loc.pose.position.y;
    //     this->current_goal_location.translation.z = last_goal_loc.pose.position.z;
    //     this->current_goal_location.rotation.z = last_goal_loc.pose.orientation.z;
    //     this->generated_paths_.push_back(generated_single_path);

    //     // debug vis of RRT
    //     PointSet coarse_path = this->exploration_planner->rrt_planner_.getCoarsePath();
    //     visualization_msgs::msg::MarkerArray rrt_vis_array;

    //     if (coarse_path.empty())
    //     {
    //         planning_debug_vis->publish(rrt_vis_array);
    //     }
    //     else
    //     {
    //         const std::string frame_id = "map";
    //         const rclcpp::Time stamp = node_handle_->now();

    //         // ----- LINE_STRIP for the polyline -----
    //         visualization_msgs::msg::Marker line;
    //         line.header.frame_id = frame_id;
    //         line.header.stamp = stamp;
    //         line.ns = "rrt_coarse_path";
    //         line.id = 0;
    //         line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    //         line.action = visualization_msgs::msg::Marker::ADD;
    //         line.pose.orientation.w = 1.0; // identity
    //         line.scale.x = 0.03;           // line width (m)
    //         line.color.r = 0.1f;
    //         line.color.g = 0.5f;
    //         line.color.b = 1.0f;
    //         line.color.a = 0.9f;
    //         line.lifetime = rclcpp::Duration(0, 0); // forever

    //         line.points.reserve(coarse_path.size());
    //         for (const auto &v : coarse_path)
    //         {
    //             geometry_msgs::msg::Point p;
    //             p.x = v.x;
    //             p.y = v.y;
    //             p.z = v.z;
    //             line.points.push_back(p);
    //         }
    //         rrt_vis_array.markers.push_back(line);

    //         // ----- SPHERE_LIST for vertices -----
    //         visualization_msgs::msg::Marker verts;
    //         verts.header.frame_id = frame_id;
    //         verts.header.stamp = stamp;
    //         verts.ns = "rrt_coarse_path";
    //         verts.id = 1;
    //         verts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    //         verts.action = visualization_msgs::msg::Marker::ADD;
    //         verts.pose.orientation.w = 1.0;
    //         verts.scale.x = 0.10;
    //         verts.scale.y = 0.10;
    //         verts.scale.z = 0.10; // sphere diameter (m)
    //         verts.color.r = 1.0f;
    //         verts.color.g = 0.2f;
    //         verts.color.b = 0.2f;
    //         verts.color.a = 0.9f;
    //         verts.lifetime = rclcpp::Duration(0, 0);

    //         verts.points.reserve(coarse_path.size());
    //         for (const auto &v : coarse_path)
    //         {
    //             geometry_msgs::msg::Point p;
    //             p.x = v.x;
    //             p.y = v.y;
    //             p.z = v.z;
    //             verts.points.push_back(p);
    //         }
    //         rrt_vis_array.markers.push_back(verts);

    //         // publish
    //         planning_debug_vis->publish(rrt_vis_array);
    //     }
    // }
    // else
    // {
    //     RCLCPP_ERROR(node_handle_->get_logger(), "Failed to generate path, size was 0");
    //     this->last_replan_failed = true;
    // }
}

void PlannerNode::publish_plan()
{
    // nav_msgs::msg::Path full_path;
    // for (auto path : this->generated_paths_)
    // {
    //     for (auto point : path.poses)
    //     {
    //         full_path.poses.push_back(point);
    //     }
    // }
    // full_path.header.stamp = node_handle_->now();
    // full_path.header.frame_id = this->world_frame_id_;
    // this->pub_global_plan->publish(full_path);
    // RCLCPP_INFO(node_handle_->get_logger(), "Published full path");

    // if (!full_path.poses.empty())
    // {
    //     this->pub_goal_posestamped->publish(full_path.poses.back());
    // }
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
        current_location = transform_stamped.transform;
        if (!received_first_robot_tf)
        {
            received_first_robot_tf = true;
            last_location = current_location;
            last_position_change = node_handle_->now();
            RCLCPP_WARN(node_handle_->get_logger(), "Received first robot_tf");
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node_handle_->get_logger(), "Robot tf not received: %s", ex.what());
    }

    generate_plan();

    RCLCPP_WARN(node_handle_->get_logger(), "Timer callback end");

    // if (this->enable_exploration)
    // {
    //     if (!this->is_path_executing)
    //     {
    //         if (this->received_first_robot_tf)
    //         {
    //             this->last_replan_failed = false;
    //             this->generate_plan();
    //             current_goal_vp = this->exploration_planner->rrt_planner_.getExecutingPath().back();
    //             this->publish_plan();
    //             this->is_path_executing = true;
    //             this->last_position_change = this->now(); // Reset stall timer when starting new plan
    //         }
    //         else
    //         {
    //             RCLCPP_INFO(this->get_logger(), "viewpoint list size is %zu", this->exploration_planner->viewp_sample_->viewpoint_list_->size());
    //             RCLCPP_INFO(this->get_logger(), "tf received is %d", this->received_first_robot_tf);
    //         }
    //     }
    //     else
    //     {
    //         // check if the robot has reached the goal point
    //         std::tuple<float, float, float> current_point = std::make_tuple(
    //             this->current_location.translation.x, this->current_location.translation.y,
    //             this->current_location.translation.z);
    //         std::tuple<float, float, float> goal_point = std::make_tuple(
    //             this->current_goal_location.translation.x, this->current_goal_location.translation.y,
    //             this->current_goal_location.translation.z);
    //         if (get_point_distance(current_point, goal_point) < this->exploration_planner->path_end_threshold_m)
    //         {
    //             // this->is_path_executing = false;
    //             // this->generated_paths_.clear();
    //             RCLCPP_INFO(this->get_logger(), "Reached goal point, planning for the next section");
    //             this->last_replan_failed = false;
    //             this->generate_plan();
    //             this->publish_plan();
    //         }
    //         else if (this->exploration_planner->check_curr_path_collision()) // executing path will collide
    //         {
    //             ViewPoint coll_point = this->exploration_planner->get_curr_path_collision();
    //             visualization_msgs::msg::Marker m_coll;
    //             m_coll.header.frame_id = "map";
    //             m_coll.header.stamp = this->now();

    //             m_coll.ns = "coll_pt"; // namespace for RViz
    //             m_coll.id = 0;         // unique per namespace
    //             m_coll.type = visualization_msgs::msg::Marker::CUBE;
    //             m_coll.action = visualization_msgs::msg::Marker::ADD;

    //             // Pose: CENTER of the cube at your object's xyz
    //             m_coll.pose.position.x = coll_point.x;
    //             m_coll.pose.position.y = coll_point.y;
    //             m_coll.pose.position.z = coll_point.z;
    //             m_coll.pose.orientation.x = 0.0;
    //             m_coll.pose.orientation.y = 0.0;
    //             m_coll.pose.orientation.z = 0.0;
    //             m_coll.pose.orientation.w = 1.0; // identity (no rotation)

    //             // Scale: full side lengths (must be > 0)
    //             m_coll.scale.x = 1.5;
    //             m_coll.scale.y = 1.5;
    //             m_coll.scale.z = 1.5;

    //             // Color: remember alpha must be > 0 to see it in RViz
    //             m_coll.color.r = 1.0f;
    //             m_coll.color.g = 0.0f;
    //             m_coll.color.b = 1.0f;
    //             m_coll.color.a = 0.8f;

    //             m_coll.frame_locked = false;
    //             pub_coll_point_vis->publish(m_coll);

    //             RCLCPP_INFO(this->get_logger(), "Current path will collide with updated map, replan.");
    //             if (this->last_replan_failed)
    //             {
    //                 RCLCPP_INFO(this->get_logger(), "5 replans fails for the current goal, replan with new sampled viewpoints");
    //                 this->generated_paths_.clear();
    //                 this->last_replan_failed = false;
    //                 this->generate_plan();
    //                 this->publish_plan();
    //                 this->is_path_executing = true;
    //                 this->last_position_change = this->now();
    //             }
    //             else if (this->exploration_planner->rrt_planner_.getExecutingPath().empty())
    //             {
    //                 RCLCPP_INFO(this->get_logger(), "Current RRT path empty, replan with new sampled viewpoints");
    //                 this->generated_paths_.clear();
    //                 this->last_replan_failed = false;
    //                 this->generate_plan();
    //                 this->publish_plan();
    //                 this->is_path_executing = true;
    //                 this->last_position_change = this->now();
    //             }
    //             else
    //             {
    //                 this->generated_paths_.clear();
    //                 RCLCPP_INFO_STREAM(this->get_logger(), "Replan with current goal" << current_goal_vp.x << ", " << current_goal_vp.y << ", " << current_goal_vp.z);
    //                 this->generate_replan(current_goal_vp);
    //                 this->publish_plan();
    //                 this->is_path_executing = true;
    //                 this->last_position_change = this->now();
    //             }
    //         }
    //         else
    //         {
    //             // Check if position has changed significantly
    //             std::tuple<float, float, float> last_point = std::make_tuple(this->last_location.translation.x,
    //                                                                          this->last_location.translation.y,
    //                                                                          this->last_location.translation.z);

    //             if (get_point_distance(current_point, last_point) > this->position_change_threshold)
    //             {
    //                 this->last_location = this->current_location;
    //                 this->last_position_change = this->now();
    //                 RCLCPP_INFO(this->get_logger(), "Path exec normally.");
    //             }
    //             else
    //             {
    //                 // Check if we've been stationary for too long
    //                 rclcpp::Duration stall_duration = this->now() - this->last_position_change;
    //                 if (stall_duration.seconds() > this->stall_timeout_seconds)
    //                 {
    //                     RCLCPP_INFO(this->get_logger(), "Robot stationary for %f seconds, clearing plan",
    //                                 stall_duration.seconds());
    //                     this->is_path_executing = false;
    //                     this->generated_paths_.clear();
    //                     this->last_position_change = this->now(); // Reset timer to avoid spam
    //                 }
    //             }
    //         }
    //     }
    // }
}