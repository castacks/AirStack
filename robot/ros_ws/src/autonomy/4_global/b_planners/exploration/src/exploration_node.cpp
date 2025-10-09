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

#include "../include/exploration_node.hpp"
#include "../include/exploration_logic.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "utils/utils.hpp"

std::optional<init_params> ExplorationNode::readParameters()
{
    // Read in parameters based off the default yaml file
    init_params params;
    this->declare_parameter<std::string>("robot_frame_id");
    if (!this->get_parameter("robot_frame_id", this->robot_frame_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: robot_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("lidar_frame_id");
    if (!this->get_parameter("lidar_frame_id", this->lidar_frame_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: lidar_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("map_frame_id");
    if (!this->get_parameter("map_frame_id", this->map_frame_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: map_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_global_plan_topic");
    if (!this->get_parameter("pub_global_plan_topic", this->pub_global_plan_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_global_plan_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_goal_point_viz_topic");
    if (!this->get_parameter("pub_goal_point_viz_topic", this->pub_goal_point_viz_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_goal_point_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_trajectory_viz_topic");
    if (!this->get_parameter("pub_trajectory_viz_topic", this->pub_trajectory_viz_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_trajectory_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_grid_viz_topic");
    if (!this->get_parameter("pub_grid_viz_topic", this->pub_grid_viz_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_grid_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_frontier_viz_topic");
    if (!this->get_parameter("pub_frontier_viz_topic", this->pub_frontier_viz_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_frontier_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_clustered_frontier_viz_topic");
    if (!this->get_parameter("pub_clustered_frontier_viz_topic", this->pub_clustered_frontier_viz_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_clustered_frontier_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_lidar_topic");
    if (!this->get_parameter("sub_lidar_topic", this->sub_lidar_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_map_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_map_topic");
    if (!this->get_parameter("sub_map_topic", this->sub_map_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_map_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_vdb_topic");
    if (!this->get_parameter("sub_vdb_topic", this->sub_vdb_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_vdb_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_robot_tf_topic");
    if (!this->get_parameter("sub_robot_tf_topic", this->sub_robot_tf_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_robot_tf_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("srv_exploration_toggle_topic");
    if (!this->get_parameter("srv_exploration_toggle_topic", this->srv_exploration_toggle_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: srv_exploration_toggle_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<bool>("publish_visualizations");
    if (!this->get_parameter("publish_visualizations", this->publish_visualizations))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: publish_visualizations");
        return std::optional<init_params>{};
    }
    this->declare_parameter<int>("num_paths_to_generate");
    if (!this->get_parameter("num_paths_to_generate", this->num_paths_to_generate_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: num_paths_to_generate");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_start_to_goal_dist_m");
    if (!this->get_parameter("max_start_to_goal_dist_m", params.max_start_to_goal_dist_m))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_start_to_goal_dist_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<int>("checking_point_cnt");
    if (!this->get_parameter("checking_point_cnt", params.checking_point_cnt))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: checking_point_cnt");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_z_change_m");
    if (!this->get_parameter("max_z_change_m", params.max_z_change_m))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_z_change_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("collision_padding_m");
    if (!this->get_parameter("collision_padding_m", params.collision_padding_m))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: collision_padding_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("path_end_threshold_m");
    if (!this->get_parameter("path_end_threshold_m", params.path_end_threshold_m))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: path_end_threshold_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_yaw_change_degrees");
    if (!this->get_parameter("max_yaw_change_degrees", params.max_yaw_change_degrees))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_yaw_change_degrees");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("cluster_cube_dim");
    if (!this->get_parameter("cluster_cube_dim", this->cluster_cube_dim))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: cluster_cube_dim");
        return std::optional<init_params>{};
    }
    this->declare_parameter<int>("voxel_cluster_count_thresh");
    if (!this->get_parameter("voxel_cluster_count_thresh", this->voxel_cluster_count_thresh))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: voxel_cluster_count_thresh");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("position_change_threshold");
    if (!this->get_parameter("position_change_threshold", this->position_change_threshold))
    {

        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: position_change_threshold");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("map_voxel_resolution");
    if (!this->get_parameter("map_voxel_resolution", this->map_voxel_resolution))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: map_voxel_resolution");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("stall_timeout_seconds");
    if (!this->get_parameter("stall_timeout_seconds", this->stall_timeout_seconds))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: stall_timeout_seconds");
        return std::optional<init_params>{};
    }

    // viewpoint sampling
    params.kLOcc_ = this->declare_parameter<double>("l_occ", 1.0);
    this->get_parameter<double>("map_voxel_resolution", params.kVoxSize_);
    params.kBboxLength_ = this->declare_parameter<double>("bbox_length", 3.0);
    params.kBboxBreadth_ = this->declare_parameter<double>("bbox_breadth", 3.0);
    params.kBboxHeight_ = this->declare_parameter<double>("bbox_height", 3.0);
    params.kViewingDistance_ = this->declare_parameter<double>("viewing_distance", 3.0);
    params.kNumViewpointsPerCluster_ = this->declare_parameter<int>("num_viewpts_per_cluster", 6);
    params.kViewingDistanceInnerR_ = this->declare_parameter<double>("kViewingDistanceInnerR", 3.0);
    params.kViewingDistanceOuterR_ = this->declare_parameter<double>("kViewingDistanceOuterR", 6.0);
    params.kViewingDistanceZOffset_ = this->declare_parameter<double>("kViewingDistanceZOffset", 2.0);
    params.kViewpBBoxUnknownFracThresh_ = this->declare_parameter<double>("viewp_bbox_unknown_frac_thresh", 0.85);
    params.kViewpBBoxOccupiedFracThresh_ = this->declare_parameter<double>("viewp_bbox_known_occ_thresh", 0.0);
    params.kCollCheckEndpointOffset_ = this->declare_parameter<double>("coll_check_endpoint_offset", 0.4);
    params.kRobotModelBBoxFraction_ = this->declare_parameter<double>("robot_model_bbox_fraction", 0.1);

    // viewpoint selection
    params.too_close_distance = this->declare_parameter<double>("too_close_distance", 4.0);
    params.too_close_penalty = this->declare_parameter<double>("too_close_penalty", 5.0);
    params.odometry_match_distance = this->declare_parameter<double>("odometry_match_distance", 3.0);
    params.odometry_match_angle = this->declare_parameter<double>("odometry_match_angle", 3.14159);
    params.odometry_match_penalty = this->declare_parameter<double>("odometry_match_penalty", 1000.0);
    params.momentum_collision_check_distance = this->declare_parameter<double>("momentum_collision_check_distance", 15.0);
    params.momentum_collision_check_distance_min = this->declare_parameter<double>("momentum_collision_check_distance_min", 4.0);
    params.momentum_change_max_reward = this->declare_parameter<double>("momentum_change_max_reward", 6.0);
    params.momentum_change_collision_reward = this->declare_parameter<double>("momentum_change_collision_reward", 2.0);
    params.viewpoint_neighborhood_too_close_radius = this->declare_parameter<double>("viewpoint_neighborhood_too_close_radius", 10.0);
    params.viewpoint_neighborhood_sphere_radius = this->declare_parameter<double>("viewpoint_neighborhood_sphere_radius", 6.0);
    params.viewpoint_neighborhood_count = this->declare_parameter<int>("viewpoint_neighborhood_count", 5);
    params.momentum_minimum_distance = this->declare_parameter<double>("momentum_minimum_distance", 0.5);
    params.momentum_time = this->declare_parameter<double>("momentum_time", 2.0);
    params.momentum_collision_check_step_size = this->declare_parameter<double>("momentum_collision_check_step_size", 0.4);

    this->momentum_time_ = params.momentum_time;

    params.bound_exploration_ = this->declare_parameter<bool>("bound_exploration", false);
    params.x_min = this->declare_parameter<double>("x_min", -10.0);
    params.y_min = this->declare_parameter<double>("y_min", -10.0);
    params.z_min = this->declare_parameter<double>("z_min", -10.0);
    params.x_max = this->declare_parameter<double>("x_max", 10.0);
    params.y_max = this->declare_parameter<double>("y_max", 10.0);
    params.z_max = this->declare_parameter<double>("z_max", 10.0);

    // RRT
    params.planner_norm_limit_ = this->declare_parameter<double>("planner_norm_limit", 0.1);
    params.max_explore_dist_ = this->declare_parameter<double>("max_explore_dist", 100.0);
    params.planner_max_iter_ = this->declare_parameter<int>("planner_max_iter", 20000);
    params.max_connection_iter_ = this->declare_parameter<int>("max_connection_iter", 1000);
    params.traj_smooth_horizon_ = this->declare_parameter<int>("traj_smooth_horizon", 50);
    params.rrt_region_nodes_count_ = this->declare_parameter<int>("rrt_region_nodes_count", 16);
    params.rrt_region_nodes_radius_ = this->declare_parameter<double>("rrt_region_nodes_radius", 0.8);
    params.rrt_region_nodes_z_layers_count_up_ = this->declare_parameter<int>("rrt_region_nodes_z_layers_count_up", 1);
    params.rrt_region_nodes_z_layers_step_ = this->declare_parameter<double>("rrt_region_nodes_z_layers_step", 0.4);

    params.priority_level_thred_ = this->declare_parameter<int>("priority_level_thred", 20);
    params.dense_step_ = this->declare_parameter<double>("dense_step", 0.1);
    return params;
}

ExplorationNode::ExplorationNode() : Node("exploration_node")
{
    // Initialize float 543 grid
    openvdb::initialize();

    // Initialize the exploration planner
    std::optional<init_params> params_opt = ExplorationNode::readParameters();

    this->world_frame_id_ = this->map_frame_id_;

    if (params_opt.has_value())
    {
        this->params = params_opt.value();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize exploration planner");
    }

    this->params.voxel_size_m = std::tuple<float, float, float>(map_voxel_resolution, map_voxel_resolution, map_voxel_resolution);

    // Initialize exploration planner
    this->exploration_planner = std::make_unique<ExplorationPlanner>(this->params);
    RCLCPP_INFO(this->get_logger(), "Initialized exploration planner logic");

    VDBUtil::setVoxelSize(*(this->exploration_planner->occ_grid), map_voxel_resolution);
    VDBUtil::setVoxelSize(*(this->exploration_planner->prev_grid), map_voxel_resolution);
    VDBUtil::setVoxelSize(*(this->exploration_planner->frontier_grid), map_voxel_resolution);

    RCLCPP_INFO(this->get_logger(), "Initialized exploration inner voxel grids");
}

void ExplorationNode::initialize()
{

    // TF buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_lidar_topic_, rclcpp::QoS(1).durability_volatile().best_effort(), std::bind(&ExplorationNode::lidarCallback, this, std::placeholders::_1));

    // Trying message filter to avoid extrapolation when lookuptransform, not succedded yet

    // auto node_ptr = rclcpp::Node::shared_from_this();
    // RCLCPP_WARN(this->get_logger(), "Type: %s", typeid(*node_ptr).name());

    // tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0), node_ptr);
    // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // lidar_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    //     this, sub_lidar_topic_, rmw_qos_profile_sensor_data);

    // tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    //         *lidar_filter_sub_, *tf_buffer, map_frame_id_, 10, this->get_node_logging_interface(), this->get_node_clock_interface());

    // tf_filter_->registerCallback(&ExplorationNode::lidarCallback, this);

    // Previous setup

    // this->sub_map = this->create_subscription<visualization_msgs::msg::Marker>(
    //     sub_map_topic_, 10, std::bind(&ExplorationNode::mapCallback, this, std::placeholders::_1));

    // this->sub_vdb = this->create_subscription<std_msgs::msg::ByteMultiArray>(
    //     sub_vdb_topic_, 10, std::bind(&ExplorationNode::vdbCallback, this, std::placeholders::_1));

    this->pub_global_plan = this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
    this->pub_goal_point =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_viz_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);
    this->pub_vdb =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_grid_viz_topic_, 10);
    this->pub_frontier_vis =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_frontier_viz_topic_, 10);
    this->pub_clustered_frontier_vis =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_clustered_frontier_viz_topic_, 10);
    this->planning_debug_vis =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/exploration_debug_vis", 10);
    this->pub_goal_posestamped =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);

    // Set up the timer
    this->timer = this->create_wall_timer(std::chrono::seconds(5),
                                          std::bind(&ExplorationNode::timerCallback, this));
    // Set up the service
    this->srv_exploration_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_exploration_toggle_topic_, std::bind(&ExplorationNode::ExplorationToggleCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Exploration node initialized");
}

void ExplorationNode::ExplorationToggleCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->enable_exploration == false)
    {
        this->enable_exploration = true;
        response->success = true;
        response->message = "Exploration enabled";
        RCLCPP_INFO(this->get_logger(), "Exploration enabled");
    }
    else
    {
        this->enable_exploration = false;
        response->success = true;
        response->message = "Exploration disabled";
        RCLCPP_INFO(this->get_logger(), "Exploration disabled");
    }
}

void ExplorationNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped cloud_origin_tf;

    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *incoming_cloud);

    try
    {
        cloud_origin_tf = this->tf_buffer->lookupTransform(
            map_frame_id_, lidar_frame_id_, msg->header.stamp, rclcpp::Duration(0, 100000000));
        // map_frame_id_, lidar_frame_id_, msg->header.stamp);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Could not transform %s to %s: %s",
                     map_frame_id_.c_str(),
                     lidar_frame_id_.c_str(),
                     ex.what());
        return;
    }

    // pre-processing: remove nan
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*incoming_cloud, *incoming_cloud, idx);

    double min_range = 0.5;

    // pre-processing: remove too close points
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    tmp->reserve(incoming_cloud->size());
    const float r2min = min_range * min_range;
    for (const auto &p : incoming_cloud->points)
    {
        const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
        if (r2 >= r2min)
            tmp->points.push_back(p);
    }
    tmp->width = static_cast<uint32_t>(tmp->points.size());
    tmp->height = 1;
    tmp->is_dense = true;
    incoming_cloud.swap(tmp);

    pcl::transformPointCloud(*incoming_cloud, *incoming_cloud, tf2::transformToEigen(cloud_origin_tf).matrix());
    tf2::Vector3 origin_vec(cloud_origin_tf.transform.translation.x,
                            cloud_origin_tf.transform.translation.y,
                            cloud_origin_tf.transform.translation.z);

    VDBUtil::updateOccMapFromNdArray(this->exploration_planner->occ_grid, incoming_cloud, origin_vec);
    this->exploration_planner->collision_checker_.updateGridPtr(this->exploration_planner->occ_grid);

    // visualization_msgs::msg::Marker vdb_marker_msg;
    // generateVDBMarker(this->exploration_planner->occ_grid, map_frame_id_, vdb_marker_msg);
    // vdb_marker_msg.header.stamp = this->now();
    // this->pub_vdb->publish(vdb_marker_msg);

    // RCLCPP_WARN(this->get_logger(), "Lidar Msg Received");
}

void ExplorationNode::mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    // updating the local voxel points and generating a path only if the path is not executing
    if (!this->received_first_map)
    {
        this->received_first_map = true;
        this->world_frame_id_ = msg->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "Received first map");
        this->params.voxel_size_m =
            std::tuple<float, float, float>(msg->scale.x, msg->scale.y, msg->scale.z);
        // this->exploration_planner = ExplorationPlanner(this->params);
        this->exploration_planner = std::make_unique<ExplorationPlanner>(this->params);

        RCLCPP_INFO(this->get_logger(), "Initialized exploration planner logic");
    }
    this->exploration_planner->voxel_points.clear();
    for (int i = 0; i < msg->points.size(); i++)
    {
        this->exploration_planner->voxel_points.push_back(
            std::tuple<float, float, float>(msg->points[i].x, msg->points[i].y, msg->points[i].z));
    }
}

void ExplorationNode::vdbCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    std::string raw_data(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
    std::istringstream iss(raw_data, std::ios::binary);

    openvdb::io::Stream in_stream(iss);
    std::vector<openvdb::GridBase::Ptr> grids = *in_stream.getGrids();

    if (grids.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received VDB grid message, but no grids were found.");
        return;
    }

    using GridT = openvdb::Grid<openvdb::tree::Tree4<float, 5, 4, 3>::Type>;
    GridT::Ptr float_grid = openvdb::gridPtrCast<GridT>(grids.front());

    if (!float_grid)
    {
        RCLCPP_WARN(this->get_logger(), "First grid in received VDB message is not a FloatGrid.");
        return;
    }

    if (this->exploration_planner)
    {
        this->exploration_planner->vdb_grid = float_grid;

        if (!this->exploration_planner->first_grid_received)
        {
            this->exploration_planner->first_grid_received = true;
            this->exploration_planner->frontier_grid->setTransform(float_grid->transform().copy());
            this->exploration_planner->prev_grid->setTransform(float_grid->transform().copy());
            RCLCPP_WARN(this->get_logger(), "Exploration Grids Initialized");
        }
        // RCLCPP_WARN(this->get_logger(), "Successfully received and stored VDB FloatGrid (%zu active voxels).",
        //              this->exploration_planner->vdb_grid->activeVoxelCount());
        // RCLCPP_WARN(this->get_logger(), "received type is %s", grids.front()->type().c_str());
        // RCLCPP_WARN(this->get_logger(), "our type is %s", this->exploration_planner->vdb_grid->type().c_str());

        // visualization_msgs::msg::Marker vdb_marker_msg;
        // generateVDBMarker(this->exploration_planner->vdb_grid, "map", vdb_marker_msg);
        // vdb_marker_msg.header.stamp = this->now();
        // this->pub_vdb->publish(vdb_marker_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Exploration planner is not yet initialized.");
    }
}

void ExplorationNode::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    // get the current location
    for (int i = 0; i < msg->transforms.size(); i++)
    {
        if (msg->transforms[i].child_frame_id.c_str() == this->robot_frame_id_)
        {
            this->current_location = msg->transforms[i].transform;
            if (!this->received_first_robot_tf)
            {
                this->received_first_robot_tf = true;
                RCLCPP_WARN(this->get_logger(), "Received first robot_tf");
            }
        }
    }
}

void ExplorationNode::generate_plan()
{
    RCLCPP_INFO(this->get_logger(), "Starting to generate plan...");

    // std::tuple<float, float, float, float> start_loc;
    // if (this->generated_paths.size() == 0)
    // {
    //     geometry_msgs::msg::Quaternion orientation = this->current_location.rotation;
    //     tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    //     q.normalize();
    //     double roll, pitch, yaw;
    //     tf2::Matrix3x3 m(q);
    //     m.getRPY(roll, pitch, yaw);
    //     start_loc = std::make_tuple(this->current_location.translation.x,
    //                                 this->current_location.translation.y,
    //                                 this->current_location.translation.z, yaw);
    // }
    // else
    // {
    //     geometry_msgs::msg::Quaternion orientation =
    //         this->generated_paths.back().poses.back().pose.orientation;
    //     tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    //     q.normalize();
    //     double roll, pitch, yaw;
    //     tf2::Matrix3x3 m(q);
    //     m.getRPY(roll, pitch, yaw);
    //     start_loc = std::make_tuple(this->generated_paths.back().poses.back().pose.position.x,
    //                                 this->generated_paths.back().poses.back().pose.position.y,
    //                                 this->generated_paths.back().poses.back().pose.position.z, yaw);
    // }

    ViewPoint start_point;
    start_point.x = this->current_location.translation.x;
    start_point.y = this->current_location.translation.y;
    start_point.z = this->current_location.translation.z;

    tf2::Quaternion q;
    tf2::fromMsg(this->current_location.rotation, q);
    q.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    start_point.orientation = this->current_location.rotation;
    start_point.orientation_set = true;
    start_point.orientation_yaw = yaw;

    // Convert start_loc to Point as starting point?

    // Replace the generate_straight_rand_path with the viewpoint score and RRT?

    float timeout_duration = 5.0;
    std::optional<Path> gen_path_opt =
        // this->exploration_planner->generate_straight_rand_path(start_loc, timeout_duration);
        this->exploration_planner->select_viewpoint_and_plan(start_point, timeout_duration);
    if (gen_path_opt.has_value() && gen_path_opt.value().size() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Generated path with %ld points",
                    gen_path_opt.value().size());

        // set the current goal location
        this->current_goal_location = geometry_msgs::msg::Transform();
        this->current_goal_location.translation.x = std::get<0>(gen_path_opt.value().back());
        this->current_goal_location.translation.y = std::get<1>(gen_path_opt.value().back());
        this->current_goal_location.translation.z = std::get<2>(gen_path_opt.value().back());
        float z_rot = std::get<3>(gen_path_opt.value().back());
        tf2::Quaternion q;
        q.setRPY(0, 0, z_rot); // Roll = 0, Pitch = 0, Yaw = yaw
        q.normalize();
        this->current_goal_location.rotation.x = q.x();
        this->current_goal_location.rotation.y = q.y();
        this->current_goal_location.rotation.z = q.z();
        this->current_goal_location.rotation.w = q.w();

        // publish the path
        nav_msgs::msg::Path generated_single_path;
        generated_single_path = nav_msgs::msg::Path();
        generated_single_path.header.stamp = this->now();
        generated_single_path.header.frame_id = world_frame_id_;
        for (auto point : gen_path_opt.value())
        {
            geometry_msgs::msg::PoseStamped point_msg;
            point_msg.pose.position.x = std::get<0>(point);
            point_msg.pose.position.y = std::get<1>(point);
            point_msg.pose.position.z = std::get<2>(point);
            // convert yaw rotation to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, std::get<3>(point)); // Roll = 0, Pitch = 0, Yaw = yaw
            q.normalize();
            point_msg.pose.orientation.x = q.x();
            point_msg.pose.orientation.y = q.y();
            point_msg.pose.orientation.z = q.z();
            point_msg.pose.orientation.w = q.w();
            point_msg.header.stamp = this->now();
            generated_single_path.poses.push_back(point_msg);
        }
        geometry_msgs::msg::PoseStamped last_goal_loc = generated_single_path.poses.back();
        this->current_goal_location.translation.x = last_goal_loc.pose.position.x;
        this->current_goal_location.translation.y = last_goal_loc.pose.position.y;
        this->current_goal_location.translation.z = last_goal_loc.pose.position.z;
        this->current_goal_location.rotation.z = last_goal_loc.pose.orientation.z;
        this->generated_paths.push_back(generated_single_path);

        // debug vis of RRT
        PointSet coarse_path = this->exploration_planner->rrt_planner_.getCoarsePath();
        visualization_msgs::msg::MarkerArray rrt_vis_array;

        if (coarse_path.empty())
        {
            planning_debug_vis->publish(rrt_vis_array);
        }
        else
        {
            const std::string frame_id = "map";
            const rclcpp::Time stamp = this->now();

            // ----- LINE_STRIP for the polyline -----
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id;
            line.header.stamp = stamp;
            line.ns = "rrt_coarse_path";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0; // identity
            line.scale.x = 0.03;           // line width (m)
            line.color.r = 0.1f;
            line.color.g = 0.5f;
            line.color.b = 1.0f;
            line.color.a = 0.9f;
            line.lifetime = rclcpp::Duration(0, 0); // forever

            line.points.reserve(coarse_path.size());
            for (const auto &v : coarse_path)
            {
                geometry_msgs::msg::Point p;
                p.x = v.x;
                p.y = v.y;
                p.z = v.z;
                line.points.push_back(p);
            }
            rrt_vis_array.markers.push_back(line);

            // ----- SPHERE_LIST for vertices -----
            visualization_msgs::msg::Marker verts;
            verts.header.frame_id = frame_id;
            verts.header.stamp = stamp;
            verts.ns = "rrt_coarse_path";
            verts.id = 1;
            verts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            verts.action = visualization_msgs::msg::Marker::ADD;
            verts.pose.orientation.w = 1.0;
            verts.scale.x = 0.10;
            verts.scale.y = 0.10;
            verts.scale.z = 0.10; // sphere diameter (m)
            verts.color.r = 1.0f;
            verts.color.g = 0.2f;
            verts.color.b = 0.2f;
            verts.color.a = 0.9f;
            verts.lifetime = rclcpp::Duration(0, 0);

            verts.points.reserve(coarse_path.size());
            for (const auto &v : coarse_path)
            {
                geometry_msgs::msg::Point p;
                p.x = v.x;
                p.y = v.y;
                p.z = v.z;
                verts.points.push_back(p);
            }
            rrt_vis_array.markers.push_back(verts);

            // publish
            planning_debug_vis->publish(rrt_vis_array);
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
    }
}

void ExplorationNode::publish_plan()
{
    nav_msgs::msg::Path full_path;
    for (auto path : this->generated_paths)
    {
        for (auto point : path.poses)
        {
            full_path.poses.push_back(point);
        }
    }
    full_path.header.stamp = this->now();
    full_path.header.frame_id = this->world_frame_id_;
    this->pub_global_plan->publish(full_path);
    RCLCPP_INFO(this->get_logger(), "Published full path");

    if (!full_path.poses.empty())
    {
        this->pub_goal_posestamped->publish(full_path.poses.back());
    }
}

void ExplorationNode::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "timer callback start");
    // get current TF to world
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped =
            this->tf_buffer->lookupTransform(this->world_frame_id_, this->robot_frame_id_,
                                             rclcpp::Time(0));
        this->current_location = transform_stamped.transform;
        if (!this->received_first_robot_tf)
        {
            this->received_first_robot_tf = true;
            this->last_location = this->current_location;
            this->last_position_change = this->now();
            RCLCPP_WARN(this->get_logger(), "Received first robot_tf");
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Robot tf not received: %s", ex.what());
    }

    // Maintain the trajectory
    ViewPoint vp;
    vp.x = this->current_location.translation.x;
    vp.y = this->current_location.translation.y;
    vp.z = this->current_location.translation.z;

    vp.orientation.x = this->current_location.rotation.x;
    vp.orientation.y = this->current_location.rotation.y;
    vp.orientation.z = this->current_location.rotation.z;
    vp.orientation.w = this->current_location.rotation.w;

    vp.orientation_set = true;

    double roll, pitch, yaw;
    tf2::Quaternion tf2_q;
    tf2::fromMsg(vp.orientation, tf2_q); // geometry_msgs -> tf2::Quaternion
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    vp.orientation_yaw = yaw;

    this->exploration_planner->executed_trajectory_.push_back(vp);
    this->exploration_planner->momentum_executed_trajectory_.push_back(TimedViewPoint{vp, this->now()});

    // Prune old momentum samples (keep last momentum_time_ seconds)
    const rclcpp::Time now = this->now();
    const rclcpp::Duration keep = rclcpp::Duration::from_seconds(this->momentum_time_);

    auto &traj = this->exploration_planner->momentum_executed_trajectory_;
    while (!traj.empty() && (traj.front().time + keep) < now)
    {
        traj.erase(traj.begin());
        RCLCPP_INFO_ONCE(this->get_logger(), "Trim momentum traj");
    }

    // Update frontier map

    openvdb::Vec3d xyz;
    openvdb::Coord ijk;
    openvdb::FloatGrid::Accessor prev_acc = this->exploration_planner->prev_grid->getAccessor();
    openvdb::FloatGrid::Accessor lidar_acc = this->exploration_planner->occ_grid->getAccessor();
    openvdb::BoolGrid::Accessor frontier_acc = this->exploration_planner->frontier_grid->getAccessor();

    for (auto iter = this->exploration_planner->frontier_grid->cbeginValueOn(); iter; ++iter)
    {
        ijk = iter.getCoord();
        // if the surrounding voxels are still frontier
        bool still_front = surface_edge_frontier(lidar_acc, ijk);
        if (!still_front)
        {
            frontier_acc.setValueOff(ijk, false);
        }
    }
    // go through current free voxels
    for (auto iter = this->exploration_planner->occ_grid->cbeginValueOn(); iter; ++iter)
    {
        ijk = iter.getCoord();
        openvdb::Vec3d xyz = this->exploration_planner->occ_grid->indexToWorld(ijk);

        // if (xyz.x() > explore_x_min_ &&
        //     xyz.x() < explore_x_max_ &&
        //     xyz.y() > explore_y_min_ &&
        //     xyz.y() < explore_y_max_ &&
        //     xyz.z() > explore_z_min_ &&
        //     xyz.z() < explore_z_max_)
        {
            if (lidar_acc.getValue(ijk) < 0.0)
            { // new free voxels
                if (prev_acc.getValue(ijk) >= 0.0)
                {
                    bool is_front = surface_edge_frontier(lidar_acc, ijk);
                    if (is_front)
                    {
                        frontier_acc.setValueOn(ijk, true);
                    }
                }
            }
        }
    }

    // update previous grid
    this->exploration_planner->prev_grid = this->exploration_planner->occ_grid->deepCopy();

    // clustering
    openvdb::FloatGrid::Ptr visualized_cluster_grid_;
    visualized_cluster_grid_ = openvdb::FloatGrid::create(0.0);
    visualized_cluster_grid_->setTransform(this->exploration_planner->occ_grid->transform().copy());

    std::vector<std::vector<Eigen::Vector3d>> clustered_points_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_centroids_(new pcl::PointCloud<pcl::PointXYZI>);

    clustered_points_.clear();
    cluster_centroids_->clear();

    point_clustering_vis(this->exploration_planner->frontier_grid,
                         this->cluster_cube_dim,
                         cluster_centroids_,
                         this->voxel_cluster_count_thresh,
                         clustered_points_,
                         visualized_cluster_grid_);

    // visualize
    visualization_msgs::msg::Marker vdb_marker_msg;
    generateVDBMarker(this->exploration_planner->occ_grid, map_frame_id_, vdb_marker_msg);
    vdb_marker_msg.header.stamp = this->now();
    this->pub_vdb->publish(vdb_marker_msg);

    visualization_msgs::msg::Marker frontier_marker_msg;
    generateFrontierMarker(this->exploration_planner->frontier_grid, map_frame_id_, frontier_marker_msg);
    frontier_marker_msg.header.stamp = this->now();
    this->pub_frontier_vis->publish(frontier_marker_msg);

    visualization_msgs::msg::Marker clustered_marker_msg;
    generateClusterMarker(visualized_cluster_grid_, map_frame_id_, clustered_marker_msg);
    this->pub_clustered_frontier_vis->publish(clustered_marker_msg);

    // viewpoint sampling
    this->exploration_planner->viewp_sample_->reset(clustered_points_);
    this->exploration_planner->viewp_sample_->updateGridPtr(this->exploration_planner->occ_grid);
    this->exploration_planner->viewp_sample_->updateCentroidListPtr(cluster_centroids_);
    this->exploration_planner->viewp_sample_->sampleViewpoints();

    RCLCPP_WARN(this->get_logger(), "Viewpoints Sampled %zu", this->exploration_planner->viewp_sample_->viewpoint_list_->size());

    if (this->enable_exploration)
    {
        if (!this->is_path_executing)
        {
            if (this->exploration_planner->viewp_sample_->viewpoint_list_->size() > 0 && this->received_first_robot_tf)
            {

                this->generate_plan();

                this->publish_plan();
                this->is_path_executing = true;
                this->last_position_change = this->now(); // Reset stall timer when starting new plan
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "viewpoint list size is %zu", this->exploration_planner->viewp_sample_->viewpoint_list_->size());
                RCLCPP_INFO(this->get_logger(), "tf received is %d", this->received_first_robot_tf);
            }
        }
        else
        {
            // check if the robot has reached the goal point
            std::tuple<float, float, float> current_point = std::make_tuple(
                this->current_location.translation.x, this->current_location.translation.y,
                this->current_location.translation.z);
            std::tuple<float, float, float> goal_point = std::make_tuple(
                this->current_goal_location.translation.x, this->current_goal_location.translation.y,
                this->current_goal_location.translation.z);
            if (get_point_distance(current_point, goal_point) <
                this->exploration_planner->path_end_threshold_m)
            {
                this->is_path_executing = false;
                this->generated_paths.clear();
                RCLCPP_INFO(this->get_logger(), "Reached goal point");
            }
            else
            {
                // Check if position has changed significantly
                std::tuple<float, float, float> last_point = std::make_tuple(this->last_location.translation.x,
                                                                             this->last_location.translation.y,
                                                                             this->last_location.translation.z);

                if (get_point_distance(current_point, last_point) > this->position_change_threshold)
                {
                    this->last_location = this->current_location;
                    this->last_position_change = this->now();
                }
                else
                {
                    // Check if we've been stationary for too long
                    rclcpp::Duration stall_duration = this->now() - this->last_position_change;
                    if (stall_duration.seconds() > this->stall_timeout_seconds)
                    {
                        RCLCPP_INFO(this->get_logger(), "Robot stationary for %f seconds, clearing plan",
                                    stall_duration.seconds());
                        this->is_path_executing = false;
                        this->generated_paths.clear();
                        this->last_position_change = this->now(); // Reset timer to avoid spam
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "timer callback end");
}