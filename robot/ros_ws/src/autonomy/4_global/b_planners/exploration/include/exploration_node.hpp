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

#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <message_filters/subscriber.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <optional>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "exploration_logic.hpp"
#include "utils/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <openvdb/io/Stream.h>

class ExplorationNode : public rclcpp::Node
{
protected:
    // Planner
    // ExplorationPlanner exploration_planner;
    std::unique_ptr<ExplorationPlanner> exploration_planner;

    // String constants
    std::string world_frame_id_;
    std::string robot_frame_id_;
    std::string lidar_frame_id_;
    std::string map_frame_id_;
    std::string pub_global_plan_topic_;
    std::string pub_goal_point_viz_topic_;
    std::string pub_trajectory_viz_topic_;
    std::string pub_grid_viz_topic_;
    std::string pub_frontier_viz_topic_;
    std::string pub_clustered_frontier_viz_topic_;
    std::string sub_lidar_topic_;
    std::string sub_map_topic_;
    std::string sub_vdb_topic_;
    std::string sub_robot_tf_topic_;
    std::string srv_exploration_toggle_topic_;

    // Variables
    init_params params;
    // nav_msgs::msg::Path generated_path;
    double momentum_time_;
    int num_paths_to_generate_;
    std::vector<nav_msgs::msg::Path> generated_paths;
    bool publish_visualizations = false;
    bool received_first_map = false;
    bool received_first_robot_tf = false;
    bool enable_exploration = false;
    bool is_path_executing = false;

    geometry_msgs::msg::Transform current_location;      // x, y, z, yaw
    geometry_msgs::msg::Transform current_goal_location; // x, y, z, yaw
    geometry_msgs::msg::Transform last_location;         // Last recorded position
    rclcpp::Time last_position_change;                   // Time of last position change
    double position_change_threshold = 0.1;              // Minimum distance (meters) to consider as movement
    double stall_timeout_seconds = 5.0;                  // Time without movement before clearing plan

    double cluster_cube_dim;
    int voxel_cluster_count_thresh;

    double map_voxel_resolution; // map voxel grid size

    // Callbacks
    // void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void vdbCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

    void ExplorationToggleCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                   std_srvs::srv::Trigger::Response::SharedPtr response);

    virtual void timerCallback();

    virtual void generate_plan();

    void publish_plan();

    // Other functions
    std::optional<init_params> readParameters();

public:
    // explicit ExplorationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ExplorationNode();
    ~ExplorationNode() = default;

    virtual void initialize();
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_filter_sub_;

    // ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_vdb;
    // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_robot_tf;

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vdb;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_frontier_vis;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_clustered_frontier_vis;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planning_debug_vis;

    // ROS services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_exploration_toggle;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};
