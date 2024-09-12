
#ifndef RANDOM_WALK_NODE_H
#define RANDOM_WALK_NODE_H

#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/point.hpp"

#include "random_walk_logic.hpp"
#include "rclcpp/rclcpp.hpp"

class RandomWalkNode : public rclcpp::Node {
   private:

    // Planner
    RandomWalkPlanner random_walk_planner;

    // String constants
    std::string world_frame_id_;
    std::string pub_global_path_topic_;
    std::string pub_goal_point_topic_;
    std::string pub_trajectory_lines_topic_;
    std::string sub_vdb_map_topic_;
    std::string sub_odometry_topic_;

    // Variables
    init_params params;
    airstack_msgs::msg::TrajectoryXYZVYaw generated_path;
    std::vector<std::tuple<float, float, float>> voxel_points;
    bool publish_visualizations = false;
    bool is_path_executing = false;
    bool received_first_map = false;
    bool received_first_odometry = false;
    std::tuple<float, float, float, float> current_location;       // x, y, z, yaw
    std::tuple<float, float, float, float> current_goal_location;  // x, y, z, yaw

    // Callbacks
    void vdbmapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Other functions
    std::optional<init_params> readParameters();

    visualization_msgs::msg::Marker createGoalPointMarker();
    visualization_msgs::msg::Marker createTrajectoryLineMarker();

   public:
    RandomWalkNode();
    ~RandomWalkNode() = default;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_vdb_map;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;

    // ROS publishers
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr pub_global_path;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};

#endif  // RANDOM_WALK_NODE_H
