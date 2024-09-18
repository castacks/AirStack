
#ifndef RANDOM_WALK_NODE_H
#define RANDOM_WALK_NODE_H

#include <cmath>
#include <nav_msgs/msg/Path.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.msg>
#include <tf2_msgs/msg/TFMessage.hpp>

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
    std::string sub_map_topic_;
    std::string sub_robot_tf_topic;

    // Variables
    init_params params;
    nav_msgs::msg::Path generated_path;
    std::vector<std::tuple<float, float, float>> voxel_points;
    bool publish_visualizations = false;
    bool is_path_executing = false;
    bool received_first_map = false;
    bool received_first_odometry = false;
    geometry_msgs::msg::Transform current_location;       // x, y, z, yaw
    geometry_msgs::msg::Transform current_goal_location;  // x, y, z, yaw

    // Callbacks
    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    // void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Other functions
    std::optional<init_params> readParameters();

    visualization_msgs::msg::Marker createGoalPointMarker();
    visualization_msgs::msg::Marker createTrajectoryLineMarker();

   public:
    RandomWalkNode();
    ~RandomWalkNode() = default;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_tf;

    // ROS publishers
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr pub_global_path;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};

#endif  // RANDOM_WALK_NODE_H
