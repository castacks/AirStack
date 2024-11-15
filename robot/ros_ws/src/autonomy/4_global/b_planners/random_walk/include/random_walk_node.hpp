
#ifndef RANDOM_WALK_NODE_H
#define RANDOM_WALK_NODE_H

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <optional>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "random_walk_logic.hpp"
#include "rclcpp/rclcpp.hpp"

class RandomWalkNode : public rclcpp::Node {
   private:
    // Planner
    // RandomWalkPlanner random_walk_planner;
    std::unique_ptr<RandomWalkPlanner> random_walk_planner;

    // String constants
    std::string world_frame_id_;
    std::string robot_frame_id_;
    std::string pub_global_plan_topic_;
    std::string pub_goal_point_viz_topic_;
    std::string pub_trajectory_viz_topic_;
    std::string sub_map_topic_;
    std::string sub_robot_tf_topic_;
    std::string srv_random_walk_toggle_topic_;

    // Variables
    init_params params;
    // nav_msgs::msg::Path generated_path;
    int num_paths_to_generate_;
    std::vector<nav_msgs::msg::Path> generated_paths;
    bool publish_visualizations = false;
    bool received_first_map = false;
    bool received_first_robot_tf = false;
    bool enable_random_walk = false;
    bool is_path_executing = false;

    geometry_msgs::msg::Transform current_location;       // x, y, z, yaw
    geometry_msgs::msg::Transform current_goal_location;  // x, y, z, yaw

    // Callbacks
    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void randomWalkToggleCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                  std_srvs::srv::Trigger::Response::SharedPtr response);

    void timerCallback();

    void generate_plan();

    void publish_plan();

    // Other functions
    std::optional<init_params> readParameters();

   public:
    // explicit RandomWalkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    RandomWalkNode();
    ~RandomWalkNode() = default;

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_robot_tf;

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_random_walk_toggle;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};

#endif  // RANDOM_WALK_NODE_H
