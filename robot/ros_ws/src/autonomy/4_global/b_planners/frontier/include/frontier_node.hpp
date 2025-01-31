#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "frontier_logic.hpp"
#include "rclcpp/rclcpp.hpp"

class FrontierNode : public rclcpp::Node{
    private:
    std::unique_ptr<FrontierPlanner> frontier_planner;

    std::string world_frame_id_;
    std::string robot_frame_id_;
    std::string pub_global_plan_topic_;
    std::string pub_goal_point_viz_topic_;
    std::string pub_trajectory_viz_topic_;
    std::string sub_map_topic_;
    std::string sub_robot_tf_topic_;
    std::string srv_frontier_toggle_topic_;

    init_params params;
    std::vector<nav_msgs::msg::Path> generated_paths;

    int num_paths_to_generate_;
    bool enable_frontier = false;
    bool is_path_executing = false;
    bool received_first_map = false;
    bool received_first_robot_tf = false;

    bool publish_visualizations = false;

    geometry_msgs::msg::Transform current_location;
    geometry_msgs::msg::Transform current_goal_location;

    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void frontierToggleCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                  std_srvs::srv::Trigger::Response::SharedPtr response);
    void timerCallback();

    void generate_plan();
    void publish_plan();

    std::optional<init_params> readParameters();

    public:
    FrontierNode();
    ~FrontierNode() = default;

    //TF Buffer
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    
    //ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    
    //ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_frontier_toggle;
    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};