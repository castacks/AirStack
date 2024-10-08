
#ifndef RANDOM_WALK_NODE_H
#define RANDOM_WALK_NODE_H

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <global_planner_msgs/action/get_random_walk_plan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <optional>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "random_walk_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
    std::string sub_robot_tf_topic_;
    std::string srv_get_plan_topic_;

    // Variables
    init_params params;
    nav_msgs::msg::Path generated_path;
    std::vector<std::tuple<float, float, float>> voxel_points;
    bool publish_visualizations = false;
    bool received_first_map = false;
    bool received_first_robot_tf = false;
    bool is_ready = false;
    bool plan_reception_enabled = false;

    geometry_msgs::msg::Transform current_location;       // x, y, z, yaw
    geometry_msgs::msg::Transform current_goal_location;  // x, y, z, yaw

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const global_planner_msgs::action::GetRandomWalkPlan::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
            goal_handle);

    void handle_accepted(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
            goal_handle);

    // Callbacks
    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void generate_plan(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
            goal_handle);

    void timerCallback();

    // Other functions
    std::optional<init_params> readParameters();

    visualization_msgs::msg::Marker createGoalPointMarker();
    visualization_msgs::msg::Marker createTrajectoryLineMarker();

   public:
    // explicit RandomWalkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    RandomWalkNode();
    ~RandomWalkNode() = default;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_robot_tf;

    // ROS publishers
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS services
    rclcpp_action::Server<global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr srv_get_plan;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};

#endif  // RANDOM_WALK_NODE_H
