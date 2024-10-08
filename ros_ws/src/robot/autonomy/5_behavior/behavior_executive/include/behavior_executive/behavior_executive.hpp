#include <behavior_tree/behavior_tree.h>
#include <airstack_msgs/srv/robot_command.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <airstack_msgs/srv/takeoff_landing_command.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <nav_msgs/msg/path.hpp>
#include <behavior_tree_msgs/msg/behavior_tree_commands.hpp>
#include <global_planner_msgs/action/get_random_walk_plan.hpp>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"

class BehaviorExecutive : public rclcpp::Node {
private:
  // parameters
  bool ascent_takeoff;
  
  // Condition variables
  bt::Condition* auto_takeoff_commanded_condition;
  bt::Condition* takeoff_commanded_condition;
  bt::Condition* armed_condition;
  bt::Condition* offboard_mode_condition;
  bt::Condition* stationary_condition;
  bt::Condition* land_commanded_condition;
  bt::Condition* pause_commanded_condition;
  bt::Condition* rewind_commanded_condition;
  bt::Condition* fixed_trajectory_condition;
  bt::Condition* random_walk_condition;
  bt::Condition* disable_random_walk_condition;
  bt::Condition* offboard_commanded_condition;
  bt::Condition* arm_commanded_condition;
  bt::Condition* disarm_commanded_condition;
  std::vector<bt::Condition*> conditions;

  // Action variables
  bt::Action* arm_action;
  bt::Action* takeoff_action;
  bt::Action* land_action;
  bt::Action* pause_action;
  bt::Action* rewind_action;
  bt::Action* follow_fixed_trajectory_action;
  bt::Action* random_walk_action;
  bt::Action* request_control_action;
  bt::Action* disarm_action;
  std::vector<bt::Action*> actions;
  
  // subscribers
  rclcpp::Subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>::SharedPtr behavior_tree_commands_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr has_control_sub;
  
  // publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub;

  // services
  rclcpp::CallbackGroup::SharedPtr service_callback_group;
  rclcpp::Client<airstack_msgs::srv::RobotCommand>::SharedPtr robot_command_client;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr trajectory_mode_client;
  rclcpp::Client<airstack_msgs::srv::TakeoffLandingCommand>::SharedPtr takeoff_landing_command_client;
  rclcpp_action::Client<global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr random_walk_mode_client;
  
  // timers
  rclcpp::TimerBase::SharedPtr timer;

  // callbacks
  void timer_callback();
  void bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg);
  void is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void has_control_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // action goal callbacks
  void goal_response_callback(
      std::shared_ptr<
          rclcpp_action::ClientGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
          goal_handle);
          
  void send_random_walk_goal(
      rclcpp_action::Client<global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr
          random_walk_mode_client,
      global_planner_msgs::action::GetRandomWalkPlan::Goal goal);

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr,
      const std::shared_ptr<const global_planner_msgs::action::GetRandomWalkPlan::Feedback> feedback);

  void result_callback(const rclcpp_action::ClientGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>::WrappedResult & result);

 public:
  BehaviorExecutive();
  
};
