#include <behavior_tree/behavior_tree.h>
#include <airstack_msgs/srv/robot_command.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <airstack_msgs/srv/takeoff_landing_command.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <behavior_tree_msgs/msg/behavior_tree_commands.hpp>
#include <vector>

class BehaviorExecutive : public rclcpp::Node {
private:

  // parameters
  bool ascent_takeoff;
  
  // Condition variables
  bt::Condition* takeoff_commanded_condition;
  bt::Condition* armed_condition;
  bt::Condition* offboard_mode_condition;
  bt::Condition* stationary_condition;
  bt::Condition* land_commanded_condition;
  bt::Condition* pause_commanded_condition;
  bt::Condition* rewind_commanded_condition;
  bt::Condition* fixed_trajectory_condition;
  bt::Condition* explore_condition;
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
  bt::Action* explore_action;
  bt::Action* request_control_action;
  bt::Action* disarm_action;
  std::vector<bt::Action*> actions;
  
  // subscribers
  rclcpp::Subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>::SharedPtr behavior_tree_commands_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr has_control_sub;
  
  
  // publishers

  // services
  rclcpp::CallbackGroup::SharedPtr service_callback_group;
  rclcpp::Client<airstack_msgs::srv::RobotCommand>::SharedPtr robot_command_client;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr trajectory_mode_client;
  rclcpp::Client<airstack_msgs::srv::TakeoffLandingCommand>::SharedPtr takeoff_landing_command_client;
  
  // timers
  rclcpp::TimerBase::SharedPtr timer;

  // callbacks
  void timer_callback();
  void bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg);
  void is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void has_control_callback(const std_msgs::msg::Bool::SharedPtr msg);
  
public:
  BehaviorExecutive();
  
};
