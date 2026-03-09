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

#include <airstack_common/ros2_helper.hpp>
#include <airstack_msgs/srv/robot_command.hpp>
#include <airstack_msgs/srv/takeoff_landing_command.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <behavior_tree/behavior_tree.hpp>
#include <behavior_tree_msgs/msg/behavior_tree_commands.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"

class BehaviorExecutive : public rclcpp::Node {
   private:
    // parameters
    bool ascent_takeoff;

    // variables
    std::string takeoff_state, landing_state;

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
    bt::Condition* global_plan_condition;
    bt::Condition* offboard_commanded_condition;
    bt::Condition* arm_commanded_condition;
    bt::Condition* disarm_commanded_condition;
    bt::Condition* takeoff_complete_condition;
    bt::Condition* landing_complete_condition;
    bt::Condition* in_air_condition;
    bt::Condition* state_estimate_timed_out_condition;
    bt::Condition* stuck_condition;
    bt::Condition* autonomously_explore_condition;
    std::vector<bt::Condition*> conditions;

    // Action variables
    bt::Action* arm_action;
    bt::Action* takeoff_action;
    bt::Action* land_action;
    bt::Action* pause_action;
    bt::Action* rewind_action;
    bt::Action* follow_fixed_trajectory_action;
    bt::Action* global_plan_action;
    bt::Action* request_control_action;
    bt::Action* disarm_action;
    std::vector<bt::Action*> actions;

    // subscribers
    rclcpp::Subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>::SharedPtr
        behavior_tree_commands_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr has_control_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr takeoff_state_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr landing_state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_estimate_timed_out_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stuck_sub;

    // publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recording_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_stuck_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_map_pub;

    // services
    rclcpp::CallbackGroup::SharedPtr service_callback_group;
    rclcpp::Client<airstack_msgs::srv::RobotCommand>::SharedPtr robot_command_client;
    rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr trajectory_mode_client;
    rclcpp::Client<airstack_msgs::srv::TakeoffLandingCommand>::SharedPtr
        takeoff_landing_command_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr global_planner_toggle_client;

    // timers
    rclcpp::TimerBase::SharedPtr timer;

    // callbacks
    void timer_callback();
    void bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg);
    void is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void has_control_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void takeoff_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void landing_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void state_estimate_timed_out_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void stuck_callback(const std_msgs::msg::Bool::SharedPtr msg);

   public:
    BehaviorExecutive();
}; 
