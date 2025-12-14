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
// #include <mavros_msgs/msg/position_target.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <mavros_msgs/msg/position_target.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <airstack_common/ros2_helper.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <robot_interface/robot_interface.hpp>

#include "GeographicLib/Geoid.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include "det/YOLO11.hpp"

#include <mutex>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fstream>
#include <iomanip> // for std::setprecision
#include <cmath>

#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"

class BehaviorExecutive : public rclcpp::Node {
   private:
    // parameters
    bool ascent_takeoff;
    rclcpp::Time last_execute_time;
    nav_msgs::msg::Odometry odom;
    int image_count_;

    std::string modelPath;
    std::string labelsPath;
    bool isGPU;
    YOLO11Detector detector;

    cv::Mat latest_depth_;
    std::mutex depth_mutex_;

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
    bt::Condition* image_based_visual_servo_condition;
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
    bt::Action* ibvs_action;
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

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recording_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_stuck_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

    // services
    rclcpp::CallbackGroup::SharedPtr service_callback_group;
    rclcpp::Client<airstack_msgs::srv::RobotCommand>::SharedPtr robot_command_client;
    rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr trajectory_mode_client;
    rclcpp::Client<airstack_msgs::srv::TakeoffLandingCommand>::SharedPtr
        takeoff_landing_command_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr global_planner_toggle_client;

    // timers
    rclcpp::TimerBase::SharedPtr timer;

    double fx_;
    double fy_;
    double cx_;
    double cy_;

    double tx_;
    double ty_;
    double tz_;

    // last detection
    double det_x_ = 0.0;
    double det_y_ = 0.0;
    double det_z_ = 1.0;

    // timer
    rclcpp::TimerBase::SharedPtr ibvs_timer_;
    bool ibvs_running_ = false;

    // callbacks
    void timer_callback();
    void bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg);
    void is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void has_control_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void takeoff_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void landing_state_callback(const std_msgs::msg::String::SharedPtr msg);
    void state_estimate_timed_out_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void stuck_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publish_velocity_ibvs(double vx, double vy, double vz);
    Eigen::Vector3d camera_to_body(double cx, double cy, double cz);
    void ibvs_control(double x, double y, double z);

   public:
    BehaviorExecutive();
}; 
