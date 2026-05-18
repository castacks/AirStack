#pragma once

#include <string>
#include <termios.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/msg/waypoint_xyzv_yaw.hpp>

#define KEYBOARD_O 0x6f
#define KEYBOARD_P 0x70
#define KEYBOARD_K 0x6b
#define KEYBOARD_L 0x6c
#define KEYBOARD_W 0x77
#define KEYBOARD_S 0x73
#define KEYBOARD_A 0x61
#define KEYBOARD_D 0x64
#define KEYBOARD_Z 0x7A
#define KEYBOARD_C 0x63
#define KEYBOARD_Q 0x71
#define KEYBOARD_E 0x65

extern int kfd;
extern struct termios cooked;
extern struct termios raw;
void quit(int sig);

class KeyboardController : public rclcpp::Node {
public:
    KeyboardController();
    void KeyLoop();

private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void KeyboardInputCallback(const std_msgs::msg::String::SharedPtr msg);
    void KeyboardEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void ProcessKey(char c);
    bool MoveMAV(const double& dx, const double& dy, const double& dz, const double& dyaw);
    Eigen::Isometry3d GetEigenFromTf2(const tf2::Transform& t);

    airstack_msgs::msg::Odometry des_pose_;
    geometry_msgs::msg::PoseStamped des_pose_vis_;
    nav_msgs::msg::Odometry current_pose_;

    double increment_step_xyz_{0.5};
    double increment_step_yaw_{M_PI / 12.0};

    bool is_drone_pose_received_{false};
    bool is_keyboard_received_{false};
    bool is_enabled_{false};

    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr des_traj_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr des_pose_vis_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_input_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr keyboard_enable_sub_;
    rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_client_;
    bool set_trajectory_mode(int32_t mode);
};
