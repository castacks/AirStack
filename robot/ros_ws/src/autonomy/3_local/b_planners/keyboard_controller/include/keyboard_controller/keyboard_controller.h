#pragma once

#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/service.hpp"
#include "string"
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <airstack_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/msg/waypoint_xyzv_yaw.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

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

using namespace Eigen;

extern int kfd;
extern struct termios cooked;
extern struct termios raw;
void quit(int sig);

class KeyboardController : public rclcpp::Node{
	private:
		airstack_msgs::msg::Odometry des_pose_;
		geometry_msgs::msg::PoseStamped des_pose_vis_;
		nav_msgs::msg::Odometry current_pose_;
		std::string robot_name_;
		std::string world_frame_id_;
		double increment_step_xyz_{0.02};
		double increment_step_yaw_{3.1415926535/30.0};
		rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr des_traj_publisher_;
		rclcpp::Publisher<airstack_msgs::msg::Odometry>::SharedPtr des_pose_publisher_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr des_pose_vis_publisher_;
  		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_pose_subscriber_;
  		rclcpp::TimerBase::SharedPtr timer_;
		bool MoveMAV(const double &dx, const double &dy, const double &dz, const double &dyaw);
		Isometry3d GetEigenFromTf2(const tf2::Transform &t);
		uint8_t current_command_;
		void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
		bool is_drone_pose_received_{false};
		bool is_keyboard_received_{false};
		bool is_enabled_{false};  // (Yunwoo) for GUI control
		void timer_callback();
		
		// (Yunwoo) Topic-based keyboard input for GUI integration
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_input_sub_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr keyboard_enable_sub_;
		void KeyboardInputCallback(const std_msgs::msg::String::SharedPtr msg);
		void KeyboardEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
		void ProcessKey(char c);
		
	public:
		KeyboardController();
		void KeyLoop();
		void Publish();
};


