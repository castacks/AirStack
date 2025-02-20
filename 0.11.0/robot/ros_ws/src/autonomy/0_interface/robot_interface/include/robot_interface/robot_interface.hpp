/**
 *
 * @file robot_interface.hpp
 * @author John Keller (jkeller2@andrew.cmu.edu), Andrew Jong
 * (ajong@andrew.cmu.edu)
 * @brief the Robot Interface specifies common MAV flight control functions for
 * individual implementations to define.
 * @version 0.1
 * @date 2024-07-01
 *
 * @copyright Copyright (c) 2024. This file is developed as part of software
 * from the AirLab at the Robotics Institute at Carnegie Mellon University
 * (https://theairlab.org).
 */

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/msg/rate_thrust.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <mav_msgs/msg/torque_thrust.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace robot_interface {
  /**
   * @brief The Robot Interface class specifies common MAV flight control
   * functions for individual implementations to define.
   *
   * Sets up subscribers for different types of control commands that call their respective
   * callback implementations. It's up to the individual implementations to define the
   * control logic.
   */
  class RobotInterface : public rclcpp::Node {
  private:
    // Command subscribers
    rclcpp::Subscription<mav_msgs::msg::AttitudeThrust>::SharedPtr attitude_thrust_sub_;
    rclcpp::Subscription<mav_msgs::msg::RateThrust>::SharedPtr rate_thrust_sub_;
    rclcpp::Subscription<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr roll_pitch_yawrate_thrust_sub_;
    rclcpp::Subscription<mav_msgs::msg::TorqueThrust>::SharedPtr torque_thrust_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  public:
    RobotInterface(std::string interface_name)
      : rclcpp::Node("robot_interface"){
      attitude_thrust_sub_ =
	this->create_subscription<mav_msgs::msg::AttitudeThrust>("cmd_attitude_thrust", 1,
								 std::bind(&RobotInterface::attitude_thrust_callback, this,
									   std::placeholders::_1));
      rate_thrust_sub_ =
	this->create_subscription<mav_msgs::msg::RateThrust>("cmd_rate_thrust", 1,
							     std::bind(&RobotInterface::rate_thrust_callback,
								       this, std::placeholders::_1));
      roll_pitch_yawrate_thrust_sub_ =
	this->create_subscription<mav_msgs::msg::RollPitchYawrateThrust>("cmd_roll_pitch_yawrate_thrust", 1,
									 std::bind(&RobotInterface::roll_pitch_yawrate_thrust_callback,
										   this, std::placeholders::_1));
      torque_thrust_sub_ =
	this->create_subscription<mav_msgs::msg::TorqueThrust>("cmd_torque_thrust", 1,
							       std::bind(&RobotInterface::torque_thrust_callback,
									 this, std::placeholders::_1));
      
      velocity_sub_ =
	this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_velocity", 1,
								    std::bind(&RobotInterface::velocity_callback,
									      this, std::placeholders::_1));
      
      pose_sub_ =
	this->create_subscription<geometry_msgs::msg::PoseStamped>("cmd_pose", 1,
								   std::bind(&RobotInterface::pose_callback,
									     this, std::placeholders::_1));
    }
  
    // Control callbacks.
    virtual void attitude_thrust_callback(const mav_msgs::msg::AttitudeThrust::SharedPtr cmd) {
      RCLCPP_ERROR(this->get_logger(), "Attitude thrust callback not implemented.");
    }
    virtual void rate_thrust_callback(const mav_msgs::msg::RateThrust::SharedPtr cmd) {
      RCLCPP_ERROR(this->get_logger(), "Rate thrust callback not implemented.");
    }
    virtual void roll_pitch_yawrate_thrust_callback(const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr cmd) {
      RCLCPP_ERROR(this->get_logger(), "Roll pitch yawrate thrust callback not implemented.");
    }
    virtual void torque_thrust_callback(const mav_msgs::msg::TorqueThrust::SharedPtr cmd) {
      RCLCPP_ERROR(this->get_logger(), "Torque thrust callback not implemented.");
    }
    virtual void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd) {
      RCLCPP_ERROR(this->get_logger(), "Velocity callback not implemented.");
    }
    virtual void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr cmd) {
      RCLCPP_ERROR(this->get_logger(), "Pose callback not implemented.");
    }

    // Command functions
    /**
     * @brief Call the service to request that the flight controller relinquish
     * control to the autonomy stack.
     *
     * @return true
     * @return false
     */
    virtual bool request_control() = 0;
    /**
     * @brief Set the flight controller to "armed"
     *
     * @return true
     * @return false
     */
    virtual bool arm() = 0;
    /**
     * @brief Set the flight controller to "disarmed"
     *
     * @return true
     * @return false
     */
    virtual bool disarm() = 0;
    /**
     * @brief Check if the flight controller is armed.
     *
     * @return true
     * @return false
     */
    virtual bool is_armed() = 0;
    /**
     * @brief Check if the autonomy stack has control of the flight controller.
     *
     * @return true
     * @return false
     */
    virtual bool has_control() = 0;

    virtual bool takeoff() = 0;
    virtual bool land() = 0;

  public:
    virtual ~RobotInterface() {}
  };
}  // namespace robot_interface
