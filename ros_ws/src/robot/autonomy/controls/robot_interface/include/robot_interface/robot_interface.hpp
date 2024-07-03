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
 */
class RobotInterface : public rclcpp::Node {
   protected:
    RobotInterface(std::string interface_name) : Node(interface_name) {}

   public:
    // TODO add low thrust mode

    // Control callbacks. // TODO maybe make parameters const references?
    virtual void attitude_thrust_callback(
        mav_msgs::msg::AttitudeThrust desired_cmd) {}
    virtual void rate_thrust_callback(mav_msgs::msg::RateThrust desired_cmd) {}
    virtual void roll_pitch_yawrate_thrust_callback(
        mav_msgs::msg::RollPitchYawrateThrust desired_cmd) {}
    virtual void torque_thrust_callback(
        mav_msgs::msg::TorqueThrust desired_cmd) {}
    virtual void velocity_callback(
        geometry_msgs::msg::TwistStamped desired_cmd) {}
    virtual void pose_callback(geometry_msgs::msg::PoseStamped desired_cmd) {}

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

    virtual ~RobotInterface() {}
};
}  // namespace robot_interface