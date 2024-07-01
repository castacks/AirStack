/**
 * @file px4_interface.cpp
 * @author John Keller (jkeller2@andrew.cmu.edu), Andrew Jong (ajong@andrew.cmu.edu)
 * @brief overrides the RobotInterface class to implement the PX4 flight control interface.
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024. This file is developed as part of software from the AirLab at the
 * Robotics Institute at Carnegie Mellon University (https://theairlab.org).
 * 
 */
#include <robot_interface/robot_interface.hpp>

#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace px4_interface
{

  class PX4Interface : public robot_interface::RobotInterface
  {
  private:
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    // service client for arming/disarming the vehicle
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;

    // publisher for attitude target messages
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub;

    // subscriber for state messages
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;

    // subscriber for pixhawk pose messages
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pixhawk_pose_sub;

    bool is_state_received;
    mavros_msgs::msg::State current_state;

    bool is_pixhawk_yaw_received;
    float pixhawk_yaw;

  public:
    PX4Interface() : RobotInterface("px4_interface")
    {
      is_state_received = false;

      set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
      arming_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

      attitude_target_pub = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);

      state_sub = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 1,
                                                                     std::bind(&PX4Interface::state_callback, this,
                                                                               std::placeholders::_1));
      pixhawk_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", 1,
                                                                                    std::bind(&PX4Interface::pixhawk_pose_callback,
                                                                                              this, std::placeholders::_1));
    }

    virtual ~PX4Interface()
    {
    }

    // Control Callbacks

    void roll_pitch_yawrate_thrust_callback(mav_msgs::msg::RollPitchYawrateThrust msg) override
    {
      if (!is_pixhawk_yaw_received)
        return;

      mavros_msgs::msg::AttitudeTarget att;
      att.header.stamp = this->get_clock()->now(); //.to_msg();
      att.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;
      tf2::Matrix3x3 m;
      m.setRPY(msg.roll,
               msg.pitch,
               pixhawk_yaw);
      tf2::Quaternion q;
      m.getRotation(q);
      att.body_rate.z = msg.yaw_rate;
      att.thrust = msg.thrust.z;

      att.orientation.x = q.x();
      att.orientation.y = q.y();
      att.orientation.z = q.z();
      att.orientation.w = q.w();

      attitude_target_pub->publish(att);
    }

    // Command Functions

    bool request_control() override
    {
      bool success = false;

      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->custom_mode = "OFFBOARD";

      auto result = set_mode_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        success = true;

      return success;
    }

    bool arm() override
    {
      bool success = false;

      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = true;

      auto result = arming_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        success = true;

      return success;
    }

    bool disarm() override
    {
      bool success = false;

      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = false;

      auto result = arming_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        success = true;

      return success;
    }

    bool is_armed() override
    {
      return is_state_received && current_state.armed;
    }

    bool has_control() override
    {
      return is_state_received && current_state.mode == "OFFBOARD";
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
      is_state_received = true;
      current_state = *msg;
    }


    // Subscriber Callbacks

    void pixhawk_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
      tf2::Quaternion q(pose->pose.orientation.x,
                        pose->pose.orientation.y,
                        pose->pose.orientation.z,
                        pose->pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      is_pixhawk_yaw_received = true;
      pixhawk_yaw = yaw;
    }
  };

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(px4_interface::PX4Interface, robot_interface::RobotInterface)
