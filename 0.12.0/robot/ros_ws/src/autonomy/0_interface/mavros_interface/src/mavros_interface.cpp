/**
 * @file mavros_interface.cpp
 * @author John Keller (jkeller2@andrew.cmu.edu), Andrew Jong
 * (ajong@andrew.cmu.edu)
 * @brief overrides the RobotInterface class to implement the PX4 flight control
 * interface.
 * @version 0.1
 * @date 2024-07-01
 *
 * @copyright Copyright (c) 2024. This file is developed as part of software
 * from the AirLab at the Robotics Institute at Carnegie Mellon University
 * (https://theairlab.org).
 *
 */
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <airstack_common/ros2_helper.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <robot_interface/robot_interface.hpp>

namespace mavros_interface {

class MAVROSInterface : public robot_interface::RobotInterface {
   private:
    // parameters
    bool is_ardupilot;  // TODO make this a launch file parameter

    bool is_state_received_ = false;
    mavros_msgs::msg::State current_state_;
    bool in_air = false;
    rclcpp::Time in_air_start_time;

    // data from the flight control unit (FCU)
    bool is_yaw_received_ = false;
    float yaw_ = 0.0;

    rclcpp::CallbackGroup::SharedPtr service_callback_group;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_target_pub_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odometry_sub_;

   public:
    MAVROSInterface() : RobotInterface("mavros_interface") {
        // params
        is_ardupilot = airstack::get_param(this, "is_ardupilot", false);

        // services
        service_callback_group =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "mavros/set_mode", rmw_qos_profile_services_default, service_callback_group);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "mavros/cmd/arming", rmw_qos_profile_services_default, service_callback_group);
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
            "mavros/cmd/takeoff", rmw_qos_profile_services_default, service_callback_group);

        // publishers
        attitude_target_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "mavros/setpoint_raw/attitude", 1);
        local_position_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 1);

        // subscribers
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 1,
            std::bind(&MAVROSInterface::state_callback, this, std::placeholders::_1));
    }

    virtual ~MAVROSInterface() {}

    // Control Callbacks. Translates commands to fit the MAVROS API.
    // The MAVROS API only has two types of control: Attitude Control and
    // Position Control.

    void attitude_thrust_callback(const mav_msgs::msg::AttitudeThrust::SharedPtr cmd) override {
        mavros_msgs::msg::AttitudeTarget mavros_cmd;
        mavros_cmd.header.stamp = this->get_clock()->now();  //.to_msg();
        mavros_cmd.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                               mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                               mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

        mavros_cmd.thrust = cmd->thrust.z;
        mavros_cmd.orientation = cmd->attitude;

        attitude_target_pub_->publish(mavros_cmd);
    }

    void roll_pitch_yawrate_thrust_callback(
        const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr cmd) override {
        if (!is_yaw_received_) {
            RCLCPP_ERROR(this->get_logger(),
                         "roll_pitch_yawrate_thrust command called but haven't yet "
                         "received drone current yaw");
            return;
        }

        mavros_msgs::msg::AttitudeTarget mavros_cmd;
        mavros_cmd.header.stamp = this->get_clock()->now();  //.to_msg();
        mavros_cmd.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                               mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;
        tf2::Matrix3x3 m;
        m.setRPY(cmd->roll, cmd->pitch, yaw_);
        tf2::Quaternion q;
        m.getRotation(q);
        mavros_cmd.body_rate.z = cmd->yaw_rate;
        mavros_cmd.thrust = cmd->thrust.z;

        mavros_cmd.orientation.x = q.x();
        mavros_cmd.orientation.y = q.y();
        mavros_cmd.orientation.z = q.z();
        mavros_cmd.orientation.w = q.w();

        attitude_target_pub_->publish(mavros_cmd);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr cmd) override {
        if (!is_ardupilot ||
            (in_air && ((this->get_clock()->now() - in_air_start_time).seconds() > 5.))) {
            geometry_msgs::msg::PoseStamped cmd_copy = *cmd;
            local_position_target_pub_->publish(cmd_copy);
        }
    }

    // Command Functions

    bool request_control() override {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        if (is_ardupilot)
            request->custom_mode = "GUIDED";  //"OFFBOARD";
        else
            request->custom_mode = "OFFBOARD";

        auto result = set_mode_client_->async_send_request(request);
        std::cout << "waiting rc" << std::endl;
        result.wait();
        std::cout << "done rc" << std::endl;

        return result.get()->mode_sent;
    }

    bool arm() override {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto result = arming_client_->async_send_request(request);
        std::cout << "waiting arm" << std::endl;
        result.wait();
        std::cout << "done arm" << std::endl;

        return result.get()->success;
    }

    bool disarm() override {
        bool success = false;

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto result = arming_client_->async_send_request(request);
        std::cout << "waiting disarm" << std::endl;
        result.wait();
        std::cout << "done disarm" << std::endl;

        return result.get()->success;
    }

    bool is_armed() override { return is_state_received_ && current_state_.armed; }

    bool has_control() override {
        return is_state_received_ &&
               (is_ardupilot ? current_state_.mode == "GUIDED" : current_state_.mode == "OFFBOARD");
    }

    bool takeoff() override {
        if (is_ardupilot) {
            mavros_msgs::srv::CommandTOL::Request::SharedPtr takeoff_request =
                std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            takeoff_request->altitude = 0.1;

            std::cout << "ardupilot takeoff 1" << std::endl;
            auto takeoff_result = takeoff_client_->async_send_request(takeoff_request);
            takeoff_result.wait();
            std::cout << "ardupilot takeoff 2" << std::endl;
            if (takeoff_result.get()->success) {
                in_air = true;
                in_air_start_time = this->get_clock()->now();
                return true;
            } else
                return false;
        }

        return true;
    }

    bool land() override {}

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        is_state_received_ = true;
        current_state_ = *msg;
    }
};
}  // namespace mavros_interface
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mavros_interface::MAVROSInterface, robot_interface::RobotInterface)
