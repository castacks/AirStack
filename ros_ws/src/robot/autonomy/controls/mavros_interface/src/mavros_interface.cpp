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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <robot_interface/robot_interface.hpp>

namespace mavros_interface {

class MAVROSInterface : public robot_interface::RobotInterface {
   private:
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    // service client for arming/disarming the vehicle
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;

    // publisher for attitude target messages
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr
        attitude_target_pub;

    // publisher for position target messages
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr
        local_position_target_pub;

    // subscriber for state messages
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;

    // subscriber for pixhawk pose messages
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        fcu_pose_sub;

    bool is_state_received;
    mavros_msgs::msg::State current_state;

    // data from the flight control unit (FCU)
    bool is_fcu_yaw_received;
    float fcu_yaw;

   public:
    MAVROSInterface() : RobotInterface("mavros_interface") {
        is_state_received = false;

        set_mode_client =
            this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client = this->create_client<mavros_msgs::srv::CommandBool>(
            "mavros/cmd/arming");

        // publishers
        // https://wiki.ros.org/mavros#mavros.2FPlugins.setpoint_attitude:~:text=TF%20listener%20%5BHz%5D.-,setpoint_raw,-Send%20RAW%20setpoint
        attitude_target_pub =
            this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
                "mavros/setpoint_raw/attitude", 1);
        local_position_target_pub =
            this->create_publisher<mavros_msgs::msg::PositionTarget>(
                "mavros/setpoint_raw/local", 1);

        // subscribers
        state_sub = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 1,
            std::bind(&MAVROSInterface::state_callback, this,
                      std::placeholders::_1));
        fcu_pose_sub =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "mavros/local_position/pose", 1,
                std::bind(&MAVROSInterface::fcu_pose_callback, this,
                          std::placeholders::_1));
    }

    virtual ~MAVROSInterface() {}

    // Control Callbacks. Translates desired commands to fit the MAVROS API.
    // The MAVROS API only has two types of control: Attitude Control and
    // Position Control.

    // Attitude Controls

    void attitude_thrust_callback(
        mav_msgs::msg::AttitudeThrust desired_cmd) override {
        mavros_msgs::msg::AttitudeTarget mavros_cmd;
        mavros_cmd.header.stamp = this->get_clock()->now();  //.to_msg();
        mavros_cmd.type_mask =
            mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
            mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
            mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

        mavros_cmd.thrust = desired_cmd.thrust.z;
        mavros_cmd.orientation = desired_cmd.attitude;

        attitude_target_pub->publish(mavros_cmd);
    }

    void roll_pitch_yawrate_thrust_callback(
        mav_msgs::msg::RollPitchYawrateThrust desired_cmd) override {
        if (!this->is_fcu_yaw_received) {
            RCLCPP_WARN_STREAM_ONCE(
                this->get_logger(),
                "roll_pitch_yawrate_thrust command called but haven't yet "
                "received drone current yaw");
            return;
        }

        mavros_msgs::msg::AttitudeTarget mavros_cmd;
        mavros_cmd.header.stamp = this->get_clock()->now();  //.to_msg();
        mavros_cmd.type_mask =
            mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
            mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;
        tf2::Matrix3x3 m;
        m.setRPY(desired_cmd.roll, desired_cmd.pitch, this->fcu_yaw);
        tf2::Quaternion q;
        m.getRotation(q);
        mavros_cmd.body_rate.z = desired_cmd.yaw_rate;
        mavros_cmd.thrust = desired_cmd.thrust.z;

        mavros_cmd.orientation.x = q.x();
        mavros_cmd.orientation.y = q.y();
        mavros_cmd.orientation.z = q.z();
        mavros_cmd.orientation.w = q.w();

        attitude_target_pub->publish(mavros_cmd);
    }

    // Position Controls

    void pose_callback(geometry_msgs::msg::PoseStamped desired_cmd) override {
        mavros_msgs::msg::PositionTarget pub_msg;
        // by having no type_mask, we are commanding velocity and acceleration
        // to be zero. this will cause the drone to stop at the desired
        // position. pub_msg.type_mask =
        // mavros_msgs::msg::PositionTarget::IGNORE_VX |
        // mavros_msgs::msg::PositionTarget::IGNORE_VY |
        // mavros_msgs::msg::PositionTarget::IGNORE_VZ |
        // mavros_msgs::msg::PositionTarget::IGNORE_AFX |
        // mavros_msgs::msg::PositionTarget::IGNORE_AFY |
        // mavros_msgs::msg::PositionTarget::IGNORE_AFZ;

        // copy over position
        pub_msg.position = desired_cmd.pose.position;

        // extract only yaw
        tf2::Quaternion q(
            desired_cmd.pose.orientation.x, desired_cmd.pose.orientation.y,
            desired_cmd.pose.orientation.z, desired_cmd.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        pub_msg.yaw = yaw;

        local_position_target_pub->publish(pub_msg);
    }

    // Command Functions

    bool request_control() override {
        bool success = false;

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";

        auto result = set_mode_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                               result) ==
            rclcpp::FutureReturnCode::SUCCESS)
            success = true;

        return success;
    }

    bool arm() override {
        bool success = false;

        auto request =
            std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto result = arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                               result) ==
            rclcpp::FutureReturnCode::SUCCESS)
            success = true;

        return success;
    }

    bool disarm() override {
        bool success = false;

        auto request =
            std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto result = arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                               result) ==
            rclcpp::FutureReturnCode::SUCCESS)
            success = true;

        return success;
    }

    bool is_armed() override {
        return is_state_received && current_state.armed;
    }

    bool has_control() override {
        return is_state_received && current_state.mode == "OFFBOARD";
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        is_state_received = true;
        current_state = *msg;
    }

    // Subscriber Callbacks

    /**
     * @brief Get the current Yaw from the Pixhawk. This is used to provide the
     * current yaw to the PitchRollYawrateThrust command, so that the drone
     * doesn't try to set its yaw to zero.
     *
     * @param pose
     */
    void fcu_pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        tf2::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y,
                          pose->pose.orientation.z, pose->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        this->is_fcu_yaw_received = true;
        this->fcu_yaw = yaw;
    }
};

}  // namespace mavros_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mavros_interface::MAVROSInterface,
                       robot_interface::RobotInterface)
