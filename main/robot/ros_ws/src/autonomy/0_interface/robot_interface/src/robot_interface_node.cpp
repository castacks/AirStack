/**
 * @file robot_interface_node.cpp
 * @author John Keller (jkeller2@andrew.cmu.edu), Andrew Jong
 * (ajong@andrew.cmu.edu)
 * @brief main function for the robot interface node
 * @version 0.1
 * @date 2024-07-01
 *
 * @copyright Copyright (c) 2024. This file is developed as part of software
 * from the AirLab at the Robotics Institute at Carnegie Mellon University
 * (https://theairlab.org).
 *
 */
#include <airstack_common/ros2_helper.hpp>
#include <airstack_msgs/srv/robot_command.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_interface/robot_interface.hpp>
#include <std_msgs/msg/bool.hpp>

std::shared_ptr<robot_interface::RobotInterface> ri;

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_control_pub;

void robot_command_callback(
    const std::shared_ptr<airstack_msgs::srv::RobotCommand::Request> request,
    std::shared_ptr<airstack_msgs::srv::RobotCommand::Response> response) {
    // response->sum = request->a + request->b;
    switch (request->command) {
        case airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL:
            response->success = ri->request_control();
            break;
        case airstack_msgs::srv::RobotCommand::Request::ARM:
            response->success = ri->arm();
            break;
        case airstack_msgs::srv::RobotCommand::Request::DISARM:
            response->success = ri->disarm();
            break;
        case airstack_msgs::srv::RobotCommand::Request::TAKEOFF:
            response->success = ri->takeoff();
            break;
        case airstack_msgs::srv::RobotCommand::Request::LAND:
            response->success = ri->land();
            break;
            /*
          case airstack_msgs::srv::RobotCommand::Request::SET_LOW_THRUST_MODE:
            drone_interface->set_low_thrust_mode(true);
            response->success = true;
            break;
          case airstack_msgs::srv::RobotCommand::Request::UNSET_LOW_THRUST_MODE:
            drone_interface->set_low_thrust_mode(false);
            response->success = true;
            break;
            */
    }
}

void attitude_thrust_callback(mav_msgs::msg::AttitudeThrust::SharedPtr msg) {
    ri->attitude_thrust_callback(msg);
}

void rate_thrust_callback(mav_msgs::msg::RateThrust::SharedPtr msg) {
    ri->rate_thrust_callback(msg);
}

void roll_pitch_yawrate_thrust_callback(mav_msgs::msg::RollPitchYawrateThrust::SharedPtr msg) {
    ri->roll_pitch_yawrate_thrust_callback(msg);
}

void torque_thrust_callback(mav_msgs::msg::TorqueThrust::SharedPtr msg) {
    ri->torque_thrust_callback(msg);
}

void velocity_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    ri->velocity_callback(msg);
}

void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) { ri->pose_callback(msg); }

void timer_callback() {
    std_msgs::msg::Bool is_armed;
    is_armed.data = ri->is_armed();
    is_armed_pub->publish(is_armed);

    std_msgs::msg::Bool has_control;
    has_control.data = ri->has_control();
    has_control_pub->publish(has_control);
}

rclcpp::Subscription<mav_msgs::msg::AttitudeThrust>::SharedPtr attitude_thrust_sub;
rclcpp::Subscription<mav_msgs::msg::RateThrust>::SharedPtr rate_thrust_sub;
rclcpp::Subscription<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr
    roll_pitch_yawrate_thrust_sub;
rclcpp::Subscription<mav_msgs::msg::TorqueThrust>::SharedPtr torque_thrust_sub;
rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // load the interface parameter
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_interface_node");
    std::string interface =
        airstack::get_param(node, "interface", std::string("mavros_interface::MAVROSInterface"));
    node.reset();

    pluginlib::ClassLoader<robot_interface::RobotInterface> loader(
        "robot_interface", "robot_interface::RobotInterface");

    try {
        ri = loader.createSharedInstance("mavros_interface::MAVROSInterface");

        // subscribers
        attitude_thrust_sub = ri->create_subscription<mav_msgs::msg::AttitudeThrust>(
            "attitude_thrust_command", 1, attitude_thrust_callback);
        rate_thrust_sub = ri->create_subscription<mav_msgs::msg::RateThrust>(
            "rate_thrust_command", 1, rate_thrust_callback);
        roll_pitch_yawrate_thrust_sub =
            ri->create_subscription<mav_msgs::msg::RollPitchYawrateThrust>(
                "roll_pitch_yawrate_thrust_command", 1, roll_pitch_yawrate_thrust_callback);
        torque_thrust_sub = ri->create_subscription<mav_msgs::msg::TorqueThrust>(
            "torque_thrust_command", 1, torque_thrust_callback);
        velocity_sub = ri->create_subscription<geometry_msgs::msg::TwistStamped>(
            "velocity_command", 1, velocity_callback);
        pose_sub = ri->create_subscription<geometry_msgs::msg::PoseStamped>("pose_command", 1,
                                                                            pose_callback);

        // publishers
        is_armed_pub = ri->create_publisher<std_msgs::msg::Bool>("is_armed", 1);
        has_control_pub = ri->create_publisher<std_msgs::msg::Bool>("has_control", 1);

        // services
        rclcpp::Service<airstack_msgs::srv::RobotCommand>::SharedPtr service =
            ri->create_service<airstack_msgs::srv::RobotCommand>("robot_command",
                                                                 &robot_command_callback);

        // timers
        rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(
            ri, ri->get_clock(), rclcpp::Duration::from_seconds(1. / 20.), &timer_callback);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(ri);
        executor.spin();
        // rclcpp::spin(ri);
        rclcpp::shutdown();
    } catch (pluginlib::PluginlibException& ex) {
        std::cout << "The plugin failed to load. Error: " << ex.what() << std::endl;
    }

    return 0;
}
