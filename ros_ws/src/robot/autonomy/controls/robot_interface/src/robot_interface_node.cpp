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
#include <pluginlib/class_loader.hpp>
#include <robot_interface/robot_interface.hpp>

/**
 * @brief Launches the specific robot interface for the desired flight
 * controller and requests control for the autonomy stack.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<robot_interface::RobotInterface> loader(
        "robot_interface", "robot_interface::RobotInterface");
    std::shared_ptr<robot_interface::RobotInterface> ri_node;

    try {
        ri_node = loader.createSharedInstance("mavros_interface::MAVROSInterface");
    } catch (pluginlib::PluginlibException &ex) {
        rclcpp::Node::SharedPtr logger_node =
            rclcpp::Node::make_shared("robot_interface_node_logger");
        RCLCPP_ERROR(logger_node->get_logger(),
                     "The plugin failed to load for some reason. Error: %s\n", ex.what());
        return 1;
    }
    RCLCPP_INFO(ri_node->get_logger(), "Successfully loaded the robot interface plugin.");

    // ask the flight controller to give us (the autonomy stack) control
    ri_node->request_control();
    rclcpp::spin(ri_node);
    rclcpp::shutdown();

    return 0;
}
