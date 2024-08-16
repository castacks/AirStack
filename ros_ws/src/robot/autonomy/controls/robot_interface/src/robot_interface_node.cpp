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

#include <airstack_msgs/srv/robot_command.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <std_msgs/msg/bool.hpp>

std::shared_ptr<robot_interface::RobotInterface> ri;

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_control_pub;

void robot_command_callback(const std::shared_ptr<airstack_msgs::srv::RobotCommand::Request> request,
			    std::shared_ptr<airstack_msgs::srv::RobotCommand::Response> response){
  //response->sum = request->a + request->b;
  switch(request->command){
  case airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL:
    response->success = ri->request_control();
    break;
  case airstack_msgs::srv::RobotCommand::Request::ARM:
    response->success = ri->arm();
    break;
  case airstack_msgs::srv::RobotCommand::Request::DISARM:
    response->success = ri->disarm();
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

void timer_callback(){
  std_msgs::msg::Bool is_armed;
  is_armed.data = ri->is_armed();
  is_armed_pub->publish(is_armed);

  std_msgs::msg::Bool has_control;
  has_control.data = ri->has_control();
  has_control_pub->publish(has_control);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // load the interface parameter
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_interface_node");
  std::string interface = airstack::get_param(node, "interface", std::string("mavros_interface::MAVROSInterface"));
  node.reset();
  
  pluginlib::ClassLoader<robot_interface::RobotInterface> loader("robot_interface", "robot_interface::RobotInterface");
  
  try{
    ri = loader.createSharedInstance("mavros_interface::MAVROSInterface");

    // publishers
    is_armed_pub = ri->create_publisher<std_msgs::msg::Bool>("is_armed", 1);
    has_control_pub = ri->create_publisher<std_msgs::msg::Bool>("has_control", 1);
    
    // services
    rclcpp::Service<airstack_msgs::srv::RobotCommand>::SharedPtr service =
      ri->create_service<airstack_msgs::srv::RobotCommand>("robot_command", &robot_command_callback);

    // timers
    rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(ri, ri->get_clock(),
							      rclcpp::Duration::from_seconds(1./20.), &timer_callback);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ri);
    executor.spin();
    //rclcpp::spin(ri);
    rclcpp::shutdown();
  }
  catch(pluginlib::PluginlibException& ex){
    std::cout << "The plugin failed to load. Error: " << ex.what() << std::endl;
  }
  
  return 0;
}
