#include <pluginlib/class_loader.hpp>
#include <robot_interface/robot_interface.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_interface_node");
  
  pluginlib::ClassLoader<robot_interface::RobotInterface> loader("robot_interface", "robot_interface::RobotInterface");
  
  try{
    std::shared_ptr<robot_interface::RobotInterface> ri = loader.createSharedInstance("px4_interface::PX4Interface");
    // ask the flight controller to give us (the autonomy stack) control
    ri->request_control();
  }
  catch(pluginlib::PluginlibException& ex){
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }
  
  return 0;
}
