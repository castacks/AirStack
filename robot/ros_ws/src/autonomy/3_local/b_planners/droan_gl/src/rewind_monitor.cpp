#include <droan_gl/rewind_monitor.hpp>

RewindMonitor::RewindMonitor(rclcpp::Node* node)
  : node(node){
  
}

void RewindMonitor::update_odom(const airstack_msgs::msg::Odometry::SharedPtr msg){
  positions.push_back(std::pair<tf2::Vector3, rclcpp::Time>(tf2::Vector3(msg->pose.position.x,
									 msg->pose.position.y,
									 msg->pose.position.z),
							    msg->header.stamp));
}

void RewindMonitor::do_stationary_check(bool b){
  
}

void RewindMonitor::found_trajectory(bool b){
  
}

void RewindMonitor::trigger_rewind(float seconds, float distance=-1.f){
  
}


bool RewindMonitor::should_rewind(){
  
}
