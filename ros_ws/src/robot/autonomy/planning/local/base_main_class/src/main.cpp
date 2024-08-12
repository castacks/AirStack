#include "base/BaseNode.h"
#include <ros/ros.h>
#include <signal.h>

BaseNode* base_node;

void interrupt_handler(int sig){
  delete base_node;
  ros::shutdown();
}

int main(int argc, char** argv){
  BaseNode::argc = argc;
  BaseNode::argv = argv;
  base_node = BaseNode::get();

  signal(SIGINT, interrupt_handler);

  if(base_node->_initialize())
    ros::spin();
  else
    ROS_FATAL_STREAM("Node initialization failed. Exiting.");

  return 0;
}
