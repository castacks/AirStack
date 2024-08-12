#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>

#include "base/BaseNodelet.h"


BaseNodelet::BaseNodelet(std::string node_name)
  : Base(node_name){
  is_nodelet_ = true;
}

BaseNodelet::~BaseNodelet(){
  
}

void BaseNodelet::onInit(){
  node_handle_ = &getNodeHandle();
  private_node_handle_ = &getPrivateNodeHandle();
  _initialize();
}
