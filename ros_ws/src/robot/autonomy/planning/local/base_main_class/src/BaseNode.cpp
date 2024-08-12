#include <ros/ros.h>

#include "base/BaseNode.h"


BaseNode::BaseNode(std::string node_name, bool anonymous)
  : Base(node_name)
  , anonymous_(anonymous){

  // call ros init
  if(anonymous_)
    ros::init(argc, argv, node_name_, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  else
    ros::init(argc, argv, node_name_, ros::init_options::NoSigintHandler);

  // set up the node handles
  node_handle_ = new ros::NodeHandle();
  private_node_handle_ = new ros::NodeHandle("~");
}

BaseNode::~BaseNode(){

}

int BaseNode::argc = 0;
char** BaseNode::argv = NULL;
