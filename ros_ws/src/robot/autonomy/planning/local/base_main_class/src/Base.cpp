#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "base/Base.h"
#include "base/HealthMonitor.h"
#include <iostream>
#include <string>

Base::Base(std::string node_name)
  : node_name_(node_name)
  , monitor(this)
  , is_nodelet_(false)
  , failed_(false){

}

Base::~Base(){
  monitor.print_time_statistics();
}

bool Base::_initialize(){
  // get the rate that execute should be called at
  double execute_timer_rate;
  bool found_param = get_private_node_handle()->getParam("execute_target", execute_timer_rate);
  if(!found_param){
    ROS_FATAL_STREAM("The execute_target parameter was not set. Exiting.");
    return false;
  }

  if (execute_timer_rate <= 0.0){
    ROS_FATAL_STREAM("The execute_target parameter must be larger than 0.0, Exiting.");
    return false;
  }

  // call the derived class's initialize
  bool status = initialize();
  if(!status){
    ROS_FATAL_STREAM("The initialize() function failed. Exiting.");
    return false;
  }

  // setup the execute timer
  execute_timer = get_node_handle()->createTimer(ros::Duration(1./execute_timer_rate),
						 &Base::execute_timer_callback, this);

  return true;
}

void Base::execute_timer_callback(const ros::TimerEvent& te){
  bool status = _execute();

  if(!status){
    execute_timer.stop();

    if(!is_nodelet_)
      ros::shutdown();
  }
}

bool Base::_execute(){
  monitor.tic("execute");
  bool status = execute();
  monitor.toc("execute");

  return status && !failed_;
}


void Base::fail(std::string reason){
  ROS_FATAL_STREAM("Node " << get_node_name() << " has failed." << std::endl
		   << "Reason: " << reason);

  failed_ = true;
}

std::string Base::get_node_name(){
  return ros::this_node::getName();
}


ros::NodeHandle* Base::get_node_handle(){
  return node_handle_;
}

ros::NodeHandle* Base::get_private_node_handle(){
  return private_node_handle_;
}
