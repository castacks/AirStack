#ifndef _BASE_H_
#define _BASE_H_

#include <ros/ros.h>
#include <string>
#include "HealthMonitor.h"

class Base{
 private:
  bool failed_;
  ros::Timer execute_timer;

 public:
  HealthMonitor monitor;

  // These functions will be called by main, do health monitoring
  // and call the dervied funtions below
  bool _initialize();
  bool _execute();
  virtual ~Base();

  void execute_timer_callback(const ros::TimerEvent& te);

  // These functions are implemented by the user
  virtual bool initialize() = 0;
  virtual bool execute() = 0;

  // Failing will stop execution and deinitialize
  void fail(std::string reason);

  // Returns the name the user gave to the node/nodelet
  std::string get_node_name();

  // Get node handles.
  ros::NodeHandle* get_node_handle();
  ros::NodeHandle* get_private_node_handle();

 protected:
  Base(std::string node_name);

  ros::NodeHandle* node_handle_;
  ros::NodeHandle* private_node_handle_;

  std::string node_name_;

  // If this is true, the pure virtual deinitialize() function will not be
  // called in _deinitialize(), since you can only catch the end of
  // a nodelet by implementing its destructor.
  bool is_nodelet_;
};


#endif
