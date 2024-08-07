#ifndef _TFLIB_H_
#define _TFLIB_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace tflib{
  
  // ==========================================================================
  // -------------------------------- Conversion ------------------------------
  // ==========================================================================
  
  tf::Quaternion to_tf(geometry_msgs::Quaternion q);
  tf::Vector3 to_tf(geometry_msgs::Point p);
  tf::Vector3 to_tf(geometry_msgs::Vector3 v);
  tf::StampedTransform to_tf(nav_msgs::Odometry odom, std::string frame);
  
  // ==========================================================================
  // --------------------------------- Utils ----------------------------------
  // ==========================================================================
  
  tf::Quaternion get_stabilized(tf::Quaternion q);
  tf::Transform get_stabilized(tf::Transform transform);
  tf::StampedTransform get_stabilized(tf::StampedTransform transform);
  bool transform_odometry(tf::TransformListener* listener, nav_msgs::Odometry odom,
			  std::string new_frame_id, std::string new_child_frame_id, nav_msgs::Odometry* out_odom);
  // This can throw any exception that lookupTransform throws
  nav_msgs::Odometry transform_odometry(tf::TransformListener* listener, nav_msgs::Odometry odom,
					std::string new_frame_id, std::string new_child_frame_id, ros::Duration polling_sleep_duration);
  
  // ==========================================================================
  // ------------------------------ Transforms --------------------------------
  // ==========================================================================
  
  bool to_frame(tf::TransformListener* listener,
		tf::Vector3 vec, std::string source, std::string target, ros::Time stamp,
		tf::Vector3* out, ros::Duration duration=ros::Duration(0.1));
}

#endif
