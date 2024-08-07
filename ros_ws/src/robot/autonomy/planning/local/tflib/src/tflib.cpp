#include <tflib/tflib.h>

namespace tflib {

  // ==========================================================================
  // -------------------------------- Conversion ------------------------------
  // ==========================================================================
  
  tf::Quaternion to_tf(geometry_msgs::Quaternion q){
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    return quat;
  }
  
  tf::Vector3 to_tf(geometry_msgs::Point p){
    tf::Vector3 v(p.x, p.y, p.z);
    return v;
  }
  
  
  tf::Vector3 to_tf(geometry_msgs::Vector3 v){
    tf::Vector3 vec(v.x, v.y, v.z);
    return vec;
  }

  
  tf::StampedTransform to_tf(nav_msgs::Odometry odom, std::string frame){
    tf::Transform transform;
    transform.setOrigin(to_tf(odom.pose.pose.position));
    transform.setRotation(to_tf(odom.pose.pose.orientation));
    
    tf::StampedTransform stamped_transform(transform, odom.header.stamp, odom.header.frame_id, frame);
    return stamped_transform;
  }

  // ==========================================================================
  // --------------------------------- Utils ----------------------------------
  // ==========================================================================

  tf::Quaternion get_stabilized(tf::Quaternion q){
    tf::Quaternion stabilized_q;
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    stabilized_q.setRPY(0, 0, yaw);
    return stabilized_q;
  }

  tf::Transform get_stabilized(tf::Transform transform){
    tf::Transform stabilized_transform;
    stabilized_transform.setOrigin(transform.getOrigin());
    stabilized_transform.setRotation(get_stabilized(transform.getRotation()));
    return stabilized_transform;
  }

  tf::StampedTransform get_stabilized(tf::StampedTransform transform){
    tf::StampedTransform stabilized_transform(transform);
    stabilized_transform.setRotation(get_stabilized(transform.getRotation()));
    return stabilized_transform;
  }
  
  bool transform_odometry(tf::TransformListener* listener, nav_msgs::Odometry odom,
			  std::string new_frame_id, std::string new_child_frame_id, nav_msgs::Odometry* out_odom){
    try{
      *out_odom = transform_odometry(listener, odom, new_frame_id, new_child_frame_id, ros::Duration(0.1));
    }
    catch(tf::TransformException& ex){
      ROS_ERROR_STREAM("Transform exception in transform_odometry: " << ex.what());
      return false;
    }

    return true;
  }

  // This can throw any exception that lookupTransform throws
  nav_msgs::Odometry transform_odometry(tf::TransformListener* listener, nav_msgs::Odometry odom,
					std::string new_frame_id, std::string new_child_frame_id, ros::Duration polling_sleep_duration){
    nav_msgs::Odometry out_odom;
    tf::StampedTransform transform_frame_id, transform_child_frame_id;
    listener->waitForTransform(new_frame_id, odom.header.frame_id, odom.header.stamp, polling_sleep_duration);
    listener->lookupTransform(new_frame_id, odom.header.frame_id, odom.header.stamp, transform_frame_id);
    listener->waitForTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp, polling_sleep_duration);
    listener->lookupTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp, transform_child_frame_id);
    transform_child_frame_id.setOrigin(tf::Vector3(0, 0, 0)); // don't want translation when transforming linear and angular velocities

    out_odom.header.stamp = odom.header.stamp;
    out_odom.header.frame_id = new_frame_id;
    out_odom.child_frame_id = new_child_frame_id;
      
    tf::Vector3 position = transform_frame_id*tflib::to_tf(odom.pose.pose.position);
    out_odom.pose.pose.position.x = position.x();
    out_odom.pose.pose.position.y = position.y();
    out_odom.pose.pose.position.z = position.z();

    tf::Quaternion quaternion = transform_frame_id*tflib::to_tf(odom.pose.pose.orientation);
    out_odom.pose.pose.orientation.x = quaternion.x();
    out_odom.pose.pose.orientation.y = quaternion.y();
    out_odom.pose.pose.orientation.z = quaternion.z();
    out_odom.pose.pose.orientation.w = quaternion.w();

    tf::Vector3 velocity = transform_child_frame_id*tflib::to_tf(odom.twist.twist.linear);
    out_odom.twist.twist.linear.x = velocity.x();
    out_odom.twist.twist.linear.y = velocity.y();
    out_odom.twist.twist.linear.z = velocity.z();
      
    tf::Vector3 angular = transform_child_frame_id*tflib::to_tf(odom.twist.twist.angular);
    out_odom.twist.twist.angular.x = angular.x();
    out_odom.twist.twist.angular.y = angular.y();
    out_odom.twist.twist.angular.z = angular.z();

    return out_odom;
  }

  
  // ==========================================================================
  // ------------------------------ Transforms --------------------------------
  // ==========================================================================
  
  bool to_frame(tf::TransformListener* listener,
		tf::Vector3 vec, std::string source, std::string target, ros::Time stamp,
		tf::Vector3* out, ros::Duration duration){
    if(out == NULL)
      return false;
    
    try{
      tf::StampedTransform transform;
      listener->waitForTransform(target, source, stamp, duration);
      listener->lookupTransform(target, source, stamp, transform);

      *out = transform*vec;
    }
    catch(tf::TransformException& ex){
      ROS_ERROR_STREAM("TransformException in to_frame: " << ex.what());
      return false;
    }

    return true;
  }
  
}
