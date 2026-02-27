#include <airstack_common/tflib.hpp>

namespace tflib {

  // ==========================================================================
  // -------------------------------- Conversion ------------------------------
  // ==========================================================================
  
  tf2::Quaternion to_tf(geometry_msgs::msg::Quaternion q){
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    return quat;
  }
  
  tf2::Vector3 to_tf(geometry_msgs::msg::Point p){
    tf2::Vector3 v(p.x, p.y, p.z);
    return v;
  }
  
  
  tf2::Vector3 to_tf(geometry_msgs::msg::Vector3 v){
    tf2::Vector3 vec(v.x, v.y, v.z);
    return vec;
  }

  
  tf2::Stamped<tf2::Transform> to_tf(nav_msgs::msg::Odometry odom){//, std::string frame){
    tf2::Transform transform;
    transform.setOrigin(to_tf(odom.pose.pose.position));
    transform.setRotation(to_tf(odom.pose.pose.orientation));

    tf2::Stamped<tf2::Transform> stamped_transform(transform, tf2_ros::fromMsg(odom.header.stamp), odom.header.frame_id);
    return stamped_transform;
  }
  
  tf2::Stamped<tf2::Transform> to_tf(airstack_msgs::msg::Odometry odom){//, std::string frame){
    tf2::Transform transform;
    transform.setOrigin(to_tf(odom.pose.position));
    transform.setRotation(to_tf(odom.pose.orientation));
    
    tf2::Stamped<tf2::Transform> stamped_transform(transform, tf2_ros::fromMsg(odom.header.stamp), odom.header.frame_id);
    return stamped_transform;
  }
  
  geometry_msgs::msg::Point from_tf(tf2::Vector3 v){
    geometry_msgs::msg::Point out;
    out.x = v.x();
    out.y = v.y();
    out.z = v.z();
    return out;
  }
  
  geometry_msgs::msg::Quaternion from_tf(tf2::Quaternion q){
    geometry_msgs::msg::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
  }

  // ==========================================================================
  // --------------------------------- Utils ----------------------------------
  // ==========================================================================
  
  /**
   * @brief Get the stabilized object by setting roll and pitch to 0
   * 
   * @param q 
   * @return tf2::Quaternion 
   */
  tf2::Quaternion get_stabilized(tf2::Quaternion q){
    tf2::Quaternion stabilized_q;
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    stabilized_q.setRPY(0, 0, yaw);
    return stabilized_q;
  }

  tf2::Transform get_stabilized(tf2::Transform transform){
    tf2::Transform stabilized_transform;
    stabilized_transform.setOrigin(transform.getOrigin());
    stabilized_transform.setRotation(get_stabilized(transform.getRotation()));
    return stabilized_transform;
  }

  tf2::Stamped<tf2::Transform> get_stabilized(tf2::Stamped<tf2::Transform> transform){
    tf2::Stamped<tf2::Transform> stabilized_transform(transform);
    stabilized_transform.setRotation(get_stabilized(transform.getRotation()));
    return stabilized_transform;
  }
  
  bool transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
			  std::string new_frame_id, std::string new_child_frame_id, nav_msgs::msg::Odometry* out_odom,
			  rclcpp::Duration polling_sleep_duration){
    try{
      *out_odom = transform_odometry(tf_buffer, odom, new_frame_id, new_child_frame_id, polling_sleep_duration);
    }
    catch(tf2::TransformException& ex){
      std::cout << "Transform exception in transform_odometry: " << ex.what();
      return false;
    }

    return true;
  }

  // This can throw any exception that lookupTransform throws
  nav_msgs::msg::Odometry transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
					     std::string new_frame_id, std::string new_child_frame_id,
					     rclcpp::Duration polling_sleep_duration){
    nav_msgs::msg::Odometry out_odom;
    tf2::Stamped<tf2::Transform> transform_frame_id, transform_child_frame_id;
    geometry_msgs::msg::TransformStamped t;

    t = tf_buffer->lookupTransform(new_frame_id, odom.header.frame_id,
				   rclcpp::Time(odom.header.stamp), polling_sleep_duration);
    tf2::fromMsg(t, transform_frame_id);
    t = tf_buffer->lookupTransform(new_child_frame_id, odom.child_frame_id,
				   rclcpp::Time(odom.header.stamp), polling_sleep_duration);
    tf2::fromMsg(t, transform_child_frame_id);
    
    //listener->waitForTransform(new_frame_id, odom.header.frame_id, odom.header.stamp, polling_sleep_duration);
    //listener->lookupTransform(new_frame_id, odom.header.frame_id, odom.header.stamp, transform_frame_id);
    //listener->waitForTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp, polling_sleep_duration);
    //listener->lookupTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp, transform_child_frame_id);
    transform_child_frame_id.setOrigin(tf2::Vector3(0, 0, 0)); // don't want translation when transforming linear and angular velocities
    
    out_odom.header.stamp = odom.header.stamp;
    out_odom.header.frame_id = new_frame_id;
    out_odom.child_frame_id = new_child_frame_id;
    
    tf2::Vector3 position = transform_frame_id*tflib::to_tf(odom.pose.pose.position);
    out_odom.pose.pose.position.x = position.x();
    out_odom.pose.pose.position.y = position.y();
    out_odom.pose.pose.position.z = position.z();

    tf2::Quaternion quaternion = transform_frame_id*tflib::to_tf(odom.pose.pose.orientation);
    out_odom.pose.pose.orientation.x = quaternion.x();
    out_odom.pose.pose.orientation.y = quaternion.y();
    out_odom.pose.pose.orientation.z = quaternion.z();
    out_odom.pose.pose.orientation.w = quaternion.w();

    tf2::Vector3 velocity = transform_child_frame_id*tflib::to_tf(odom.twist.twist.linear);
    out_odom.twist.twist.linear.x = velocity.x();
    out_odom.twist.twist.linear.y = velocity.y();
    out_odom.twist.twist.linear.z = velocity.z();
      
    tf2::Vector3 angular = transform_child_frame_id*tflib::to_tf(odom.twist.twist.angular);
    out_odom.twist.twist.angular.x = angular.x();
    out_odom.twist.twist.angular.y = angular.y();
    out_odom.twist.twist.angular.z = angular.z();

    return out_odom;
  }

  bool transform_odometry(tf2_ros::Buffer* tf_buffer, airstack_msgs::msg::Odometry odom,
			  std::string new_frame_id, std::string new_child_frame_id, airstack_msgs::msg::Odometry* out_odom,
			  rclcpp::Duration polling_sleep_duration){
    try{
      *out_odom = transform_odometry(tf_buffer, odom, new_frame_id, new_child_frame_id, polling_sleep_duration);
    }
    catch(tf2::TransformException& ex){
      std::cout << "Transform exception in transform_odometry: " << ex.what();
      return false;
    }
    
    return true;
  }

  // This can throw any exception that lookupTransform throws
  airstack_msgs::msg::Odometry transform_odometry(tf2_ros::Buffer* tf_buffer, airstack_msgs::msg::Odometry odom,
						  std::string new_frame_id, std::string new_child_frame_id,
						  rclcpp::Duration polling_sleep_duration){
    airstack_msgs::msg::Odometry out_odom;
    tf2::Stamped<tf2::Transform> transform_frame_id, transform_child_frame_id;
    geometry_msgs::msg::TransformStamped t;
    
    t = tf_buffer->lookupTransform(new_frame_id, odom.header.frame_id,
				   rclcpp::Time(odom.header.stamp), polling_sleep_duration);
    tf2::fromMsg(t, transform_frame_id);
    t = tf_buffer->lookupTransform(new_child_frame_id, odom.child_frame_id,
				   rclcpp::Time(odom.header.stamp), polling_sleep_duration);
    tf2::fromMsg(t, transform_child_frame_id);
    
    //listener->waitForTransform(new_frame_id, odom.header.frame_id, odom.header.stamp, polling_sleep_duration);
    //listener->lookupTransform(new_frame_id, odom.header.frame_id, odom.header.stamp, transform_frame_id);
    //listener->waitForTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp, polling_sleep_duration);
    //listener->lookupTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp, transform_child_frame_id);
    transform_child_frame_id.setOrigin(tf2::Vector3(0, 0, 0)); // don't want translation when transforming linear and angular velocities

    out_odom.header.stamp = odom.header.stamp;
    out_odom.header.frame_id = new_frame_id;
    out_odom.child_frame_id = new_child_frame_id;
      
    tf2::Vector3 position = transform_frame_id*tflib::to_tf(odom.pose.position);
    out_odom.pose.position.x = position.x();
    out_odom.pose.position.y = position.y();
    out_odom.pose.position.z = position.z();

    tf2::Quaternion quaternion = transform_frame_id*tflib::to_tf(odom.pose.orientation);
    out_odom.pose.orientation.x = quaternion.x();
    out_odom.pose.orientation.y = quaternion.y();
    out_odom.pose.orientation.z = quaternion.z();
    out_odom.pose.orientation.w = quaternion.w();

    tf2::Vector3 velocity = transform_child_frame_id*tflib::to_tf(odom.twist.linear);
    out_odom.twist.linear.x = velocity.x();
    out_odom.twist.linear.y = velocity.y();
    out_odom.twist.linear.z = velocity.z();
      
    tf2::Vector3 angular = transform_child_frame_id*tflib::to_tf(odom.twist.angular);
    out_odom.twist.angular.x = angular.x();
    out_odom.twist.angular.y = angular.y();
    out_odom.twist.angular.z = angular.z();
    
    tf2::Vector3 acceleration = transform_child_frame_id*tflib::to_tf(odom.acceleration);
    out_odom.acceleration.x = acceleration.x();
    out_odom.acceleration.y = acceleration.y();
    out_odom.acceleration.z = acceleration.z();

    tf2::Vector3 jerk = transform_child_frame_id*tflib::to_tf(odom.jerk);
    out_odom.jerk.x = jerk.x();
    out_odom.jerk.y = jerk.y();
    out_odom.jerk.z = jerk.z();

    return out_odom;
  }
  
  // ==========================================================================
  // ------------------------------ Transforms --------------------------------
  // ==========================================================================
  
  bool to_frame(tf2_ros::Buffer* tf_buffer,
		tf2::Vector3 vec, std::string source, std::string target, rclcpp::Time stamp,
		tf2::Vector3* out, rclcpp::Duration polling_sleep_duration){
    if(out == NULL)
      return false;
    
    try{
      //tf2::StampedTransform transform;
      //listener->waitForTransform(target, source, stamp, duration);
      //listener->lookupTransform(target, source, stamp, transform);

      tf2::Stamped<tf2::Transform> transform;
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer->lookupTransform(target, source, stamp, polling_sleep_duration);
      tf2::fromMsg(t, transform);
      

      *out = transform*vec;
    }
    catch(tf2::TransformException& ex){
      std::cout << "TransformException in to_frame: " << ex.what();
      return false;
    }

    return true;
  }
}
