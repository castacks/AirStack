#include <tflib/tflib.h>

namespace tflib {

// ==========================================================================
// -------------------------------- Conversion ------------------------------
// ==========================================================================

tf2::Quaternion to_tf(geometry_msgs::msg::Quaternion q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    return quat;
}

tf2::Vector3 to_tf(geometry_msgs::msg::Point p) {
    tf2::Vector3 v(p.x, p.y, p.z);
    return v;
}

tf2::Vector3 to_tf(geometry_msgs::msg::Vector3 v) {
    tf2::Vector3 vec(v.x, v.y, v.z);
    return vec;
}
/*
geometry_msgs::msg::TransformStamped to_tf(nav_msgs::msg::Odometry odom, std::string frame) {
    tf::Transform transform;
    transform.setOrigin(to_tf(odom.pose.pose.position));
    transform.setRotation(to_tf(odom.pose.pose.orientation));

    tf::StampedTransform stamped_transform(transform, odom.header.stamp, odom.header.frame_id,
                                           frame);
    return stamped_transform;
}
*/
geometry_msgs::msg::TransformStamped to_tf(nav_msgs::msg::Odometry odom, std::string frame) {
    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = odom.header.stamp;
    stamped_transform.header.frame_id = odom.header.frame_id;
    stamped_transform.child_frame_id = frame;
    stamped_transform.transform.translation.x = odom.pose.pose.position.x;
    stamped_transform.transform.translation.y = odom.pose.pose.position.y;
    stamped_transform.transform.translation.z = odom.pose.pose.position.z;
    stamped_transform.transform.rotation = odom.pose.pose.orientation;

    return stamped_transform;
}
// ==========================================================================
// --------------------------------- Utils ----------------------------------
// ==========================================================================

tf2::Quaternion get_stabilized(tf2::Quaternion q) {
    tf2::Quaternion stabilized_q;
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    stabilized_q.setRPY(0, 0, yaw);
    return stabilized_q;
}

tf2::Transform get_stabilized(tf2::Transform transform) {
    tf2::Transform stabilized_transform;
    stabilized_transform.setOrigin(transform.getOrigin());
    stabilized_transform.setRotation(get_stabilized(transform.getRotation()));
    return stabilized_transform;
}
/*
tf2::StampedTransform get_stabilized(tf2::StampedTransform transform) {
    tf2::StampedTransform stabilized_transform(transform);
    stabilized_transform.setRotation(get_stabilized(transform.getRotation()));
    return stabilized_transform;
}
*/
geometry_msgs::msg::TransformStamped get_stabilized(
    geometry_msgs::msg::TransformStamped transform) {
    geometry_msgs::msg::TransformStamped stabilized_transform = transform;
    tf2::Quaternion q, stabilized_q;
    tf2::fromMsg(transform.transform.rotation, q);
    stabilized_q = get_stabilized(q);
    stabilized_transform.transform.rotation =
        tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(stabilized_q);
    return stabilized_transform;
}
bool transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
                        std::string new_frame_id, std::string new_child_frame_id,
                        nav_msgs::msg::Odometry* out_odom) {
    try {
        *out_odom = transform_odometry(tf_buffer, odom, new_frame_id, new_child_frame_id,
                                       rclcpp::Duration(0, 100000000));
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("tflib"), "Transform exception in transform_odometry: %s",
                     ex.what());

        return false;
    }

    return true;
}

// This can throw any exception that lookupTransform throws
nav_msgs::msg::Odometry transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
                                           std::string new_frame_id, std::string new_child_frame_id,
                                           rclcpp::Duration polling_sleep_duration) {
    nav_msgs::msg::Odometry out_odom;
    geometry_msgs::msg::TransformStamped transform_frame_id, transform_child_frame_id;
    /*
    listener->waitForTransform(new_frame_id, odom.header.frame_id, odom.header.stamp,
                               polling_sleep_duration);
    listener->lookupTransform(new_frame_id, odom.header.frame_id, odom.header.stamp,
                              transform_frame_id);
    listener->waitForTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp,
                               polling_sleep_duration);
    listener->lookupTransform(new_child_frame_id, odom.child_frame_id, odom.header.stamp,
                              transform_child_frame_id);
    */
    transform_frame_id = tf_buffer->lookupTransform(new_frame_id, odom.header.frame_id,
                                                    odom.header.stamp, polling_sleep_duration);
    transform_child_frame_id = tf_buffer->lookupTransform(
        new_child_frame_id, odom.child_frame_id, odom.header.stamp, polling_sleep_duration);

    // transform_child_frame_id.setOrigin(tf::Vector3(
    //    0, 0, 0));  // don't want translation when transforming linear and angular velocities
    transform_child_frame_id.transform.translation.x = 0;
    transform_child_frame_id.transform.translation.y = 0;
    transform_child_frame_id.transform.translation.z = 0;

    out_odom.header.stamp = odom.header.stamp;
    out_odom.header.frame_id = new_frame_id;
    out_odom.child_frame_id = new_child_frame_id;
    /*
        tf2::Vector3 position = transform_frame_id * tflib::to_tf(odom.pose.pose.position);
        out_odom.pose.pose.position.x = position.x();
        out_odom.pose.pose.position.y = position.y();
        out_odom.pose.pose.position.z = position.z();

        tf2::Quaternion quaternion = transform_frame_id * tflib::to_tf(odom.pose.pose.orientation);
        out_odom.pose.pose.orientation.x = quaternion.x();
        out_odom.pose.pose.orientation.y = quaternion.y();
        out_odom.pose.pose.orientation.z = quaternion.z();
        out_odom.pose.pose.orientation.w = quaternion.w();

        tf2::Vector3 velocity = transform_child_frame_id * tflib::to_tf(odom.twist.twist.linear);
        out_odom.twist.twist.linear.x = velocity.x();
        out_odom.twist.twist.linear.y = velocity.y();
        out_odom.twist.twist.linear.z = velocity.z();

        tf2::Vector3 angular = transform_child_frame_id * tflib::to_tf(odom.twist.twist.angular);
        out_odom.twist.twist.angular.x = angular.x();
        out_odom.twist.twist.angular.y = angular.y();
        out_odom.twist.twist.angular.z = angular.z();
    */
    // Transform position
    tf2::doTransform(odom.pose.pose.position, out_odom.pose.pose.position, transform_frame_id);

    // Transform orientation
    tf2::doTransform(odom.pose.pose.orientation, out_odom.pose.pose.orientation,
                     transform_frame_id);

    // Transform linear velocity
    tf2::doTransform(odom.twist.twist.linear, out_odom.twist.twist.linear,
                     transform_child_frame_id);

    // Transform angular velocity
    tf2::doTransform(odom.twist.twist.angular, out_odom.twist.twist.angular,
                     transform_child_frame_id);
    return out_odom;
}

// ==========================================================================
// ------------------------------ Transforms --------------------------------
// ==========================================================================

bool to_frame(tf2_ros::Buffer* tf_buffer, tf2::Vector3 vec, std::string source, std::string target,
              rclcpp::Time stamp, tf2::Vector3* out, rclcpp::Duration duration) {
    if (out == NULL) return false;

    try {
        // tf::StampedTransform transform;
        // listener->waitForTransform(target, source, stamp, duration);
        // listener->lookupTransform(target, source, stamp, transform);
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer->lookupTransform(target, source, stamp, duration);

        tf2::Transform tf2_transform;
        tf2::fromMsg(transform.transform, tf2_transform);

        *out = tf2_transform * vec;
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("tflib"), "TransformException in to_frame: %s", ex.what());
        return false;
    }

    return true;
}

}  // namespace tflib
