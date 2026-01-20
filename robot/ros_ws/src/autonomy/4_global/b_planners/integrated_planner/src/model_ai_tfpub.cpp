// odom_to_tf_node.cpp
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomToTfNode : public rclcpp::Node
{
public:
    OdomToTfNode() : rclcpp::Node("odom_to_tf_node")
    {
        parent_frame_ = "map";
        // child_frame_ = "imu_apps_odom";
        child_frame_ = "imu_apps_odom";

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/ov/odom",
                                                                  rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
                                                                  std::bind(&OdomToTfNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Listening: %s -> Publishing TF: %s -> %s",
                    sub_->get_topic_name(), parent_frame_.c_str(), child_frame_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = msg->header.stamp;

        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = child_frame_;

        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;

        tf_msg.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::string parent_frame_;
    std::string child_frame_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTfNode>());
    rclcpp::shutdown();
    return 0;
}
