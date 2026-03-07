// odom_to_tf_node.cpp
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OdomToTfNode : public rclcpp::Node
{
public:
    OdomToTfNode() : rclcpp::Node("odom_to_tf_node")
    {
        parent_frame_ = "map";
        // child_frame_ = "imu_apps_odom";
        child_frame_ = "imu_apps_odom";

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mocap_odom_processed", 10);
        px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/modalai/pose",
                                                                               rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
                                                                               std::bind(&OdomToTfNode::poseCallback, this, std::placeholders::_1));
        
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/ov/odom",
        //                                                                rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
        //                                                                std::bind(&OdomToTfNode::odomCallback, this, std::placeholders::_1));

        Eigen::Matrix4d mat;
        mat << 0.99923, 0.03749, -0.01151, 0.00900,
            -0.03744, 0.99929, 0.00403, 0.00115,
            0.01166, -0.00360, 0.99993, 0.01700,
            0.00000, 0.00000, 0.00000, 1.00000;
        Eigen::Isometry3d T_mocap_rigidbody_to_imu_flu = Eigen::Isometry3d(mat);

        mat << 1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        T_px4_local_to_mocap_world_ = Eigen::Isometry3d(mat);
        Eigen::Isometry3d T_imu_flu_to_imu_frd = Eigen::Isometry3d(mat);

        T_mocap_rigidbody_to_imu_frd_ = T_mocap_rigidbody_to_imu_flu * T_imu_flu_to_imu_frd;
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

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(),"subscribing pose stamped");
        Eigen::Isometry3d T_mocap_world_to_mocap_rigidbody;
        tf2::fromMsg(msg->pose, T_mocap_world_to_mocap_rigidbody);

        Eigen::Isometry3d T_px4_local_to_imu_frd =
            T_px4_local_to_mocap_world_ * T_mocap_world_to_mocap_rigidbody * T_mocap_rigidbody_to_imu_frd_;

        ////////////////////////////////////////////////////////////////
        // TF for vdb mapping
        ////////////////////////////////////////////////////////////////
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = child_frame_;
        tf_msg.transform = tf2::eigenToTransform(T_px4_local_to_imu_frd).transform;
        tf_broadcaster_->sendTransform(tf_msg);

        ////////////////////////////////////////////////////////////////
        // Odometry for ros controller node
        ////////////////////////////////////////////////////////////////
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = parent_frame_;
        odom_msg.child_frame_id = child_frame_;

        odom_msg.pose.pose = tf2::toMsg(T_px4_local_to_imu_frd);

        odom_msg.pose.covariance = {
            1e-4, 0, 0, 0, 0, 0,
            0, 1e-4, 0, 0, 0, 0,
            0, 0, 1e-4, 0, 0, 0,
            0, 0, 0, 1e-4, 0, 0,
            0, 0, 0, 0, 1e-4, 0,
            0, 0, 0, 0, 0, 1e-4};
        odom_msg.twist.covariance[0] = -1.0;
        odom_pub_->publish(odom_msg);

        ////////////////////////////////////////////////////////////////
        // PX4 Odometry for modal ai drone
        ////////////////////////////////////////////////////////////////
        px4_msgs::msg::VehicleOdometry px4_odom;

        // ROS(s/ns) to PX4(us)
        uint64_t timestamp_us = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
        px4_odom.timestamp = timestamp_us;
        px4_odom.timestamp_sample = timestamp_us;

        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

        px4_odom.position[0] = T_px4_local_to_imu_frd.translation().x();
        px4_odom.position[1] = T_px4_local_to_imu_frd.translation().y();
        px4_odom.position[2] = T_px4_local_to_imu_frd.translation().z();

        Eigen::Quaterniond q(T_px4_local_to_imu_frd.rotation());
        px4_odom.q[0] = q.w();
        px4_odom.q[1] = q.x();
        px4_odom.q[2] = q.y();
        px4_odom.q[3] = q.z();

        px4_odom.position_variance = {1e-4f, 1e-4f, 1e-4f};
        px4_odom.orientation_variance = {1e-4f, 1e-4f, 1e-4f};

        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        px4_odom.velocity = {NAN, NAN, NAN};
        px4_odom.angular_velocity = {NAN, NAN, NAN};
        px4_odom.velocity_variance = {NAN, NAN, NAN};
        
        px4_odom.reset_counter = 0;
        px4_odom.quality = 100;
        px4_odom_pub_->publish(px4_odom);
    }

    std::string parent_frame_;
    std::string child_frame_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;

    Eigen::Isometry3d T_mocap_rigidbody_to_imu_frd_;
    // px4 local is in FRD, mocap world is FLU
    Eigen::Isometry3d T_px4_local_to_mocap_world_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTfNode>());
    rclcpp::shutdown();
    return 0;
}
