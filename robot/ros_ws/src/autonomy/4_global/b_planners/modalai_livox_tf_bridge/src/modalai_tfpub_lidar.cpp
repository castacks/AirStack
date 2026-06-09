#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTfLidarNode : public rclcpp::Node
{
public:
    OdomToTfLidarNode() : rclcpp::Node("odom_to_tf_lidar_node")
    {
        parent_frame_ = "map";
        child_frame_ = "imu_apps_odom";

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_odom_processed", 10);
        // px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/laser_odometry",
            rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
            std::bind(&OdomToTfLidarNode::odomCallback, this, std::placeholders::_1));

        // Extrinsic: odom source Lidar pose (tilted, x left) -> Lidar pose (tilted, x front)
        Eigen::Matrix4d mat1;
        mat1 << 0.0, -1.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix4d mat2;
        mat2 << 0.8703557, 0.0, -0.4924236, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.4924236, 0.0, 0.8703557, 0.0,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix4d mat3;
        mat3 << 1.0, 0.0, 0.0, -0.01315,
            0.0, 1.0, 0.0, 0.01100,
            0.0, 0.0, 1.0, -0.01047,
            0.0, 0.0, 0.0, 1.0;

        T_superodom_lidarodom_to_imu_frd = Eigen::Isometry3d(mat1 * mat2 * mat3);

        mat1 << 0.8703557, 0.0, 0.4924236, 0.0,
            0.0, 1.0, 0.0, 0.0,
            -0.4924236, 0.0, 0.8703557, 0.0,
            0.0, 0.0, 0.0, 1.0;

        mat2 << 0.0, 1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        T_px4_local_to_superodom_lidar_mapinit = Eigen::Isometry3d(mat1 * mat2);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Isometry3d T_superodom_lidar_mapinit_to_superodom_lidarodom;
        tf2::fromMsg(msg->pose.pose, T_superodom_lidar_mapinit_to_superodom_lidarodom);

        Eigen::Isometry3d T_px4_local_to_imu_frd =
            T_px4_local_to_superodom_lidar_mapinit *
            T_superodom_lidar_mapinit_to_superodom_lidarodom *
            T_superodom_lidarodom_to_imu_frd;

        const Eigen::Vector3d vel_lidar_body(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        const Eigen::Vector3d angular_vel_lidar(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z);

        // Pose composition uses T_superodom_lidarodom_to_imu_frd as the IMU pose in the lidar frame,
        // so for body-frame velocities we need the inverse rotation (lidar -> IMU).
        const Eigen::Matrix3d R_imu_from_lidar = T_superodom_lidarodom_to_imu_frd.rotation().transpose();
        const Eigen::Vector3d p_imu_in_lidar = T_superodom_lidarodom_to_imu_frd.translation();

        // First shift the linear velocity from the lidar origin to the IMU origin in lidar coordinates,
        // then rotate both linear and angular velocity into IMU BODY_FRD.
        const Eigen::Vector3d vel_imu_origin_in_lidar =
            vel_lidar_body + angular_vel_lidar.cross(p_imu_in_lidar);
        const Eigen::Vector3d vel_px4_frd = R_imu_from_lidar * vel_imu_origin_in_lidar;
        const Eigen::Vector3d angular_vel_frd = R_imu_from_lidar * angular_vel_lidar;

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = child_frame_;
        tf_msg.transform = tf2::eigenToTransform(T_px4_local_to_imu_frd).transform;
        tf_broadcaster_->sendTransform(tf_msg);

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

        px4_msgs::msg::VehicleOdometry px4_odom;

        // Experiment: use the current ROS time instead of the message stamp to test
        // whether PX4 is rejecting the sample due to stale or inconsistent timestamps.
        // uint64_t timestamp_us = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
        
        // this part should be done on the drone
        // const uint64_t now_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
        // px4_odom.timestamp = now_us;
        // px4_odom.timestamp_sample = now_us;

        // px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

        // px4_odom.position[0] = T_px4_local_to_imu_frd.translation().x();
        // px4_odom.position[1] = T_px4_local_to_imu_frd.translation().y();
        // px4_odom.position[2] = T_px4_local_to_imu_frd.translation().z();

        // Eigen::Quaterniond q(T_px4_local_to_imu_frd.rotation());
        // px4_odom.q[0] = q.w();
        // px4_odom.q[1] = q.x();
        // px4_odom.q[2] = q.y();
        // px4_odom.q[3] = q.z();

        // px4_odom.position_variance = {1e-2f, 1e-2f, 1e-2f};
        // px4_odom.orientation_variance = {1e-2f, 1e-2f, 1e-2f};

        // px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
        // px4_odom.velocity[0] = vel_px4_frd.x();
        // px4_odom.velocity[1] = vel_px4_frd.y();
        // px4_odom.velocity[2] = vel_px4_frd.z();
        // px4_odom.angular_velocity[0] = angular_vel_frd.x();
        // px4_odom.angular_velocity[1] = angular_vel_frd.y();
        // px4_odom.angular_velocity[2] = angular_vel_frd.z();
        // px4_odom.velocity_variance = {2.5e-3f, 2.5e-3f, 2.5e-3f};

        // px4_odom.reset_counter = 0;
        // px4_odom.quality = 100;
        // px4_odom_pub_->publish(px4_odom);
    }

    std::string parent_frame_;
    std::string child_frame_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;

    Eigen::Isometry3d T_superodom_lidarodom_to_imu_frd;
    Eigen::Isometry3d T_px4_local_to_superodom_lidar_mapinit;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTfLidarNode>());
    rclcpp::shutdown();
    return 0;
}
