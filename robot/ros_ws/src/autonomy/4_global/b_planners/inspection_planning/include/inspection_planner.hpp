// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <memory>
#include <string>
#include <mutex>

struct SemanticObject
{
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Vector3 principal_axis;
    std::string semantic_class;
    int instance_id;
    std::string frame_id;
    rclcpp::Time timestamp;
};

class InspectionPlanner : public rclcpp::Node
{
public:
    InspectionPlanner();
    ~InspectionPlanner() = default;

    void initialize();

private:
    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_observed_inspection_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_predicted_inspection_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_observed_semantic_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_predicted_semantic_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_observed_local_frames_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_predicted_local_frames_;

    // ROS2 Subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_vdb_map_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_observed_semantics_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_predicted_semantics_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

    // ROS2 Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_inspection_toggle_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string robot_frame_id_;
    std::string map_frame_id_;
    std::string pub_global_plan_topic_;
    std::string pub_observed_inspection_poses_topic_;
    std::string pub_predicted_inspection_poses_topic_;
    std::string pub_observed_semantic_poses_topic_;
    std::string pub_predicted_semantic_poses_topic_;
    std::string pub_observed_local_frames_topic_;
    std::string pub_predicted_local_frames_topic_;
    std::string sub_vdb_map_topic_;
    std::string sub_observed_semantics_topic_;
    std::string sub_predicted_semantics_topic_;
    std::string sub_odometry_topic_;
    std::string srv_inspection_toggle_topic_;
    std::vector<std::vector<double>> inspection_offsets_;
    double robot_radius_;
    double workspace_x_min_, workspace_x_max_;
    double workspace_y_min_, workspace_y_max_;
    double workspace_z_min_, workspace_z_max_;

    // Data storage
    std::vector<SemanticObject> observed_semantics_;
    std::vector<SemanticObject> predicted_semantics_;
    visualization_msgs::msg::Marker vdb_map_data_;
    nav_msgs::msg::Odometry current_odometry_;
    
    // State
    bool is_active_;
    bool vdb_map_received_;
    bool odometry_received_;
    std::mutex data_mutex_;

    // Callbacks
    void vdbMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void observedSemanticsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void predictedSemanticsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void inspectionToggleCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Core functionality
    void generateInspectionPlan();
    nav_msgs::msg::Path createInspectionPath();
    
    // Utility functions
    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, 
                                           const std::string& target_frame);
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    void logSemanticsInfo();
    
    // Local coordinate frame generation
    void generateLocalFrame(const geometry_msgs::msg::Vector3& principal_axis,
                          tf2::Vector3& x_axis, tf2::Vector3& y_axis, tf2::Vector3& z_axis);
    
    // Marker processing functions
    SemanticObject extractSemanticFromMarker(const visualization_msgs::msg::Marker& marker);
    geometry_msgs::msg::Vector3 extractPrincipalAxisFromArrow(const visualization_msgs::msg::Marker& arrow_marker);
    
    // Inspection pose generation functions
    geometry_msgs::msg::PoseArray generateInspectionPoses(const std::vector<SemanticObject>& semantic_objects);
    geometry_msgs::msg::Pose calculateInspectionPose(const SemanticObject& obj, const geometry_msgs::msg::Point& offset);
    
    // Collision checking functions
    bool isInCollision(const geometry_msgs::msg::Point& position) const;
    bool isInsideWorkspace(const geometry_msgs::msg::Point& position) const;
    geometry_msgs::msg::PoseArray filterCollisionFreePoses(const geometry_msgs::msg::PoseArray& poses) const;
    geometry_msgs::msg::PoseArray filterWorkspacePoses(const geometry_msgs::msg::PoseArray& poses) const;
    
    // Semantic object pose generation functions
    geometry_msgs::msg::PoseArray generateSemanticObjectPoses(const std::vector<SemanticObject>& semantic_objects);
    geometry_msgs::msg::PoseArray generateLocalFramePoses(const std::vector<SemanticObject>& semantic_objects);
    geometry_msgs::msg::Pose createSemanticObjectPose(const SemanticObject& obj);
    geometry_msgs::msg::Pose createLocalFramePose(const SemanticObject& obj);
};
