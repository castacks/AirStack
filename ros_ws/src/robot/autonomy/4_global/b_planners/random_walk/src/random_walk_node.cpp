/**************************************************************************
random_walk_planner.cpp

Graeme Best (bestg@oregonstate.edu)
Jan 2020

Copyright Carnegie Mellon University (CMU) <2019>

This code is proprietary to the CMU SubT challenge. Do not share or distribute
without express permission of a project lead (Sebastian or Matt).
**************************************************************************/

#include <math.h>
#include <tf2/transform_datatypes.h>
#define PI 3.14159265
#include <stdlib.h>
#include <visualization_msgs/msg/marker.h>

#include "../include/random_walk_logic.h"
#include "../include/random_walk_node.h"

RandomWalkNode::RandomWalkNode() {
    // read parameters
    std::optional<random_walk_init_params> init_params = readParameters();
    // Initialize the random walk planner
    if (init_params)
      this->random_walk_planner = RandomWalkPlanner(init_params);
    else
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize random walk planner");

    this->create_subscription<visualization_msgs::Marker>(sub_vdb_map_topic_, 10,
                                                          &RandomWalkNode::vdbmapCallback);
    this->pub_global_path = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(
        pub_global_path_topic_, 10);
}

bool RandomWalkNode::readParameters() void RandomWalkNode::vdbmapCallback(
    const visualization_msgs::Marker& msg) {
    // get the cube list
    std::tuple<float,float,float> start_point = std::tuple<float,float,float>(0,0,0);
    Path generated_path = this->random_walk_planner.generate_path(start_point);
    if (generated_path.size() > 0) {
        // publish the path
        airstack_msgs::msg::TrajectoryXYZVYaw path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = world_frame_id_;
        for (auto point : generated_path) {
            airstack_msgs::msg::TrajectoryPointXYZVYaw path_point;
            path_point.x = std::get<0>(point);
            path_point.y = std::get<1>(point);
            path_point.z = std::get<2>(point);
            path_point.yaw = std::get<3>(point);
            path_msg.points.push_back(path_point);
        }
        pub_global_path->publish(path_msg);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
    }

}

std::optional<random_walk_init_params> RandomWalkNode::readParameters() {
    // Read in parameters based off the default yaml file
    random_walk_init_params init_params;
    if (!this->get_parameter("world_frame_id_", world_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: world_frame_id_");
        return false;
    }
    else { init_params.world_frame_id_ = world_frame_id_; }
    if (!this->get_parameter("pub_global_path_topic", pub_global_path_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_global_path_topic");
        return false;
    }
    else { init_params.pub_global_path_topic_ = pub_global_path_topic_; }
    if (!this->get_parameter("sub_vdb_map_topic", sub_vdb_map_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_vdb_map_topic");
        return false;
    }
    else { init_params.sub_vdb_map_topic_ = sub_vdb_map_topic_; }
    if (!this->get_parameter("max_start_to_goal_dist_m", max_start_to_goal_dist_m_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_start_to_goal_dist_m");
        return false;
    }
    else { init_params.max_start_to_goal_dist_m_ = max_start_to_goal_dist_m_; }
    if (!this->get_parameter("max_angle_change_deg", max_angle_change_deg_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_angle_change_deg");
        return false;
    }
    else { init_params.max_angle_change_deg_ = max_angle_change_deg_; }
    if (!this->get_parameter("checking_dist_m", checking_dist_m_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: checking_dist_m");
        return false;
    }
    else { init_params.checking_dist_m_ = checking_dist_m_; }
    if (!this->get_parameter("waypoint_dist_m", waypoint_dist_m_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: waypoint_dist_m");
        return false;
    }
    else { init_params.waypoint_dist_m_ = waypoint_dist_m_; }
    if (!this->get_parameter("max_z_m", max_z_m_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_z_m");
        return false;
    }
    else { init_params.max_z_m_ = max_z_m_; }
    if (!this->get_parameter("collision_padding_m", collision_padding_m_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: collision_padding_m");
        return false;
    }
    else { init_params.collision_padding_m = collision_padding_m_; }

    return init_params;
}