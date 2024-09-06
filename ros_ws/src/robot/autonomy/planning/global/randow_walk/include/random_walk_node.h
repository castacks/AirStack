/**************************************************************************
random_walk_planner.h

Graeme Best (bestg@oregonstate.edu)
Jan 2020

Copyright Carnegie Mellon University (CMU) <2019>

This code is proprietary to the CMU SubT challenge. Do not share or distribute
without express permission of a project lead (Sebastian or Matt).
**************************************************************************/
#include <string>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include 

#include "rclcpp/rclcpp.hpp"
#include "random_walk_logic.h"

class RandomWalkNode : public rclcpp::Node {
   private:
    // ROS subscribers
    ros::Subscriber sub_vdb_map;

    // ROS publishers
    ros::Publisher pub_global_path;

    //Planner
    RandomWalkPlanner random_walk_planner;

    // String constants
    std::string world_frame_id_;
    std::string pub_global_path_topic_;
    std::string sub_vdb_map_topic_;

    // Variables
    airstack_msgs::msg::TrajectoryXYZVYaw generated_path;
    bool is_path_generated = false;
    bool is_path_sent = false;
    bool is_path_executing = false;
    bool received_first_odometry = false;

    // Callbacks
    void vdbmapCallback(const visualization_msgs::Marker &msg);

    void odometryCallback(const nav_msgs::Odometry &msg);

    // Other functions
    std::optional<random_walk_init_params> readParameters();

   public:
    RandomWalkNode();
    
};

#endif  // RANDOM_WALK_PLANNER_H
