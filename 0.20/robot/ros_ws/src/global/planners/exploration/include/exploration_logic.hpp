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

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "utils/utils.hpp"
#include "utils/rrt_planner.h"
#include "viewpoint_sampling.hpp"
#include "utils/viewpoint.hpp"
#include "utils/collision_checker.h"
#include "rclcpp/rclcpp.hpp"

typedef std::vector<std::tuple<float, float, float, float>> Path; // x, y, z, yaw
using GridT = openvdb::Grid<openvdb::tree::Tree4<float, 5, 4, 3>::Type>;

struct TimedViewPoint
{
    ViewPoint point;
    rclcpp::Time time;
};

struct init_params
{
    float max_start_to_goal_dist_m;
    int checking_point_cnt;
    float max_z_change_m;
    float collision_padding_m;
    float path_end_threshold_m;
    float max_yaw_change_degrees;

    std::tuple<float, float, float> voxel_size_m;

    // viewpoint sampling params
    double kLOcc_;
    double kVoxSize_;
    double kViewingDistance_;
    double kBboxLength_, kBboxBreadth_, kBboxHeight_;
    int kNumViewpointsPerCluster_;
    double kViewingDistanceInnerR_, kViewingDistanceOuterR_, kViewingDistanceZOffset_;
    double kCollCheckEndpointOffset_;
    double kRobotModelBBoxFraction_;
    double kViewpBBoxUnknownFracThresh_, kViewpBBoxOccupiedFracThresh_;

    // in init_params
    double too_close_distance, too_close_penalty;
    double odometry_match_distance, odometry_match_angle, odometry_match_penalty;
    double momentum_collision_check_distance, momentum_collision_check_distance_min;
    double momentum_change_max_reward, momentum_change_collision_reward;
    double viewpoint_neighborhood_too_close_radius, viewpoint_neighborhood_sphere_radius;
    int viewpoint_neighborhood_count;
    double momentum_minimum_distance;
    double momentum_time;
    double momentum_collision_check_step_size;
    
    // RRT
    double planner_norm_limit_;
    double max_explore_dist_;
    int planner_max_iter_;
    int max_connection_iter_;
    int traj_smooth_horizon_;
    int rrt_region_nodes_count_;
    double rrt_region_nodes_radius_;
    int rrt_region_nodes_z_layers_count_up_;
    double rrt_region_nodes_z_layers_step_;

    int priority_level_thred_;
    double dense_step_;
};

class ExplorationPlanner
{
public:
    ExplorationPlanner() = default;
    ExplorationPlanner(init_params params);
    ~ExplorationPlanner() = default;

    std::optional<Path> generate_straight_rand_path(
        std::tuple<float, float, float, float> start_point,
        float timeout_duration); // x, y, z, yaw

    std::optional<Path> select_viewpoint_and_plan(const ViewPoint& start_point, float timeout_duration);

    std::optional<Path> plan_to_given_waypoint(const ViewPoint& start_point, const ViewPoint& goal_point);

    float path_end_threshold_m;

    std::vector<std::tuple<float, float, float>> voxel_points;

    bool first_grid_received = false;
    bool grid_ready = false;

    GridT::Ptr vdb_grid;

    openvdb::FloatGrid::Ptr occ_grid = openvdb::FloatGrid::create(0.0);
    openvdb::FloatGrid::Ptr prev_grid = openvdb::FloatGrid::create(0.0); // to save time for frontier extraction, only check diff for new free voxels
    openvdb::BoolGrid::Ptr frontier_grid = openvdb::BoolGrid::create(false);

    std::vector<std::vector<Eigen::Vector3d>> clustered_points_;

    std::shared_ptr<viewpoint_sampling_ns::ViewpointSampling> viewp_sample_;

    collision_checker_ns::CollisionChecker collision_checker_;

    std::vector<ViewPoint> executed_trajectory_;
    std::vector<TimedViewPoint> momentum_executed_trajectory_;

    RRT_Planner rrt_planner_;
    double dense_step_;

private:
    // Numerical constants
    float max_start_to_goal_dist_m_;
    int checking_point_cnt;
    float max_z_change_m_;
    float collision_padding_m;
    float max_yaw_change_degrees;

    double kLOcc_;
    double kVoxSize_;
    double kViewingDistance_;
    double kBboxLength_, kBboxBreadth_, kBboxHeight_;
    int kNumViewpointsPerCluster_;
    double kViewingDistanceInnerR_, kViewingDistanceOuterR_, kViewingDistanceZOffset_;
    double kCollCheckEndpointOffset_;
    double kRobotModelBBoxFraction_;
    double kViewpBBoxUnknownFracThresh_, kViewpBBoxOccupiedFracThresh_;

    double too_close_distance_, too_close_penalty_;
    double odometry_match_distance_, odometry_match_angle_, odometry_match_penalty_;
    double momentum_collision_check_distance_, momentum_collision_check_distance_min_;
    double momentum_change_max_reward_, momentum_change_collision_reward_;
    double viewpoint_neighborhood_too_close_radius_, viewpoint_neighborhood_sphere_radius_;
    int viewpoint_neighborhood_count_;
    double momentum_minimum_distance_;
    double momentum_time_;
    double momentum_collision_check_step_size_;

    double planner_norm_limit_;
    double max_explore_dist_;
    int planner_max_iter_;
    int max_connection_iter_;
    int traj_smooth_horizon_;
    int rrt_region_nodes_count_;
    double rrt_region_nodes_radius_;
    int rrt_region_nodes_z_layers_count_up_;
    double rrt_region_nodes_z_layers_step_;

    int priority_level_thred_;

    ViewPoint robot_pos_, explore_goal_;

    bool explore_goal_cleared_by_other_robot_;

    // Variables
    std::tuple<float, float, float> voxel_size_m;

    std::mutex mutex;

    std::vector<ViewPoint> sampled_viewpoints;

    // Functions
    bool check_if_collided_single_voxel(const std::tuple<float, float, float> &point,
                                        const std::tuple<float, float, float> &voxel_center);

    bool check_if_collided(const std::tuple<float, float, float> &point);

    double angleBetweenHeadingAndMomentum(const ViewPoint &point);

    bool computeMomentumVector(double &momentum_direction_x, double &momentum_direction_y, double &momentum_direction_z);

    float getCylinderReward(const int &viewpoint_index, const ViewPoint &planning_point);

    bool extractCylinderViewpointDistances(const int &viewpoint_index, const ViewPoint &planning_point, std::vector<float> &viewpoint_distances);

    bool doesPointMatchTrajectory(const ViewPoint &point);

    void goalSelector(int priority_level);

    static bool compareFrontier(const ViewPoint &p1, const ViewPoint &p2);

    std::tuple<float, float, float> generate_goal_point(std::tuple<float, float, float, float> start_point);
};

double get_point_distance(const std::tuple<float, float, float> &point1,
                          const std::tuple<float, float, float> &point2);

double deg2rad(double deg);

double rad2deg(double rad);

bool lineSegDistances(const ViewPoint &point,
                      const ViewPoint &line_segment_start,
                      const ViewPoint &line_segment_end,
                      double &parallel_distance,
                      double &perpendicular_distance);