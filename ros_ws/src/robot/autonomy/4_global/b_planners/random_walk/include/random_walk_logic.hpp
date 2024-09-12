
#ifndef RANDOM_WALK_LOGIC_H
#define RANDOM_WALK_LOGIC_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <vector>

typedef std::vector<std::tuple<float, float, float, float>> Path;  // x, y, z, yaw

struct init_params {
    float max_start_to_goal_dist_m;
    float max_angle_change_deg;
    int checking_point_cnt;
    float waypoint_dist_m;
    float max_z_m;
    float collision_padding_m;
    float path_end_threshold_m;
    std::tuple<float, float, float> voxel_size_m;
};

class RandomWalkPlanner {
   public:
    RandomWalkPlanner() = default;
    RandomWalkPlanner(init_params params);
    ~RandomWalkPlanner() = default;

    std::optional<Path> generate_straight_rand_path(
        std::tuple<float, float, float, float> start_point,
        float timeout_duration = 1.0);  // x, y, z, yaw

    std::vector<std::tuple<float, float, float>> voxel_points;

    float path_end_threshold_m;

   private:
    // Numerical constants
    float max_start_to_goal_dist_m_;
    float max_angle_change_deg_;
    int checking_point_cnt;
    float waypoint_dist_m_;
    float max_z_m_;
    float collision_padding_m;

    // Variables
    std::tuple<float, float, float> voxel_size_m;

    // Functions
    bool check_if_collided_single_voxel(std::tuple<float, float, float> point,
                                        std::tuple<float, float, float> voxel_center);

    bool check_if_collided(std::tuple<float, float, float> point);

    std::tuple<float, float, float> generate_goal_point(
        std::tuple<float, float, float> start_point);
};

double get_point_distance(std::tuple<float, float, float> point1,
                          std::tuple<float, float, float> point2);

double deg2rad(double deg);

#endif  // RANDOM_WALK_LOGIC_H
