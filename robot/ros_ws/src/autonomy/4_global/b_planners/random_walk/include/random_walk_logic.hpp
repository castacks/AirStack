
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
#include <mutex>

typedef std::vector<std::tuple<float, float, float, float>> Path;  // x, y, z, yaw

struct init_params {
    float max_start_to_goal_dist_m;
    int checking_point_cnt;
    float max_z_change_m;
    float collision_padding_m;
    float path_end_threshold_m;
    float max_z_angle_change_rad;
    std::tuple<float, float, float> voxel_size_m;
};

class RandomWalkPlanner {
   public:
    RandomWalkPlanner() = default;
    RandomWalkPlanner(init_params params);
    ~RandomWalkPlanner() = default;

    std::optional<Path> generate_straight_rand_path(
        std::tuple<float, float, float, float> start_point, float timeout_duration);  // x, y, z, yaw

    float path_end_threshold_m;
    
    std::vector<std::tuple<float, float, float>> voxel_points;

   private:
    // Numerical constants
    float max_start_to_goal_dist_m_;
    int checking_point_cnt;
    float max_z_change_m_;
    float collision_padding_m;
    float max_z_angle_change_rad;

    // Variables
    std::tuple<float, float, float> voxel_size_m;

    std::mutex mutex;

    // Functions
    bool check_if_collided_single_voxel(const std::tuple<float, float, float>& point,
                                        const std::tuple<float, float, float>& voxel_center);

    bool check_if_collided(const std::tuple<float, float, float>& point);

    std::tuple<float, float, float> generate_goal_point(
        std::tuple<float, float, float, float> start_point);
};

double get_point_distance(const std::tuple<float, float, float>& point1,
                          const std::tuple<float, float, float>& point2);

double deg2rad(double deg);

double rad2deg(double rad);

#endif  // RANDOM_WALK_LOGIC_H
