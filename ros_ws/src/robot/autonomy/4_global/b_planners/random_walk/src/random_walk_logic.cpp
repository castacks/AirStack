
#include <string>
#include <vector>
#include <tuple>
#include <cmath>
#include <random>
#include <chrono>

#include "../include/random_walk_logic.h"

RandomWalkPlanner::RandomWalkPlanner(random_walk_init_params init_params)
{
    max_start_to_goal_dist_m_ = init_params.max_start_to_goal_dist_m;
    max_angle_change_deg_ = init_params.max_angle_change_deg;
    checking_dist_m_ = init_params.checking_dist_m;
    waypoint_dist_m_ = init_params.waypoint_dist_m;
    max_z_m_ = init_params.max_z_m;
    voxel_size_m = init_params.voxel_size_m;
    collision_padding_m = init_params.collision_padding_m;
}

std::vector<std::tuple<float, float, float>> RandomWalkPlanner::generate_path(
    std::tuple<float, float, float> start_point) {
        std::vector<std::tuple<float, float, float>> path;
        std::tuple<float, float, float> goal_point = generate_goal_point(start_point);
        if (std::get<2>(goal_point) == -1) {
            return path;
        }
        path.push_back(start_point);
        
        // TODO: complete the rest of the function
    return std::vector<std::tuple<float, float, float>>();
}

bool RandomWalkPlanner::check_if_collided_single_voxel(std::tuple<float, float, float> point,
                                                       std::tuple<float, float, float> voxel_center)
{
    // Check if the point is within the voxel
    float x = std::get<0>(point);
    float y = std::get<1>(point);
    float z = std::get<2>(point);
    float x_voxel = std::get<0>(voxel_center);
    float y_voxel = std::get<1>(voxel_center);
    float z_voxel = std::get<2>(voxel_center);

    if (x > x_voxel - std::get<0>(voxel_size_m) / 2 && x < x_voxel + std::get<0>(voxel_size_m) / 2 &&
        y > y_voxel - std::get<1>(voxel_size_m) / 2 && y < y_voxel + std::get<1>(voxel_size_m) / 2 &&
        z > z_voxel - std::get<2>(voxel_size_m) / 2 && z < z_voxel + std::get<2>(voxel_size_m) / 2)
    {
        return true;
    }
    return false;
}

bool RandomWalkPlanner::check_if_collided(std::tuple<float, float, float> point)
{
    // Check if the point is within the voxel
    for (auto voxel : this->voxel_points)
    {
        if (check_if_collided_single_voxel(point, voxel))
        {
            return true;
        }
    }
    return false;
}

std::tuple<float, float, float> RandomWalkPlanner::generate_goal_point(std::tuple<float, float, float> start_point) {
    //TODO: make this limited by some input
    int time_out_duration = 1;
    auto start_time = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_time).count() < time_out_duration) {
        float rand_x = rand() % 10;
        float rand_y = rand() % 10;
        float rand_z = rand() % 10;
        float x_diff = rand_x - std::get<0>(start_point);
        float y_diff = rand_y - std::get<1>(start_point);
        float z_diff = rand_z - std::get<2>(start_point); 
        if (rand_z < max_z_m_) {
            if (std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff) < max_start_to_goal_dist_m_) {
                if (!(check_if_collided(std::tuple<float, float, float>(rand_x, rand_y, rand_z)))) {
                    return std::tuple<float, float, float>(rand_x, rand_y, rand_z);
                }
            }
        }
    }
    return std::tuple<float, float, float>(0, 0, -1);
}
