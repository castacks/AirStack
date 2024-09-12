
#include "../include/random_walk_logic.hpp"

RandomWalkPlanner::RandomWalkPlanner(init_params params) {
    this->max_start_to_goal_dist_m_ = params.max_start_to_goal_dist_m;
    this->max_angle_change_deg_ = params.max_angle_change_deg;
    this->checking_point_cnt = params.checking_point_cnt;
    this->waypoint_dist_m_ = params.waypoint_dist_m;
    this->max_z_m_ = params.max_z_m;
    this->voxel_size_m = params.voxel_size_m;
    this->collision_padding_m = params.collision_padding_m;
}

std::optional<Path> RandomWalkPlanner::generate_straight_rand_path(
    std::tuple<float, float, float, float> start_point, float timeout_duration) {
    std::tuple<float, float, float> start_point_wo_yaw(
        std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point));
    std::optional<Path> path;
    bool is_goal_point_valid = false;
    // get start time
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::cout << "Starting path searching" << std::endl;
    while (!is_goal_point_valid && std::chrono::duration_cast<std::chrono::seconds>(
                                       std::chrono::steady_clock::now() - start_time)
                                           .count() < timeout_duration) {
        std::tuple<float, float, float> goal_point = generate_goal_point(start_point_wo_yaw);
        if (std::get<2>(goal_point) == -1) {
            std::cout << "No valid goal point found" << std::endl;
            break;
        }
        float x_diff = std::get<0>(goal_point) - std::get<0>(start_point_wo_yaw);
        float y_diff = std::get<1>(goal_point) - std::get<1>(start_point_wo_yaw);
        float z_diff = std::get<2>(goal_point) - std::get<2>(start_point_wo_yaw);
        float yaw = std::atan2(y_diff, x_diff);

        path = Path();
        std::tuple<float, float, float, float> first_point(
            std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point), yaw);
        path.value().push_back(first_point);

        // start moving in the direction of the goal point
        std::tuple<float, float, float> current_point = start_point_wo_yaw;

        while (get_point_distance(current_point, goal_point) > this->path_end_threshold_m) {
            if (!check_if_collided(current_point)) {
                float new_x = std::get<0>(current_point) + x_diff / this->checking_point_cnt;
                float new_y = std::get<1>(current_point) + y_diff / this->checking_point_cnt;
                float new_z = std::get<2>(current_point) + z_diff / this->checking_point_cnt;
                std::tuple<float, float, float> new_point(new_x, new_y, new_z);
                current_point = new_point;
                std::tuple<float, float, float, float> new_point_with_yaw(
                    std::get<0>(new_point), std::get<1>(new_point), std::get<2>(new_point), yaw);
                path.value().push_back(new_point_with_yaw);
            } else {
                break;
            }
            is_goal_point_valid = true;
        }
    }
    if (!is_goal_point_valid) {
        return std::nullopt;
    } else {
        return path;
    }
}

bool RandomWalkPlanner::check_if_collided_single_voxel(
    std::tuple<float, float, float> point, std::tuple<float, float, float> voxel_center) {
    // Check if the point is within the voxel
    float x = std::get<0>(point);
    float y = std::get<1>(point);
    float z = std::get<2>(point);
    float x_voxel = std::get<0>(voxel_center);
    float y_voxel = std::get<1>(voxel_center);
    float z_voxel = std::get<2>(voxel_center);

    if (x > x_voxel - std::get<0>(voxel_size_m) / 2 &&
        x < x_voxel + std::get<0>(voxel_size_m) / 2 &&
        y > y_voxel - std::get<1>(voxel_size_m) / 2 &&
        y < y_voxel + std::get<1>(voxel_size_m) / 2 &&
        z > z_voxel - std::get<2>(voxel_size_m) / 2 &&
        z < z_voxel + std::get<2>(voxel_size_m) / 2) {
        return true;
    }
    return false;
}

bool RandomWalkPlanner::check_if_collided(std::tuple<float, float, float> point) {
    // Check if the point is within the voxel
    for (auto voxel : this->voxel_points) {
        if (check_if_collided_single_voxel(point, voxel)) {
            return true;
        }
    }
    return false;
}

std::tuple<float, float, float> RandomWalkPlanner::generate_goal_point(
    std::tuple<float, float, float> start_point) {
    // TODO: make this random generation limited by some input to generate less points
    float time_out_duration = 1.0;
    const clock_t start_time = clock();

    while ((clock() - start_time) / CLOCKS_PER_SEC < time_out_duration) {
        float rand_x = rand() % 10;
        float rand_y = rand() % 10;
        float rand_z = rand() % 10;
        float x_diff = rand_x - std::get<0>(start_point);
        float y_diff = rand_y - std::get<1>(start_point);
        float z_diff = rand_z - std::get<2>(start_point);
        // if the z value of the point is low enough
        if (rand_z < max_z_m_) {
            // if the point is close enough to the start point
            if (std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff) <
                max_start_to_goal_dist_m_) {
                // if the point doesnt collide with any of the voxels
                if (!(check_if_collided(std::tuple<float, float, float>(rand_x, rand_y, rand_z)))) {
                    return std::tuple<float, float, float>(rand_x, rand_y, rand_z);
                }
            }
        }
    }
    return std::tuple<float, float, float>(0, 0, -1);
}

double get_point_distance(std::tuple<float, float, float> point1,
                          std::tuple<float, float, float> point2) {
    float x_diff = std::get<0>(point1) - std::get<0>(point2);
    float y_diff = std::get<1>(point1) - std::get<1>(point2);
    float z_diff = std::get<2>(point1) - std::get<2>(point2);
    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }
