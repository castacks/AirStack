
#include "../include/random_walk_logic.hpp"

#include "rclcpp/rclcpp.hpp"

RandomWalkPlanner::RandomWalkPlanner(init_params params) {
    this->max_start_to_goal_dist_m_ = params.max_start_to_goal_dist_m;
    this->checking_point_cnt = params.checking_point_cnt;
    this->max_z_change_m_ = params.max_z_change_m;
    this->voxel_size_m = params.voxel_size_m;
    this->collision_padding_m = params.collision_padding_m;
    this->path_end_threshold_m = params.path_end_threshold_m;
    this->max_z_angle_change_rad = params.max_z_angle_change_rad;
}

std::optional<Path> RandomWalkPlanner::generate_straight_rand_path(
    std::tuple<float, float, float, float> start_point, float timeout_duration) {
    // std::tuple<float, float, float> start_point_wo_yaw(
    //     std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point));
    std::optional<Path> path;
    bool is_goal_point_valid = false;
    // get start ti
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Starting Path Search...");
    while (!is_goal_point_valid && std::chrono::duration_cast<std::chrono::seconds>(
                                       std::chrono::steady_clock::now() - start_time)
                                           .count() < timeout_duration) {
        std::tuple<float, float, float> goal_point = generate_goal_point(start_point);
        // RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Point Generated...");
        if (std::get<2>(goal_point) == -1) {
            RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"),
                        "No valid goal point found in timeout duration");
            break;
        }

        float x_diff = std::get<0>(goal_point) - std::get<0>(start_point);
        float y_diff = std::get<1>(goal_point) - std::get<1>(start_point);
        float z_diff = std::get<2>(goal_point) - std::get<2>(start_point);
        float yaw = std::atan2(y_diff, x_diff);

        path = Path();
        std::tuple<float, float, float, float> first_point(
            std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point), yaw);
        path.value().push_back(first_point);

        // start moving in the direction of the goal point
        std::tuple<float, float, float> current_point = std::make_tuple(
            std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point));
        // RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"),
        //             "Checking intermediate points at tolerance %f, with point count %d",
        //             this->path_end_threshold_m, this->checking_point_cnt);
        while (get_point_distance(current_point, goal_point) > 0.001) {
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
                RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Collision detected");
                is_goal_point_valid = false;
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
    float dist = get_point_distance(point, voxel_center);
    float max_size =
        std::max(std::get<0>(this->voxel_size_m),
                 std::max(std::get<1>(this->voxel_size_m), std::get<2>(this->voxel_size_m)));
    if (dist < max_size + this->collision_padding_m) {
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
    std::tuple<float, float, float, float> start_point) {
    float time_out_duration = 1.0;
    const clock_t start_time = clock();

    while ((clock() - start_time) / CLOCKS_PER_SEC < time_out_duration) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> distribution_xy(-this->max_start_to_goal_dist_m_,
                                                     this->max_start_to_goal_dist_m_);
        std::uniform_real_distribution<float> distribution_z(-this->max_z_change_m_, this->max_z_change_m_);
        float delta_x = distribution_xy(gen);
        float delta_y = distribution_xy(gen);
        float delta_z = distribution_z(gen);
        float rand_x = std::get<0>(start_point) + delta_x;
        float rand_y = std::get<1>(start_point) + delta_y;
        float rand_z = std::get<2>(start_point) + delta_z;
        std::tuple<float, float, float> rand_point(rand_x, rand_y, rand_z);
        std::tuple<float, float, float> start_point_wo_yaw(
            std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point));
        float dist = get_point_distance(start_point_wo_yaw, rand_point);
        float new_angle = std::atan2(std::get<1>(rand_point) - std::get<1>(start_point_wo_yaw),
                                     std::get<0>(rand_point) - std::get<0>(start_point_wo_yaw));
        float angle_diff = std::abs(std::get<3>(start_point) - new_angle);
        // if the z value of the point is low enough
        if (rand_z < max_z_change_m_) {
            // if the angle change is low enough
            if (angle_diff < this->max_z_angle_change_rad) {
                // if the point is close enough to the start point
                if (dist < this->max_start_to_goal_dist_m_) {
                    // if the point doesnt collide with any of the voxels
                    if (!(check_if_collided(rand_point))) {
                        // RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"),
                        //             "Angle Difference: %f", rad2deg(angle_diff));
                        return rand_point;
                    }
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

double rad2deg(double rad) { return rad * 180.0 / M_PI; }
