
#include "../include/random_walk_logic.hpp"

#include "rclcpp/rclcpp.hpp"

RandomWalkPlanner::RandomWalkPlanner(init_params params)
{
    this->max_start_to_goal_dist_m_ = params.max_start_to_goal_dist_m;
    this->checking_point_cnt = params.checking_point_cnt;
    this->max_z_change_m_ = params.max_z_change_m;
    this->voxel_size_m = params.voxel_size_m;
    this->collision_padding_m = params.collision_padding_m;
    this->path_end_threshold_m = params.path_end_threshold_m;
    this->max_z_angle_change_rad = params.max_z_angle_change_rad;
}

std::optional<Path> RandomWalkPlanner::generate_straight_rand_path(
    std::tuple<float, float, float, float> start_point, float timeout_duration)
{
    // Locking mutex to prevent crashing when access the voxel map
    std::optional<Path> path;
    bool is_goal_point_valid = false;
    // get start ti
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Starting Path Search...");

    while (!is_goal_point_valid && std::chrono::duration_cast<std::chrono::seconds>(
                                       std::chrono::steady_clock::now() - start_time)
                                           .count() < timeout_duration)
    {
        std::tuple<float, float, float> goal_point = generate_goal_point(start_point);
        // RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Point Generated...");
        if (std::get<2>(goal_point) == -1)
        {
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

        std::tuple<float, float, float> current_point = std::make_tuple(
            std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point));

        // start moving in the direction of the goal point
        while (get_point_distance(current_point, goal_point) > 0.001)
        {
            if (!check_if_collided(current_point))
            {
                // add small delay to stop crashing
                float new_x = std::get<0>(current_point) + x_diff / float(this->checking_point_cnt);
                float new_y = std::get<1>(current_point) + y_diff / float(this->checking_point_cnt);
                float new_z = std::get<2>(current_point) + z_diff / float(this->checking_point_cnt);
                std::tuple<float, float, float> new_point(new_x, new_y, new_z);
                current_point = new_point;
                std::tuple<float, float, float, float> new_point_with_yaw(
                    std::get<0>(new_point), std::get<1>(new_point), std::get<2>(new_point), yaw);
                path.value().push_back(new_point_with_yaw);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Collision detected");
                is_goal_point_valid = false;
                break;
            }
            is_goal_point_valid = true;
        }
    }
    if (!is_goal_point_valid)
    {
        RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "No Path found in time limit");
        return std::nullopt;
    }
    else
    {
        return path;
    }
}

bool RandomWalkPlanner::check_if_collided_single_voxel(
    const std::tuple<float, float, float>& point, const std::tuple<float, float, float>& voxel_center)
{
    // Check if the point is within the voxel
    float x_max = std::get<0>(voxel_center) + std::get<0>(this->voxel_size_m) + this->collision_padding_m;
    float x_min = std::get<0>(voxel_center) - std::get<0>(this->voxel_size_m) - this->collision_padding_m;
    float y_max = std::get<1>(voxel_center) + std::get<1>(this->voxel_size_m) + this->collision_padding_m;
    float y_min = std::get<1>(voxel_center) - std::get<1>(this->voxel_size_m) - this->collision_padding_m;
    float z_max = std::get<2>(voxel_center) + std::get<2>(this->voxel_size_m) + this->collision_padding_m;
    float z_min = std::get<2>(voxel_center) - std::get<2>(this->voxel_size_m) - this->collision_padding_m;
    bool in_x = std::get<0>(point) < x_max && std::get<0>(point) > x_min;
    bool in_y = std::get<1>(point) < y_max && std::get<1>(point) > y_min;
    bool in_z = std::get<2>(point) < z_max && std::get<2>(point) > z_min;
    if (in_x && in_y && in_z)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool RandomWalkPlanner::check_if_collided(const std::tuple<float, float, float>& point)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    for (const std::tuple<float, float, float>& voxel : this->voxel_points)
    {
        if (voxel == std::tuple<float, float, float>(0, 0, 0))
        {
            RCLCPP_INFO(rclcpp::get_logger("random_walk_planner"), "Voxel is 0,0,0");
        }
        if (check_if_collided_single_voxel(point, voxel))
        {
            return true;
        }
    }
    return false;
}

std::tuple<float, float, float> RandomWalkPlanner::generate_goal_point(
    std::tuple<float, float, float, float> start_point)
{
    float time_out_duration = 1.0;
    const clock_t start_time = clock();

    while ((clock() - start_time) / CLOCKS_PER_SEC < time_out_duration)
    {
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
        if (angle_diff < this->max_z_angle_change_rad)
        {
            // if the point is close enough to the start point
            if (dist < this->max_start_to_goal_dist_m_)
            {
                // if the point doesnt collide with any of the voxels
                if (!(check_if_collided(rand_point)))
                {
                    return rand_point;
                }
            }
        }
    }
    return std::tuple<float, float, float>(0, 0, -1);
}

double get_point_distance(const std::tuple<float, float, float>& point1,
                          const std::tuple<float, float, float>& point2)
{
    float x_diff = std::get<0>(point1) - std::get<0>(point2);
    float y_diff = std::get<1>(point1) - std::get<1>(point2);
    float z_diff = std::get<2>(point1) - std::get<2>(point2);
    float dist = std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    if (std::isnan(dist) || std::isinf(dist)) {
        RCLCPP_ERROR(rclcpp::get_logger("random_walk_planner"), "Distance is nan or inf");
    }
    return dist;
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }

double rad2deg(double rad) { return rad * 180.0 / M_PI; }
