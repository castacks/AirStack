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

#include "../include/exploration_logic.hpp"

ExplorationPlanner::ExplorationPlanner(init_params params)
{
    this->max_start_to_goal_dist_m_ = params.max_start_to_goal_dist_m;
    this->checking_point_cnt = params.checking_point_cnt;
    this->max_z_change_m_ = params.max_z_change_m;
    this->voxel_size_m = params.voxel_size_m;
    this->collision_padding_m = params.collision_padding_m;
    this->path_end_threshold_m = params.path_end_threshold_m;
    this->max_yaw_change_degrees = params.max_yaw_change_degrees;

    this->kLOcc_ = params.kLOcc_;
    this->kVoxSize_ = params.kVoxSize_;
    this->kViewingDistance_ = params.kViewingDistance_;
    this->kBboxLength_ = params.kBboxLength_;
    this->kBboxBreadth_ = params.kBboxBreadth_;
    this->kBboxHeight_ = params.kBboxHeight_;
    this->kNumViewpointsPerCluster_ = params.kNumViewpointsPerCluster_;
    this->kViewingDistanceInnerR_ = params.kViewingDistanceInnerR_;
    this->kViewingDistanceOuterR_ = params.kViewingDistanceOuterR_;
    this->kViewingDistanceZOffset_ = params.kViewingDistanceZOffset_;
    this->kCollCheckEndpointOffset_ = params.kCollCheckEndpointOffset_;
    this->kRobotModelBBoxFraction_ = params.kRobotModelBBoxFraction_;
    this->kViewpBBoxUnknownFracThresh_ = params.kViewpBBoxUnknownFracThresh_;
    this->kViewpBBoxOccupiedFracThresh_ = params.kViewpBBoxOccupiedFracThresh_;

    this->too_close_distance_ = params.too_close_distance;
    this->too_close_penalty_ = params.too_close_penalty;
    this->odometry_match_distance_ = params.odometry_match_distance;
    this->odometry_match_angle_ = params.odometry_match_angle;
    this->odometry_match_penalty_ = params.odometry_match_penalty;
    this->momentum_collision_check_distance_ = params.momentum_collision_check_distance;
    this->momentum_collision_check_distance_min_ = params.momentum_collision_check_distance_min;
    this->momentum_change_max_reward_ = params.momentum_change_max_reward;
    this->momentum_change_collision_reward_ = params.momentum_change_collision_reward;
    this->viewpoint_neighborhood_too_close_radius_ = params.viewpoint_neighborhood_too_close_radius;
    this->viewpoint_neighborhood_sphere_radius_ = params.viewpoint_neighborhood_sphere_radius;
    this->viewpoint_neighborhood_count_ = params.viewpoint_neighborhood_count;
    this->momentum_minimum_distance_ = params.momentum_minimum_distance;
    this->momentum_time_ = params.momentum_time;
    this->momentum_collision_check_step_size_ = params.momentum_collision_check_step_size;

    this->planner_norm_limit_ = params.planner_norm_limit_;
    this->max_explore_dist_ = params.max_explore_dist_;
    this->planner_max_iter_ = params.planner_max_iter_;
    this->max_connection_iter_ = params.max_connection_iter_;
    this->traj_smooth_horizon_ = params.traj_smooth_horizon_;
    this->rrt_region_nodes_count_ = params.rrt_region_nodes_count_;
    this->rrt_region_nodes_radius_ = params.rrt_region_nodes_radius_;
    this->rrt_region_nodes_z_layers_count_up_ = params.rrt_region_nodes_z_layers_count_up_;
    this->rrt_region_nodes_z_layers_step_ = params.rrt_region_nodes_z_layers_step_;

    this->priority_level_thred_ = params.priority_level_thred_;
    this->dense_step_ = params.dense_step_;

    viewp_sample_ = std::make_shared<viewpoint_sampling_ns::ViewpointSampling>(clustered_points_,
                                                                               kViewingDistance_,
                                                                               kBboxLength_,
                                                                               kBboxBreadth_,
                                                                               kBboxHeight_,
                                                                               kLOcc_,
                                                                               kVoxSize_,
                                                                               uint16_t(kNumViewpointsPerCluster_),
                                                                               kViewingDistanceInnerR_,
                                                                               kViewingDistanceOuterR_,
                                                                               kViewingDistanceZOffset_,
                                                                               kCollCheckEndpointOffset_,
                                                                               kRobotModelBBoxFraction_,
                                                                               kViewpBBoxUnknownFracThresh_,
                                                                               kViewpBBoxOccupiedFracThresh_);

    collision_checker_.init(kVoxSize_,
                            kBboxLength_,
                            kBboxBreadth_,
                            kBboxHeight_,
                            kLOcc_,
                            kViewpBBoxUnknownFracThresh_,
                            kViewpBBoxOccupiedFracThresh_);

    rrt_planner_.RRT_Init(planner_norm_limit_,
                          max_explore_dist_,
                          planner_max_iter_,
                          max_connection_iter_,
                          traj_smooth_horizon_,
                          rrt_region_nodes_count_,
                          rrt_region_nodes_radius_,
                          rrt_region_nodes_z_layers_count_up_,
                          rrt_region_nodes_z_layers_step_,
                          dense_step_);
}

std::optional<Path> ExplorationPlanner::plan_to_given_waypoint(const ViewPoint& start_point, const ViewPoint& goal_point)
{
    Path output_path;
    bool is_traj = false;

    is_traj = rrt_planner_.build_RRT(start_point, goal_point, collision_checker_, true);

    // success
    if (is_traj)
    {
        PointSet traj = rrt_planner_.getPath();
        output_path.reserve(traj.size());

        // convert to our path format
        for (const auto &vp : traj)
        {
            float yaw_val;
            if (vp.orientation_set)
            {
                // Convert quaternion to yaw
                tf2::Quaternion q(vp.orientation.x, vp.orientation.y, vp.orientation.z, vp.orientation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                yaw_val = static_cast<float>(yaw);
            }
            else
            {
                yaw_val = static_cast<float>(vp.orientation_yaw); // fallback if yaw is already set
            }

            output_path.emplace_back(static_cast<float>(vp.x),
                                        static_cast<float>(vp.y),
                                        static_cast<float>(vp.z),
                                        yaw_val);
        }

        RCLCPP_WARN(rclcpp::get_logger("exploration_planner"), "RRT Path generated");

        return output_path;
    }

    RCLCPP_WARN(rclcpp::get_logger("exploration_planner"),
                "No RRT path created");
    
    return output_path;
}

std::optional<Path> ExplorationPlanner::select_viewpoint_and_plan(const ViewPoint& start_point, float timeout_duration)
{
    RCLCPP_WARN(rclcpp::get_logger("exploration_planner"), "Start planning");
    robot_pos_ = start_point;

    this->sampled_viewpoints.clear();
    this->sampled_viewpoints.reserve(this->viewp_sample_->viewpoint_list_->size());

    for (const auto &p : *this->viewp_sample_->viewpoint_list_)
    {
        this->sampled_viewpoints.push_back(pclHSVtoPoint(p));
    }

    RCLCPP_WARN(rclcpp::get_logger("exploration_planner"), "Computing Viewpoint Scores");

    for (int viewpoint_index = 0; viewpoint_index < this->sampled_viewpoints.size(); viewpoint_index++)
    {
        double euclidean_distance = distancePoint2Point(start_point, this->sampled_viewpoints[viewpoint_index]);
        double momentum_change_reward = 0;
        double momentum_change_angle = angleBetweenHeadingAndMomentum(this->sampled_viewpoints[viewpoint_index]);
        momentum_change_reward = (std::cos(momentum_change_angle) + 1.0) / 2.0; // 0 (behind) and 1 (in front)
        double momentum_change_reward_before_collision_check = momentum_change_reward;

        if ((euclidean_distance >= momentum_collision_check_distance_min_ && euclidean_distance <= momentum_collision_check_distance_) &&
            (!collision_checker_.collisionCheckInFreeSpaceVector(start_point, this->sampled_viewpoints[viewpoint_index])))
        {
            // Line of sight collision free, therefore add momentum reward
            momentum_change_reward *= momentum_change_max_reward_;
        }
        else
        {
            // Line of sight blocked, therefore just add a small reward
            // since path planning will likely require changing directions anyway
            momentum_change_reward *= momentum_change_collision_reward_;
        }
        // score from viewpoint sampler
        double map_processor_score = this->sampled_viewpoints[viewpoint_index].viewpoint_score;

        double score = -euclidean_distance + momentum_change_reward + map_processor_score;

        // Heavily penalize if viewpoint is too close to current location or lookahead point
        if (euclidean_distance < too_close_distance_)
        {
            score -= too_close_penalty_ - 2.0 * euclidean_distance;
        }

        // Similarly, reward if there's lots of viewpoints in cylinder between planning point and viewpoint
        float cylinder_reward = 0.0;
        if (euclidean_distance >= too_close_distance_)
        {
            cylinder_reward = getCylinderReward(viewpoint_index, start_point) * (0.25 + 0.75 * momentum_change_reward_before_collision_check);
        }
        score += cylinder_reward;

        // Heavily penalize if robot has already been to this viewpoint
        bool already_been_to_viewpoint = doesPointMatchTrajectory(this->sampled_viewpoints[viewpoint_index]);
        if (already_been_to_viewpoint)
        {
            // ROS_ERROR("Already been to this viewpoint!");
            score -= odometry_match_penalty_;
        }

        if ((!this->sampled_viewpoints[viewpoint_index].score_initialized) || (score > this->sampled_viewpoints[viewpoint_index].score))
        {
            this->sampled_viewpoints[viewpoint_index].score = score;
            this->sampled_viewpoints[viewpoint_index].score_initialized = true;

            // Ignore the viewpoint if the map_processor score is not 200 (unobserved by other robots) or 150 (frontier of other robot)
            // Also ignore if this robot has already been near it
            if (map_processor_score < 100 || already_been_to_viewpoint == true)
            // if (already_been_to_viewpoint)
            {
                this->sampled_viewpoints[viewpoint_index].cleared_by_other_robot = true;
            }
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("exploration_planner"), "RRT Start");
    // RRT to generate path
    Path output_path;
    bool is_traj = false;

    if (this->sampled_viewpoints.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_planner"),
                    "sampled_viewpoints empty");
    }

    for (int priority_level = 0; priority_level < priority_level_thred_ && priority_level < this->sampled_viewpoints.size(); priority_level++)
    {
        goalSelector(priority_level);
        is_traj = rrt_planner_.build_RRT(start_point, explore_goal_, collision_checker_, true);

        // success
        if (is_traj)
        {
            PointSet traj = rrt_planner_.getPath();
            output_path.reserve(traj.size());

            // convert to our path format
            for (const auto &vp : traj)
            {
                float yaw_val;
                if (vp.orientation_set)
                {
                    // Convert quaternion to yaw
                    tf2::Quaternion q(vp.orientation.x, vp.orientation.y, vp.orientation.z, vp.orientation.w);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    yaw_val = static_cast<float>(yaw);
                }
                else
                {
                    yaw_val = static_cast<float>(vp.orientation_yaw); // fallback if yaw is already set
                }

                output_path.emplace_back(static_cast<float>(vp.x),
                                         static_cast<float>(vp.y),
                                         static_cast<float>(vp.z),
                                         yaw_val);
            }

            RCLCPP_WARN(rclcpp::get_logger("exploration_planner"), "RRT Path generated");

            return output_path;
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("exploration_planner"),
                "No RRT path created");
    return output_path;
}

void ExplorationPlanner::goalSelector(int priority_level)
{
    // INPUT : viewpoint_cloud, Graph, robot_pos_;
    // OUTPUT: explore_goal_;

    if (priority_level == 0)
    {
        // assignAwardToFrontiers();
        std::sort(this->sampled_viewpoints.begin(), this->sampled_viewpoints.end(), compareFrontier);
    }

    if (priority_level < this->sampled_viewpoints.size())
    {
        explore_goal_ = this->sampled_viewpoints[priority_level];
        explore_goal_cleared_by_other_robot_ = this->sampled_viewpoints[priority_level].cleared_by_other_robot;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("exploration_planner"), "priority level > num of sampled viewpoints");
    }
}

bool ExplorationPlanner::compareFrontier(const ViewPoint &p1, const ViewPoint &p2)
{
    // Using this comparitor will sort from largest to smallest score
    if (p1.score > p2.score)
    {
        return true;
    }
    return false;
}

bool ExplorationPlanner::doesPointMatchTrajectory(const ViewPoint &point)
{
    // Does the point match any point in executed_trajectory_
    // within given tolerances

    double distance, angle;

    // Iterate through the executed trajectory, looking for a match
    for (const ViewPoint &executed_point : executed_trajectory_)
    {

        // Check if it matches
        distance = distancePoint2Point(executed_point, point);
        if (distance <= odometry_match_distance_)
        {

            angle = angleDiffPoint2Point(executed_point, point);
            if (angle >= -odometry_match_angle_ && angle <= odometry_match_angle_)
            {
                // Match found!
                return true;
            }
        }
    }
    return false;
}

bool ExplorationPlanner::extractCylinderViewpointDistances(
    const int &viewpoint_index, const ViewPoint &planning_point, std::vector<float> &viewpoint_distances)
{
    viewpoint_distances.clear();

    double distance_pp_to_v = distancePoint2Point(planning_point, this->sampled_viewpoints[viewpoint_index]);

    // Only reward viewpoints that are between too_close_distance_ and viewpoint_neighborhood_too_close_radius_
    // distance from planning point
    if ((distance_pp_to_v > viewpoint_neighborhood_too_close_radius_) || (distance_pp_to_v < too_close_distance_))
    {
        return false;
    }

    // Count how many neighbors
    int count = 0;

    // Iterate through all viewpoints
    // INCLUDING itself
    for (int i = 0; i < this->sampled_viewpoints.size(); ++i)
    {
        double distance_pp_to_viewpoint = distancePoint2Point(planning_point, this->sampled_viewpoints[i]);

        double distance_odom_to_viewpoint = distancePoint2Point(robot_pos_, this->sampled_viewpoints[i]);

        // If this viewpoint is closer than the subject viewpoint but is not too_close to planning point
        // Then this *might* be a neighbor
        if ((distance_pp_to_viewpoint <= distance_pp_to_v + 0.01) && (distance_pp_to_viewpoint > too_close_distance_) && (distance_odom_to_viewpoint > too_close_distance_))
        {

            // Extract the cylinder distances
            double parallel_distance, perpendicular_distance;
            bool on_line_seg = lineSegDistances(this->sampled_viewpoints[i], planning_point, this->sampled_viewpoints[viewpoint_index],
                                                parallel_distance, perpendicular_distance);

            // Is on line segment through center of cylinder
            if (on_line_seg)
            {

                // Check if in 45deg cone from planning_point
                if (perpendicular_distance <= parallel_distance)
                {

                    // Check if within cylinder
                    if (perpendicular_distance <= viewpoint_neighborhood_sphere_radius_)
                    {
                        // Then viewpoint is considered a "neighbor"
                        ++count;
                        viewpoint_distances.push_back(parallel_distance);
                    }
                }
            }
        }
    }

    // Reward this viewpoint if lots of neighboring viewpoints found
    return count >= viewpoint_neighborhood_count_;
}

float ExplorationPlanner::getCylinderReward(const int &viewpoint_index, const ViewPoint &planning_point)
{
    // Find if there's a consecutive set of viewpoints in cylinder between planning_point and viewpoint
    double distance_pp_to_v = distancePoint2Point(planning_point, this->sampled_viewpoints[viewpoint_index]);

    // Extract the distances from planning_point for all viewpoints in cylinder
    std::vector<float> viewpoint_distances;
    bool has_viewpoint_neighborhood = extractCylinderViewpointDistances(viewpoint_index, planning_point, viewpoint_distances);

    if (!has_viewpoint_neighborhood)
    {
        // no reward
        return 0.0;
    }

    // Process these distances

    // Sort in DESCENDING order
    std::sort(viewpoint_distances.begin(), viewpoint_distances.end(), std::greater<float>());

    // Search through the distance in descending order
    // Looking for a "gap"
    float gap_start = 0;
    int count_neighbors = 0;
    for (int i = 1; i < viewpoint_distances.size(); i++)
    {
        float previous = viewpoint_distances[i - 1];
        float current = viewpoint_distances[i];

        float gap = previous - current; // previous is greater than current

        if (gap > 3.0)
        {
            // Gap found
            if (i < viewpoint_neighborhood_count_)
            {
                // No reward
                return 0.0;
            }

            // Stop searching for gap
            gap_start = previous;
            count_neighbors = i;
            break;
        }
    }

    return (viewpoint_distances[0] - gap_start) + 0.1 * (float)count_neighbors;
}

bool lineSegDistances(const ViewPoint &point, const ViewPoint &line_segment_start, const ViewPoint &line_segment_end,
                      double &parallel_distance, double &perpendicular_distance)
{
    // code adapted from http://geomalgorithms.com/a02-_lines.html
    // Return false if point is outside of line segment
    // Otherwise return true, along with the parallel and perpendicular distances

    // vector end to start
    double v_x = line_segment_end.x - line_segment_start.x;
    double v_y = line_segment_end.y - line_segment_start.y;
    double v_z = line_segment_end.z - line_segment_start.z;

    // vector start to point
    double w_x = point.x - line_segment_start.x;
    double w_y = point.y - line_segment_start.y;
    double w_z = point.z - line_segment_start.z;

    // if outside one boundary, return false
    double c1 = v_x * w_x + v_y * w_y + v_z * w_z; // dot product
    if (c1 <= 0)
        return false;

    // if outside other boundary, return false
    double c2 = v_x * v_x + v_y * v_y + v_z * v_z; // dot product
    if (c2 <= c1)
        return false;

    // otherwise project point and get distance (seems inefficient?)
    double b = c1 / c2;
    ViewPoint point_projected;
    point_projected.x = line_segment_start.x + b * v_x;
    point_projected.y = line_segment_start.y + b * v_y;
    point_projected.z = line_segment_start.z + b * v_z;

    perpendicular_distance = distancePoint2Point(point, point_projected);
    parallel_distance = distancePoint2Point(line_segment_start, point_projected);
    return true;
}

double ExplorationPlanner::angleBetweenHeadingAndMomentum(const ViewPoint &point)
{
    // Get the momentum vector
    double momentum_direction_x, momentum_direction_y, momentum_direction_z;
    bool momentum_computed = computeMomentumVector(momentum_direction_x, momentum_direction_y, momentum_direction_z);

    if (!momentum_computed)
    {
        // No momentum -- all viewpoints will be equally weighted in terms of momentum
        return 0.0;
    }

    // vector to point
    double diff_x = point.x - robot_pos_.x;
    double diff_y = point.y - robot_pos_.y;
    double diff_z = point.z - robot_pos_.z;
    double diff_norm = std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    // avoid singularity
    if (diff_norm < 0.01)
    {
        return 0.0;
    }

    // normalise
    diff_x /= diff_norm;
    diff_y /= diff_norm;
    diff_z /= diff_norm;

    // dot product
    double dot_product = diff_x * momentum_direction_x + diff_y * momentum_direction_y + diff_z * momentum_direction_z;

    // cos(theta) = a.b / |a||b|
    double relative_angle = std::acos(dot_product);
    return relative_angle;
}

bool ExplorationPlanner::computeMomentumVector(double &momentum_direction_x, double &momentum_direction_y, double &momentum_direction_z)
{
    // return false if no momentum computed
    // momentum returned as a normalised (x,y) vector
    // for now, assume z direction is 0, to encourage staying at same z level

    // Need at least two points to estimate momentum
    if (this->momentum_executed_trajectory_.size() <= 1)
    {
        return false;
    }

    // Get the start and end point of the vector
    const ViewPoint &current_point = this->momentum_executed_trajectory_.back().point;
    const ViewPoint &previous_point = this->momentum_executed_trajectory_.front().point;

    // Check not stationary first
    double euclidean_distance = distancePoint2Point(current_point, previous_point);
    if (euclidean_distance <= momentum_minimum_distance_)
    {
        return false;
    }

    // Compute the momentum vector
    momentum_direction_x = current_point.x - previous_point.x;
    momentum_direction_y = current_point.y - previous_point.y;
    momentum_direction_z = 0.0;

    // Normalise
    double norm = std::sqrt(momentum_direction_x * momentum_direction_x + momentum_direction_y * momentum_direction_y + momentum_direction_z * momentum_direction_z);

    // avoid singularity (might happen since the above two distance measures are slightly different)
    if (norm < 0.01)
    {
        return false;
    }

    // normalise
    momentum_direction_x /= norm;
    momentum_direction_y /= norm;
    momentum_direction_z /= norm;

    return true;
}

// Junbin: Below are previous implementation to remove

std::optional<Path> ExplorationPlanner::generate_straight_rand_path(
    std::tuple<float, float, float, float> start_point, float timeout_duration)
{
    // Locking mutex to prevent crashing when access the voxel map
    std::optional<Path> path;
    bool is_goal_point_valid = false;
    // get start ti
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    // RCLCPP_INFO(rclcpp::get_logger("exploration_planner"), "Starting Path Search...");

    while (!is_goal_point_valid && std::chrono::duration_cast<std::chrono::seconds>(
                                       std::chrono::steady_clock::now() - start_time)
                                           .count() < timeout_duration)
    {
        std::tuple<float, float, float> goal_point = generate_goal_point(start_point);
        // RCLCPP_INFO(rclcpp::get_logger("exploration_planner"), "Point Generated...");
        if (std::get<2>(goal_point) == -1)
        {
            RCLCPP_INFO(rclcpp::get_logger("exploration_planner"),
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
                RCLCPP_INFO(rclcpp::get_logger("exploration_planner"), "Exploration: Collision detected");
                is_goal_point_valid = false;
                break;
            }
            is_goal_point_valid = true;
        }
    }
    if (!is_goal_point_valid)
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_planner"), "No Path found in time limit");
        return std::nullopt;
    }
    else
    {
        return path;
    }
}

bool ExplorationPlanner::check_if_collided_single_voxel(
    const std::tuple<float, float, float> &point, const std::tuple<float, float, float> &voxel_center)
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

bool ExplorationPlanner::check_if_collided(const std::tuple<float, float, float> &point)
{
    // make sure the point is within the bounds -30 to 30
    if (std::get<0>(point) > 30 || std::get<0>(point) < -30 || std::get<1>(point) > 30 ||
        std::get<1>(point) < -30 || std::get<2>(point) > 30 || std::get<2>(point) < -30)
    {
        return true;
    }

    std::lock_guard<std::mutex> lock(this->mutex);
    for (const std::tuple<float, float, float> &voxel : this->voxel_points)
    {
        if (voxel == std::tuple<float, float, float>(0, 0, 0))
        {
            RCLCPP_INFO(rclcpp::get_logger("exploration_planner"), "Voxel is 0,0,0");
        }
        if (check_if_collided_single_voxel(point, voxel))
        {
            return true;
        }
    }
    return false;
}

std::tuple<float, float, float> ExplorationPlanner::generate_goal_point(
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
        rand_z = std::max(0.5f, rand_z); // ensure don't go below the ground
        std::tuple<float, float, float> rand_point(rand_x, rand_y, rand_z);
        std::tuple<float, float, float> start_point_wo_yaw(
            std::get<0>(start_point), std::get<1>(start_point), std::get<2>(start_point));
        float dist = get_point_distance(start_point_wo_yaw, rand_point);
        float new_angle = std::atan2(std::get<1>(rand_point) - std::get<1>(start_point_wo_yaw),
                                     std::get<0>(rand_point) - std::get<0>(start_point_wo_yaw));
        float angle_diff = std::abs(std::get<3>(start_point) - new_angle);
        // if the z value of the point is low enough
        if (angle_diff <= this->max_yaw_change_degrees * 3.14159265359 / 180)
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

double get_point_distance(const std::tuple<float, float, float> &point1,
                          const std::tuple<float, float, float> &point2)
{
    float x_diff = std::get<0>(point1) - std::get<0>(point2);
    float y_diff = std::get<1>(point1) - std::get<1>(point2);
    float z_diff = std::get<2>(point1) - std::get<2>(point2);
    float dist = std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    if (std::isnan(dist) || std::isinf(dist))
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_planner"), "Distance is nan or inf");
    }
    return dist;
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }

double rad2deg(double rad) { return rad * 180.0 / M_PI; }
