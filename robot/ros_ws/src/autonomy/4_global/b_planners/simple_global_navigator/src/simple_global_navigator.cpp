#include "simple_global_navigator/simple_global_navigator.hpp"

void SimpleGlobalNavigator::cost_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    cost_map_data_.points.clear();
    cost_map_data_.costs.clear();
    
    // Check if intensity field exists
    bool has_intensity = false;
    for (const auto& field : msg->fields) {
        if (field.name == "intensity") {
            has_intensity = true;
            break;
        }
    }
    
    // Extract points and optionally intensity values from PointCloud2
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    bool first_point = true;
    
    if (has_intensity) {
        // Use intensity field for costs
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            geometry_msgs::msg::Point point;
            point.x = *iter_x;
            point.y = *iter_y;
            point.z = *iter_z;
            
            cost_map_data_.points.push_back(point);
            cost_map_data_.costs.push_back(*iter_intensity);
            
            // Update bounds
            if (first_point) {
                cost_map_data_.min_bounds = point;
                cost_map_data_.max_bounds = point;
                first_point = false;
            } else {
                cost_map_data_.min_bounds.x = std::min(cost_map_data_.min_bounds.x, point.x);
                cost_map_data_.min_bounds.y = std::min(cost_map_data_.min_bounds.y, point.y);
                cost_map_data_.min_bounds.z = std::min(cost_map_data_.min_bounds.z, point.z);
                cost_map_data_.max_bounds.x = std::max(cost_map_data_.max_bounds.x, point.x);
                cost_map_data_.max_bounds.y = std::max(cost_map_data_.max_bounds.y, point.y);
                cost_map_data_.max_bounds.z = std::max(cost_map_data_.max_bounds.z, point.z);
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Using intensity field for cost values");
    } else {
        // Use default cost of 1.0 for all points
        const float default_cost = 1.0f;
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            geometry_msgs::msg::Point point;
            point.x = *iter_x;
            point.y = *iter_y;
            point.z = *iter_z;
            
            cost_map_data_.points.push_back(point);
            cost_map_data_.costs.push_back(default_cost);
            
            // Update bounds
            if (first_point) {
                cost_map_data_.min_bounds = point;
                cost_map_data_.max_bounds = point;
                first_point = false;
            } else {
                cost_map_data_.min_bounds.x = std::min(cost_map_data_.min_bounds.x, point.x);
                cost_map_data_.min_bounds.y = std::min(cost_map_data_.min_bounds.y, point.y);
                cost_map_data_.min_bounds.z = std::min(cost_map_data_.min_bounds.z, point.z);
                cost_map_data_.max_bounds.x = std::max(cost_map_data_.max_bounds.x, point.x);
                cost_map_data_.max_bounds.y = std::max(cost_map_data_.max_bounds.y, point.y);
                cost_map_data_.max_bounds.z = std::max(cost_map_data_.max_bounds.z, point.z);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "No intensity field found, using default cost of %.1f for all points", default_cost);
    }
    
    cost_map_data_.valid = !cost_map_data_.points.empty();
    
    if (cost_map_data_.valid) {
        RCLCPP_INFO(this->get_logger(), "Received cost map with %zu points", cost_map_data_.points.size());
    }
}

void SimpleGlobalNavigator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_odom_ = *msg;
    if (!odom_received_) {
        odom_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First odometry message received");
    }
}

rclcpp_action::GoalResponse SimpleGlobalNavigator::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigationTask::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "Received goal request with %zu poses", goal->goal_poses.size());
    
    if (goal->goal_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty goal poses list");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (!cost_map_data_.valid) {
        RCLCPP_WARN(this->get_logger(), "No valid cost map available");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!odom_received_) {
            RCLCPP_WARN(this->get_logger(), "No odometry data received yet, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimpleGlobalNavigator::handle_cancel(
    const std::shared_ptr<GoalHandleNavigationTask> action_handle) {
    
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)action_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SimpleGlobalNavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigationTask> action_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted, starting execution");
    
    using namespace std::placeholders;
    std::thread{std::bind(&SimpleGlobalNavigator::execute, this, _1), action_handle}.detach();
}

void SimpleGlobalNavigator::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigationTask>>& action_handle) {
    
    RCLCPP_INFO(this->get_logger(), "Executing navigation task");
    
    const auto action_goal = action_handle->get_goal();
    auto action_feedback = std::make_shared<NavigationTask::Feedback>();
    auto action_result = std::make_shared<NavigationTask::Result>();
    
    // Store goal poses
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_goal_poses_ = action_goal->goal_poses;
        current_goal_index_ = 0;
    }
    
    rclcpp::Rate loop_rate(10); // 10 Hz feedback rate
    auto start_time = this->now();
    
    // Check if max_planning_seconds is valid
    bool has_time_limit = action_goal->max_planning_seconds > 0.0;
    
    // Main execution loop
    while (rclcpp::ok() && current_goal_index_ < current_goal_poses_.size()) {
        if (action_handle->is_canceling()) {
            action_result->success = false;
            action_result->message = "Navigation cancelled";
            action_handle->canceled(action_result);
            RCLCPP_INFO(this->get_logger(), "Navigation cancelled");
            return;
        }
        
        // Check global time constraint
        if (has_time_limit) {
            double elapsed_time = (this->now() - start_time).seconds();
            if (elapsed_time >= action_goal->max_planning_seconds) {
                RCLCPP_WARN(this->get_logger(), 
                           "Navigation timeout reached (%.2fs), stopping at current progress", 
                           elapsed_time);
                break;
            }
        }
        
        // Get current position
        geometry_msgs::msg::Point current_pos;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_pos.x = current_odom_.pose.pose.position.x;
            current_pos.y = current_odom_.pose.pose.position.y;
            current_pos.z = current_odom_.pose.pose.position.z;
        }
        
        // Update current goal index based on proximity
        size_t new_goal_index = get_current_goal_index(current_pos);
        if (new_goal_index != current_goal_index_) {
            current_goal_index_ = new_goal_index;
            RCLCPP_INFO(this->get_logger(), "Advanced to goal %zu/%zu", 
                       current_goal_index_ + 1, current_goal_poses_.size());
        }
        
        // Plan path to current goal
        if (current_goal_index_ < current_goal_poses_.size()) {
            geometry_msgs::msg::Point goal_pos = current_goal_poses_[current_goal_index_].pose.position;
            
            // Calculate remaining planning time
            double elapsed_time = (this->now() - start_time).seconds();
            double remaining_planning_time = action_goal->max_planning_seconds - elapsed_time;
            
            auto path_poses = plan_rrt_star_path(current_pos, goal_pos, remaining_planning_time);
            
            if (!path_poses.empty()) {
                auto path_msg = create_path_message(path_poses);
                global_plan_publisher_->publish(path_msg);
                
                RCLCPP_DEBUG(this->get_logger(), "Published path with %zu poses", path_poses.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to plan path to goal %zu", current_goal_index_);
            }
        }
        
        // Publish feedback
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            action_feedback->current_pose.header.stamp = this->now();
            action_feedback->current_pose.header.frame_id = current_odom_.header.frame_id;
            action_feedback->current_pose.pose = current_odom_.pose.pose;
            action_feedback->distance_remaining = calculate_distance_remaining(current_pos);
            action_feedback->time_elapsed = (this->now() - start_time).seconds();
            action_feedback->current_phase = "navigating_to_goal_" + std::to_string(current_goal_index_);
        }
        
        action_handle->publish_feedback(action_feedback);
        
        loop_rate.sleep();
    }
    
    // Determine final result based on completion status
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        action_result->final_pose.header.stamp = this->now();
        action_result->final_pose.header.frame_id = current_odom_.header.frame_id;
        action_result->final_pose.pose = current_odom_.pose.pose;
        
        // Calculate total distance traveled (simplified)
        action_result->distance_traveled = 0.0;
        if (current_goal_poses_.size() > 1) {
            for (size_t i = 1; i < std::min(current_goal_index_ + 1, current_goal_poses_.size()); ++i) {
                action_result->distance_traveled += distance(
                    current_goal_poses_[i-1].pose.position,
                    current_goal_poses_[i].pose.position
                );
            }
        }
    }
    
    // Check completion status
    if (current_goal_index_ >= current_goal_poses_.size()) {
        // All goals completed successfully
        action_result->success = true;
        action_result->message = "Navigation completed successfully";
        action_handle->succeed(action_result);
        RCLCPP_INFO(this->get_logger(), "Navigation completed successfully");
    } else if (has_time_limit && (this->now() - start_time).seconds() >= action_goal->max_planning_seconds) {
        // Timeout reached, but we made some progress
        action_result->success = false;
        action_result->message = "Navigation timeout reached - completed " + 
                                std::to_string(current_goal_index_) + "/" + 
                                std::to_string(current_goal_poses_.size()) + " goals";
        action_handle->abort(action_result);
        RCLCPP_WARN(this->get_logger(), "Navigation timeout - completed %zu/%zu goals", 
                   current_goal_index_, current_goal_poses_.size());
    } else {
        // Other failure case
        action_result->success = false;
        action_result->message = "Navigation incomplete - completed " + 
                                std::to_string(current_goal_index_) + "/" + 
                                std::to_string(current_goal_poses_.size()) + " goals";
        action_handle->abort(action_result);
        RCLCPP_WARN(this->get_logger(), "Navigation incomplete - completed %zu/%zu goals", 
                   current_goal_index_, current_goal_poses_.size());
    }
}

std::vector<geometry_msgs::msg::PoseStamped> SimpleGlobalNavigator::plan_rrt_star_path(
    const geometry_msgs::msg::Point& start, 
    const geometry_msgs::msg::Point& goal,
    double max_planning_time) {
    
    if (!cost_map_data_.valid) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: no valid cost map");
        return {};
    }
    
    // Clear previous visualization markers
    clear_rrt_tree_markers();
    
    // Initialize RRT* tree
    std::vector<std::shared_ptr<RRTNode>> nodes;
    auto start_node = std::make_shared<RRTNode>(start, nullptr, 0.0);
    nodes.push_back(start_node);
    
    std::shared_ptr<RRTNode> best_goal_node = nullptr;
    auto planning_start_time = std::chrono::steady_clock::now();
    
    // Initial visualization
    if (enable_debug_visualization_) {
        publish_rrt_tree_markers(nodes, start, goal);
    }
    
    // RRT* main loop
    for (int i = 0; i < rrt_max_iterations_; ++i) {
        RCLCPP_INFO_STREAM(this->get_logger(), "RRT* iteration " << i);
        // Check time constraint if specified (negative values disable timeout)
        if (max_planning_time > 0.0) {
            auto elapsed_duration = std::chrono::steady_clock::now() - planning_start_time;
            double elapsed_time = std::chrono::duration<double>(elapsed_duration).count();
            if (elapsed_time >= max_planning_time) {
                RCLCPP_INFO(this->get_logger(), 
                           "RRT* planning timeout reached (%.2fs), returning best path found", 
                           elapsed_time);
                break;
            }
        }
        
        // Sample random point
        geometry_msgs::msg::Point rand_point = get_random_point();
        RCLCPP_INFO_STREAM(this->get_logger(), "RRT* sampled random point at: " << rand_point.x << ", " << rand_point.y << ", " << rand_point.z);

        // Find nearest node
        auto nearest_node = get_nearest_node(nodes, rand_point);
        if (!nearest_node) {
            continue;
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "RRT* found nearest node at: " << nearest_node->position.x << ", " << nearest_node->position.y << ", " << nearest_node->position.z);
        }
        
        // Steer towards random point
        geometry_msgs::msg::Point new_point = steer(nearest_node->position, rand_point, rrt_step_size_);
        RCLCPP_INFO_STREAM(this->get_logger(), "RRT* steering from: " << nearest_node->position.x << ", " << nearest_node->position.y << ", " << nearest_node->position.z << " to: " << new_point.x << ", " << new_point.y << ", " << new_point.z);
        
        // Debug visualization: show current sampled and steered points
        if (enable_debug_visualization_) {
            publish_rrt_tree_markers(nodes, start, goal, &rand_point, &new_point);
            // Small delay to make visualization easier to follow during debugging
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Check collision
        if (!is_collision_free(nearest_node->position, new_point)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "RRT* collision detected between: " << nearest_node->position.x << ", " << nearest_node->position.y << ", " << nearest_node->position.z << " and: " << new_point.x << ", " << new_point.y << ", " << new_point.z);
            // Show collision case in visualization (sampled and steered points still visible)
            if (enable_debug_visualization_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Brief pause to see collision case
            }
            continue;
        }
        
        // Create new node
        double new_cost = nearest_node->cost + distance(nearest_node->position, new_point) + 
                         get_cost_at_point(new_point);
        auto new_node = std::make_shared<RRTNode>(new_point, nearest_node, new_cost);
        RCLCPP_INFO_STREAM(this->get_logger(), "RRT* created new node at: " << new_point.x << ", " << new_point.y << ", " << new_point.z << " with cost: " << new_cost);
        
        // Find near nodes for rewiring
        auto near_nodes = get_near_nodes(nodes, new_point, rrt_rewire_radius_);
        RCLCPP_INFO_STREAM(this->get_logger(), "RRT* found " << near_nodes.size() << " near nodes for rewiring");
        
        // Choose parent with minimum cost
        for (auto& near_node : near_nodes) {
            if (is_collision_free(near_node->position, new_point)) {
                double potential_cost = near_node->cost + distance(near_node->position, new_point) + 
                                      get_cost_at_point(new_point);
                if (potential_cost < new_cost) {
                    new_node->parent = near_node;
                    new_cost = potential_cost;
                    new_node->cost = new_cost;
                }
            }
        }
        
        // Add new node to tree
        nodes.push_back(new_node);
        new_node->parent->children.push_back(new_node);
        RCLCPP_INFO_STREAM(this->get_logger(), "New tree size: " << nodes.size());

        // Rewire tree
        rewire(new_node, near_nodes);
        
        // Update visualization after adding new node (show final state of this iteration)
        if (enable_debug_visualization_) {
            publish_rrt_tree_markers(nodes, start, goal);
            // Small delay to make visualization easier to follow during debugging
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Check if we reached the goal
        if (distance(new_point, goal) <= rrt_goal_tolerance_) {
            if (!best_goal_node || new_cost < best_goal_node->cost) {
                best_goal_node = new_node;
                RCLCPP_INFO(this->get_logger(), 
                           "Found improved path to goal with cost: %.2f (iteration %d)", 
                           new_cost, i);
                
                // Update visualization when goal is found
                if (enable_debug_visualization_) {
                    publish_rrt_tree_markers(nodes, start, goal);
                }
            }
        }
    }
    
    // Final visualization update
    if (enable_debug_visualization_) {
        publish_rrt_tree_markers(nodes, start, goal);
    }
    
    // Extract path if goal was reached
    if (best_goal_node) {
        auto elapsed_duration = std::chrono::steady_clock::now() - planning_start_time;
        double planning_time = std::chrono::duration<double>(elapsed_duration).count();
        RCLCPP_INFO(this->get_logger(), 
                   "RRT* found path with cost: %.2f in %.2fs", 
                   best_goal_node->cost, planning_time);
        return extract_path(best_goal_node);
    } else {
        // If no goal was reached, try to find the closest node to goal
        std::shared_ptr<RRTNode> closest_node = nullptr;
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& node : nodes) {
            double dist = distance(node->position, goal);
            if (dist < min_distance) {
                min_distance = dist;
                closest_node = node;
            }
        }
        
        if (closest_node && min_distance < rrt_goal_tolerance_ * 3.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "RRT* couldn't reach goal exactly, returning path to closest point (%.2fm away)", 
                       min_distance);
            return extract_path(closest_node);
        } else {
            RCLCPP_WARN_STREAM(this->get_logger(), "RRT* failed to find any viable path to goal. min_distance: " << min_distance << ", tolerance: " << rrt_goal_tolerance_);
            return {};
        }
    }
}

std::shared_ptr<RRTNode> SimpleGlobalNavigator::get_nearest_node(
    const std::vector<std::shared_ptr<RRTNode>>& nodes, 
    const geometry_msgs::msg::Point& point) {
    
    if (nodes.empty()) {
        return nullptr;
    }
    
    std::shared_ptr<RRTNode> nearest = nodes.at(0);
    double min_dist = distance(nearest->position, point);
    
    for (const auto& node : nodes) {
        double dist = distance(node->position, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

geometry_msgs::msg::Point SimpleGlobalNavigator::steer(
    const geometry_msgs::msg::Point& from, 
    const geometry_msgs::msg::Point& to, 
    double step_size) {
    
    double dist = distance(from, to);
    if (dist <= step_size) {
        return to;
    }
    
    geometry_msgs::msg::Point result;
    result.x = from.x + (to.x - from.x) * step_size / dist;
    result.y = from.y + (to.y - from.y) * step_size / dist;
    result.z = from.z + (to.z - from.z) * step_size / dist;
    
    return result;
}

bool SimpleGlobalNavigator::is_collision_free(
    const geometry_msgs::msg::Point& from, 
    const geometry_msgs::msg::Point& to) {
    
    // Simple collision checking: sample points along the line and check costs
    int num_samples = static_cast<int>(distance(from, to) / (cost_map_data_.resolution * 0.5)) + 1;
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        geometry_msgs::msg::Point sample;
        sample.x = from.x + t * (to.x - from.x);
        sample.y = from.y + t * (to.y - from.y);
        sample.z = from.z + t * (to.z - from.z);
        
        double cost = get_cost_at_point(sample);
        if (cost > 0.8) { // High cost threshold indicates obstacle
            RCLCPP_INFO_STREAM(this->get_logger(), "Collision at sampled point: " << sample.x << ", " << sample.y << ", " << sample.z << " with cost: " << cost);
            return false;
        }
    }
    
    return true;
}

double SimpleGlobalNavigator::get_cost_at_point(const geometry_msgs::msg::Point& point) {
    if (!cost_map_data_.valid || cost_map_data_.points.empty()) {
        return 0.0;
    }
    
    // Find nearest cost map point (simple nearest neighbor)
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    
    for (size_t i = 0; i < cost_map_data_.points.size(); ++i) {
        double dist = distance(point, cost_map_data_.points[i]);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    return cost_map_data_.costs[nearest_idx];
}

double SimpleGlobalNavigator::distance(
    const geometry_msgs::msg::Point& p1, 
    const geometry_msgs::msg::Point& p2) {
    
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<std::shared_ptr<RRTNode>> SimpleGlobalNavigator::get_near_nodes(
    const std::vector<std::shared_ptr<RRTNode>>& nodes, 
    const geometry_msgs::msg::Point& point, 
    double radius) {
    
    std::vector<std::shared_ptr<RRTNode>> near_nodes;
    
    for (const auto& node : nodes) {
        if (distance(node->position, point) <= radius) {
            near_nodes.push_back(node);
        }
    }
    
    return near_nodes;
}

void SimpleGlobalNavigator::rewire(
    std::shared_ptr<RRTNode> new_node, 
    const std::vector<std::shared_ptr<RRTNode>>& near_nodes) {
    
    for (auto& near_node : near_nodes) {
        if (near_node == new_node || near_node == new_node->parent) {
            continue;
        }
        
        double new_cost = new_node->cost + distance(new_node->position, near_node->position);
        
        if (new_cost < near_node->cost && is_collision_free(new_node->position, near_node->position)) {
            // Remove near_node from its current parent's children
            if (near_node->parent) {
                auto& siblings = near_node->parent->children;
                siblings.erase(std::remove(siblings.begin(), siblings.end(), near_node), siblings.end());
            }
            
            // Set new parent
            near_node->parent = new_node;
            near_node->cost = new_cost;
            new_node->children.push_back(near_node);
            
            // Update costs of all descendants
            std::function<void(std::shared_ptr<RRTNode>)> update_descendants = 
                [&](std::shared_ptr<RRTNode> node) {
                    for (auto& child : node->children) {
                        child->cost = node->cost + distance(node->position, child->position) + 
                                     get_cost_at_point(child->position);
                        update_descendants(child);
                    }
                };
            update_descendants(near_node);
        }
    }
}

std::vector<geometry_msgs::msg::PoseStamped> SimpleGlobalNavigator::extract_path(
    std::shared_ptr<RRTNode> goal_node) {
    
    std::vector<geometry_msgs::msg::PoseStamped> path;
    std::shared_ptr<RRTNode> current = goal_node;
    
    // Trace back from goal to start
    while (current) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map"; // Assume map frame
        pose.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        pose.pose.position = current->position;
        
        // Set orientation (simplified - pointing towards next waypoint)
        pose.pose.orientation.w = 1.0;
        
        path.insert(path.begin(), pose);
        current = current->parent;
    }
    
    return path;
}

nav_msgs::msg::Path SimpleGlobalNavigator::create_path_message(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    path_msg.poses = poses;
    
    return path_msg;
}

geometry_msgs::msg::Point SimpleGlobalNavigator::get_random_point() {
    if (!cost_map_data_.valid) {
        geometry_msgs::msg::Point point;
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
        return point;
    }
    
    std::uniform_real_distribution<double> dist_x(cost_map_data_.min_bounds.x, cost_map_data_.max_bounds.x);
    std::uniform_real_distribution<double> dist_y(cost_map_data_.min_bounds.y, cost_map_data_.max_bounds.y);
    std::uniform_real_distribution<double> dist_z(cost_map_data_.min_bounds.z, cost_map_data_.max_bounds.z);
    
    geometry_msgs::msg::Point point;
    point.x = dist_x(rng_);
    point.y = dist_y(rng_);
    point.z = dist_z(rng_);
    
    return point;
}

size_t SimpleGlobalNavigator::get_current_goal_index(const geometry_msgs::msg::Point& current_pos) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (current_goal_poses_.empty()) {
        return 0;
    }
    
    // Find the closest unvisited goal
    size_t best_index = current_goal_index_;
    double min_dist = distance(current_pos, current_goal_poses_[current_goal_index_].pose.position);
    
    // Check if we're close enough to current goal to advance
    if (min_dist <= rrt_goal_tolerance_ && current_goal_index_ + 1 < current_goal_poses_.size()) {
        return current_goal_index_ + 1;
    }
    
    return current_goal_index_;
}

double SimpleGlobalNavigator::calculate_distance_remaining(const geometry_msgs::msg::Point& current_pos) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (current_goal_poses_.empty() || current_goal_index_ >= current_goal_poses_.size()) {
        return 0.0;
    }
    
    double total_distance = 0.0;
    
    // Distance to current goal
    total_distance += distance(current_pos, current_goal_poses_[current_goal_index_].pose.position);
    
    // Distance between remaining goals
    for (size_t i = current_goal_index_; i + 1 < current_goal_poses_.size(); ++i) {
        total_distance += distance(
            current_goal_poses_[i].pose.position,
            current_goal_poses_[i + 1].pose.position
        );
    }
    
    return total_distance;
}

void SimpleGlobalNavigator::publish_rrt_tree_markers(
    const std::vector<std::shared_ptr<RRTNode>>& nodes,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    const geometry_msgs::msg::Point* sampled_point,
    const geometry_msgs::msg::Point* steered_point) {
    
    if (!enable_debug_visualization_ || !rrt_tree_marker_publisher_) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "rrt_tree";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Create tree edges marker (LINE_LIST)
    visualization_msgs::msg::Marker edges_marker;
    edges_marker.header.frame_id = "map";
    edges_marker.header.stamp = this->now();
    edges_marker.ns = "rrt_tree";
    edges_marker.id = 0;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::msg::Marker::ADD;
    edges_marker.pose.orientation.w = 1.0;
    edges_marker.scale.x = 0.02; // Line width
    edges_marker.color.r = 0.0;
    edges_marker.color.g = 0.8;
    edges_marker.color.b = 1.0;
    edges_marker.color.a = 0.6;
    
    // Add edges to the marker
    for (const auto& node : nodes) {
        if (node->parent) {
            // Add parent position
            geometry_msgs::msg::Point parent_point = node->parent->position;
            edges_marker.points.push_back(parent_point);
            
            // Add current node position
            geometry_msgs::msg::Point current_point = node->position;
            edges_marker.points.push_back(current_point);
        }
    }
    
    if (!edges_marker.points.empty()) {
        marker_array.markers.push_back(edges_marker);
    }
    
    // Create nodes marker (SPHERE_LIST)
    visualization_msgs::msg::Marker nodes_marker;
    nodes_marker.header.frame_id = "map";
    nodes_marker.header.stamp = this->now();
    nodes_marker.ns = "rrt_tree";
    nodes_marker.id = 1;
    nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    nodes_marker.pose.orientation.w = 1.0;
    nodes_marker.scale.x = 0.05; // Sphere diameter
    nodes_marker.scale.y = 0.05;
    nodes_marker.scale.z = 0.05;
    nodes_marker.color.r = 0.2;
    nodes_marker.color.g = 1.0;
    nodes_marker.color.b = 0.2;
    nodes_marker.color.a = 0.8;
    
    // Add all node positions
    for (const auto& node : nodes) {
        nodes_marker.points.push_back(node->position);
    }
    
    if (!nodes_marker.points.empty()) {
        marker_array.markers.push_back(nodes_marker);
    }
    
    // Create start marker
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = this->now();
    start_marker.ns = "rrt_tree";
    start_marker.id = 2;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position = start;
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = 0.15;
    start_marker.scale.y = 0.15;
    start_marker.scale.z = 0.15;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;
    marker_array.markers.push_back(start_marker);
    
    // Create start text marker
    visualization_msgs::msg::Marker start_text;
    start_text.header.frame_id = "map";
    start_text.header.stamp = this->now();
    start_text.ns = "rrt_tree";
    start_text.id = 6;
    start_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    start_text.action = visualization_msgs::msg::Marker::ADD;
    start_text.pose.position = start;
    start_text.pose.position.z += 0.2; // Offset text above the marker
    start_text.pose.orientation.w = 1.0;
    start_text.scale.z = 0.1; // Text height
    start_text.color.r = 1.0;
    start_text.color.g = 1.0;
    start_text.color.b = 1.0;
    start_text.color.a = 1.0;
    start_text.text = "START";
    marker_array.markers.push_back(start_text);
    
    // Create goal marker
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = this->now();
    goal_marker.ns = "rrt_tree";
    goal_marker.id = 3;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position = goal;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.15;
    goal_marker.scale.y = 0.15;
    goal_marker.scale.z = 0.15;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;
    marker_array.markers.push_back(goal_marker);
    
    // Create goal text marker
    visualization_msgs::msg::Marker goal_text;
    goal_text.header.frame_id = "map";
    goal_text.header.stamp = this->now();
    goal_text.ns = "rrt_tree";
    goal_text.id = 7;
    goal_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    goal_text.action = visualization_msgs::msg::Marker::ADD;
    goal_text.pose.position = goal;
    goal_text.pose.position.z += 0.2; // Offset text above the marker
    goal_text.pose.orientation.w = 1.0;
    goal_text.scale.z = 0.1; // Text height
    goal_text.color.r = 1.0;
    goal_text.color.g = 1.0;
    goal_text.color.b = 1.0;
    goal_text.color.a = 1.0;
    goal_text.text = "GOAL";
    marker_array.markers.push_back(goal_text);
    
    // Create sampled point marker (if provided)
    if (sampled_point) {
        visualization_msgs::msg::Marker sampled_marker;
        sampled_marker.header.frame_id = "map";
        sampled_marker.header.stamp = this->now();
        sampled_marker.ns = "rrt_tree";
        sampled_marker.id = 4;
        sampled_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sampled_marker.action = visualization_msgs::msg::Marker::ADD;
        sampled_marker.pose.position = *sampled_point;
        sampled_marker.pose.orientation.w = 1.0;
        sampled_marker.scale.x = 0.12;
        sampled_marker.scale.y = 0.12;
        sampled_marker.scale.z = 0.12;
        sampled_marker.color.r = 1.0;
        sampled_marker.color.g = 1.0;
        sampled_marker.color.b = 0.0;  // Yellow for sampled point
        sampled_marker.color.a = 0.9;
        marker_array.markers.push_back(sampled_marker);
        
        // Create sampled point text marker
        visualization_msgs::msg::Marker sampled_text;
        sampled_text.header.frame_id = "map";
        sampled_text.header.stamp = this->now();
        sampled_text.ns = "rrt_tree";
        sampled_text.id = 8;
        sampled_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        sampled_text.action = visualization_msgs::msg::Marker::ADD;
        sampled_text.pose.position = *sampled_point;
        sampled_text.pose.position.z += 0.2; // Offset text above the marker
        sampled_text.pose.orientation.w = 1.0;
        sampled_text.scale.z = 0.08; // Text height
        sampled_text.color.r = 1.0;
        sampled_text.color.g = 1.0;
        sampled_text.color.b = 0.0;  // Yellow text to match marker
        sampled_text.color.a = 1.0;
        sampled_text.text = "SAMPLED";
        marker_array.markers.push_back(sampled_text);
    }
    
    // Create steered point marker (if provided)
    if (steered_point) {
        visualization_msgs::msg::Marker steered_marker;
        steered_marker.header.frame_id = "map";
        steered_marker.header.stamp = this->now();
        steered_marker.ns = "rrt_tree";
        steered_marker.id = 5;
        steered_marker.type = visualization_msgs::msg::Marker::SPHERE;
        steered_marker.action = visualization_msgs::msg::Marker::ADD;
        steered_marker.pose.position = *steered_point;
        steered_marker.pose.orientation.w = 1.0;
        steered_marker.scale.x = 0.10;
        steered_marker.scale.y = 0.10;
        steered_marker.scale.z = 0.10;
        steered_marker.color.r = 1.0;
        steered_marker.color.g = 0.5;
        steered_marker.color.b = 0.0;  // Orange for steered point
        steered_marker.color.a = 0.9;
        marker_array.markers.push_back(steered_marker);
        
        // Create steered point text marker
        visualization_msgs::msg::Marker steered_text;
        steered_text.header.frame_id = "map";
        steered_text.header.stamp = this->now();
        steered_text.ns = "rrt_tree";
        steered_text.id = 9;
        steered_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        steered_text.action = visualization_msgs::msg::Marker::ADD;
        steered_text.pose.position = *steered_point;
        steered_text.pose.position.z += 0.2; // Offset text above the marker
        steered_text.pose.orientation.w = 1.0;
        steered_text.scale.z = 0.08; // Text height
        steered_text.color.r = 1.0;
        steered_text.color.g = 0.5;
        steered_text.color.b = 0.0;  // Orange text to match marker
        steered_text.color.a = 1.0;
        steered_text.text = "STEERED";
        marker_array.markers.push_back(steered_text);
    }
    
    // Publish the markers
    rrt_tree_marker_publisher_->publish(marker_array);
}

void SimpleGlobalNavigator::clear_rrt_tree_markers() {
    if (!enable_debug_visualization_ || !rrt_tree_marker_publisher_) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "rrt_tree";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    rrt_tree_marker_publisher_->publish(marker_array);
}