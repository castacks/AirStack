#include "jps_global_navigator/jps_global_navigator.hpp"

void JPSGlobalNavigator::cost_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    cost_map_data_.points.clear();
    cost_map_data_.costs.clear();
    cost_map_data_.grid_costs.clear();
    
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
        build_grid_from_cost_map();
        RCLCPP_INFO(this->get_logger(), "Received cost map with %zu points, grid size: %dx%dx%d", 
                   cost_map_data_.points.size(), 
                   cost_map_data_.grid_size_x, cost_map_data_.grid_size_y, cost_map_data_.grid_size_z);
    }
}

void JPSGlobalNavigator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_odom_ = *msg;
    if (!odom_received_) {
        odom_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First odometry message received");
    }
}

rclcpp_action::GoalResponse JPSGlobalNavigator::handle_goal(
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

rclcpp_action::CancelResponse JPSGlobalNavigator::handle_cancel(
    const std::shared_ptr<GoalHandleNavigationTask> action_handle) {
    
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)action_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void JPSGlobalNavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigationTask> action_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted, starting execution");
    
    using namespace std::placeholders;
    std::thread{std::bind(&JPSGlobalNavigator::execute, this, _1), action_handle}.detach();
}

void JPSGlobalNavigator::execute(
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
            
            auto path_poses = plan_jps_path(current_pos, goal_pos, remaining_planning_time);
            
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
        action_result->success = true;
        action_result->message = "Navigation completed successfully";
        action_handle->succeed(action_result);
        RCLCPP_INFO(this->get_logger(), "Navigation completed successfully");
    } else if (has_time_limit && (this->now() - start_time).seconds() >= action_goal->max_planning_seconds) {
        action_result->success = false;
        action_result->message = "Navigation timeout reached - completed " + 
                                std::to_string(current_goal_index_) + "/" + 
                                std::to_string(current_goal_poses_.size()) + " goals";
        action_handle->abort(action_result);
        RCLCPP_WARN(this->get_logger(), "Navigation timeout - completed %zu/%zu goals", 
                   current_goal_index_, current_goal_poses_.size());
    } else {
        action_result->success = false;
        action_result->message = "Navigation incomplete - completed " + 
                                std::to_string(current_goal_index_) + "/" + 
                                std::to_string(current_goal_poses_.size()) + " goals";
        action_handle->abort(action_result);
        RCLCPP_WARN(this->get_logger(), "Navigation incomplete - completed %zu/%zu goals", 
                   current_goal_index_, current_goal_poses_.size());
    }
}

std::vector<geometry_msgs::msg::PoseStamped> JPSGlobalNavigator::plan_jps_path(
    const geometry_msgs::msg::Point& start, 
    const geometry_msgs::msg::Point& goal,
    double max_planning_time) {
    
    if (!cost_map_data_.valid) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: no valid cost map");
        return {};
    }
    
    // Clear previous visualization markers
    clear_jps_search_markers();
    
    // Convert world coordinates to grid coordinates
    GridCell start_cell = point_to_grid(start);
    GridCell goal_cell = point_to_grid(goal);
    
    RCLCPP_INFO(this->get_logger(), "Planning JPS path from (%d,%d,%d) to (%d,%d,%d)", 
               start_cell.x, start_cell.y, start_cell.z,
               goal_cell.x, goal_cell.y, goal_cell.z);
    
    // Check if start and goal are valid
    if (!is_valid_cell(start_cell) || !is_valid_cell(goal_cell)) {
        RCLCPP_WARN(this->get_logger(), "Start or goal position is outside valid bounds");
        return {};
    }
    
    if (is_obstacle(start_cell) || is_obstacle(goal_cell)) {
        RCLCPP_WARN(this->get_logger(), "Start or goal position is in obstacle");
        return {};
    }
    
    // Run JPS search
    auto planning_start_time = std::chrono::steady_clock::now();
    std::vector<GridCell> path_cells = jps_search(start_cell, goal_cell, max_planning_time);
    
    if (path_cells.empty()) {
        RCLCPP_WARN(this->get_logger(), "JPS failed to find path");
        return {};
    }
    
    auto elapsed_duration = std::chrono::steady_clock::now() - planning_start_time;
    double planning_time = std::chrono::duration<double>(elapsed_duration).count();
    
    RCLCPP_INFO(this->get_logger(), "JPS found path with %zu waypoints in %.2fs", 
               path_cells.size(), planning_time);
    
    // Convert grid path to world coordinates
    std::vector<geometry_msgs::msg::PoseStamped> raw_path;
    for (const auto& cell : path_cells) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";  // Assuming map frame
        pose.header.stamp = this->now();
        pose.pose.position = grid_to_point(cell);
        
        // Set orientation (could be improved to face direction of travel)
        pose.pose.orientation.w = 1.0;
        
        raw_path.push_back(pose);
    }
    
    // Apply path smoothing
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path = smooth_path(raw_path);
    
    RCLCPP_INFO(this->get_logger(), "Path smoothed from %zu to %zu waypoints", 
               raw_path.size(), smoothed_path.size());
    
    return smoothed_path;
}

std::vector<GridCell> JPSGlobalNavigator::jps_search(const GridCell& start, const GridCell& goal, double max_planning_time) {
    auto search_start_time = std::chrono::steady_clock::now();
    
    // Priority queue for open set
    std::priority_queue<std::shared_ptr<JPSNode>, std::vector<std::shared_ptr<JPSNode>>, JPSNodeComparator> open_set;
    
    // Maps for tracking nodes
    std::unordered_map<GridCell, std::shared_ptr<JPSNode>, GridCellHash> all_nodes;
    std::unordered_set<GridCell, GridCellHash> closed_set;
    
    // Create start node
    auto start_node = std::make_shared<JPSNode>(start, nullptr, 0.0, heuristic(start, goal));
    open_set.push(start_node);
    all_nodes[start] = start_node;
    
    std::vector<std::shared_ptr<JPSNode>> explored_nodes;
    int iterations = 0;
    
    while (!open_set.empty() && iterations < jps_max_iterations_) {
        iterations++;
        
        // Check time constraint
        if (max_planning_time > 0.0) {
            auto elapsed_duration = std::chrono::steady_clock::now() - search_start_time;
            double elapsed_time = std::chrono::duration<double>(elapsed_duration).count();
            if (elapsed_time >= max_planning_time) {
                RCLCPP_WARN(this->get_logger(), "JPS search timeout reached (%.2fs)", elapsed_time);
                break;
            }
        }
        
        // Get node with lowest f_cost
        auto current = open_set.top();
        open_set.pop();
        
        // Skip if already processed
        if (closed_set.find(current->cell) != closed_set.end()) {
            continue;
        }
        
        closed_set.insert(current->cell);
        explored_nodes.push_back(current);
        
        // Check if we reached the goal
        if (current->cell == goal) {
            RCLCPP_INFO(this->get_logger(), "JPS found goal in %d iterations", iterations);
            
            // Visualize final result
            if (enable_debug_visualization_) {
                std::vector<GridCell> final_path;
                auto node = current;
                while (node) {
                    final_path.insert(final_path.begin(), node->cell);
                    node = node->parent;
                }
                publish_jps_search_markers(explored_nodes, start, goal, final_path);
            }
            
            // Extract path
            std::vector<GridCell> path;
            auto node = current;
            while (node) {
                path.insert(path.begin(), node->cell);
                node = node->parent;
            }
            return path;
        }
        
        // Get direction from parent (if any)
        Direction parent_direction(0, 0, 0);
        if (current->parent) {
            parent_direction.dx = current->cell.x - current->parent->cell.x;
            parent_direction.dy = current->cell.y - current->parent->cell.y;
            parent_direction.dz = current->cell.z - current->parent->cell.z;
        }
        
        // Get natural neighbors based on parent direction
        std::vector<Direction> neighbors = get_natural_neighbors(current->cell, parent_direction);
        
        // Process each neighbor direction
        for (const auto& direction : neighbors) {
            auto jump_node = jump(current->cell, direction, goal);
            if (!jump_node) continue;
            
            // Skip if already in closed set
            if (closed_set.find(jump_node->cell) != closed_set.end()) {
                continue;
            }
            
            // Calculate costs
            double tentative_g = current->g_cost + heuristic(current->cell, jump_node->cell);
            
            // Check if this is a better path to this node
            auto existing_it = all_nodes.find(jump_node->cell);
            if (existing_it == all_nodes.end() || tentative_g < existing_it->second->g_cost) {
                jump_node->parent = current;
                jump_node->g_cost = tentative_g;
                jump_node->h_cost = heuristic(jump_node->cell, goal);
                jump_node->f_cost = jump_node->g_cost + jump_node->h_cost;
                
                all_nodes[jump_node->cell] = jump_node;
                open_set.push(jump_node);
            }
        }
        
        // Periodic visualization update
        if (enable_debug_visualization_ && iterations % 100 == 0) {
            publish_jps_search_markers(explored_nodes, start, goal);
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "JPS search failed after %d iterations", iterations);
    
    // Final visualization showing explored area
    if (enable_debug_visualization_) {
        publish_jps_search_markers(explored_nodes, start, goal);
    }
    
    return {};
}

std::shared_ptr<JPSNode> JPSGlobalNavigator::jump(const GridCell& current, const Direction& direction, const GridCell& goal) {
    GridCell next(current.x + direction.dx, current.y + direction.dy, current.z + direction.dz);
    
    // Check bounds and obstacles
    if (!is_valid_cell(next) || is_obstacle(next)) {
        return nullptr;
    }
    
    // Check if we reached the goal
    if (next == goal) {
        return std::make_shared<JPSNode>(next);
    }
    
    // Check for forced neighbors
    if (has_forced_neighbor(next, direction)) {
        return std::make_shared<JPSNode>(next);
    }
    
    // For diagonal movement, check if we can jump in component directions
    if (direction.dx != 0 && direction.dy != 0) {
        // Check horizontal component
        if (jump(next, Direction(direction.dx, 0, 0), goal)) {
            return std::make_shared<JPSNode>(next);
        }
        // Check vertical component
        if (jump(next, Direction(0, direction.dy, 0), goal)) {
            return std::make_shared<JPSNode>(next);
        }
    }
    
    // For 3D diagonal movement, also check z component
    if (direction.dz != 0) {
        if (jump(next, Direction(0, 0, direction.dz), goal)) {
            return std::make_shared<JPSNode>(next);
        }
    }
    
    // Continue jumping in the same direction
    return jump(next, direction, goal);
}

std::vector<Direction> JPSGlobalNavigator::get_natural_neighbors(const GridCell& current, const Direction& parent_direction) {
    std::vector<Direction> neighbors;
    
    // If no parent (start node), return all directions
    if (parent_direction.dx == 0 && parent_direction.dy == 0 && parent_direction.dz == 0) {
        return directions_;
    }
    
    // For straight movement, continue in same direction
    if ((parent_direction.dx != 0 && parent_direction.dy == 0 && parent_direction.dz == 0) ||
        (parent_direction.dx == 0 && parent_direction.dy != 0 && parent_direction.dz == 0) ||
        (parent_direction.dx == 0 && parent_direction.dy == 0 && parent_direction.dz != 0)) {
        neighbors.push_back(parent_direction);
    }
    // For diagonal movement, include the diagonal and component directions
    else {
        neighbors.push_back(parent_direction);
        if (parent_direction.dx != 0) {
            neighbors.push_back(Direction(parent_direction.dx, 0, 0));
        }
        if (parent_direction.dy != 0) {
            neighbors.push_back(Direction(0, parent_direction.dy, 0));
        }
        if (parent_direction.dz != 0) {
            neighbors.push_back(Direction(0, 0, parent_direction.dz));
        }
    }
    
    // Add forced neighbors
    auto forced = get_forced_neighbors(current, parent_direction);
    neighbors.insert(neighbors.end(), forced.begin(), forced.end());
    
    return neighbors;
}

std::vector<Direction> JPSGlobalNavigator::get_forced_neighbors(const GridCell& current, const Direction& direction) {
    std::vector<Direction> forced;
    
    // This is a simplified version - full JPS forced neighbor detection is quite complex for 3D
    // For now, we'll use a basic approach
    
    for (const auto& dir : directions_) {
        // Skip the direction we came from
        if (dir.dx == -direction.dx && dir.dy == -direction.dy && dir.dz == -direction.dz) {
            continue;
        }
        
        GridCell neighbor(current.x + dir.dx, current.y + dir.dy, current.z + dir.dz);
        if (is_valid_cell(neighbor) && !is_obstacle(neighbor)) {
            // Check if there's an obstacle that would force this direction
            GridCell blocking(current.x - dir.dx, current.y - dir.dy, current.z - dir.dz);
            if (!is_valid_cell(blocking) || is_obstacle(blocking)) {
                forced.push_back(dir);
            }
        }
    }
    
    return forced;
}

bool JPSGlobalNavigator::has_forced_neighbor(const GridCell& current, const Direction& direction) {
    auto forced = get_forced_neighbors(current, direction);
    return !forced.empty();
}

std::vector<geometry_msgs::msg::PoseStamped> JPSGlobalNavigator::smooth_path(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) {
    
    if (raw_path.size() <= 2) {
        return raw_path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> smoothed = raw_path;
    
    // Apply multiple iterations of smoothing
    for (int iter = 0; iter < smoothing_iterations_; ++iter) {
        std::vector<geometry_msgs::msg::PoseStamped> temp_path;
        temp_path.push_back(smoothed.front()); // Keep start
        
        for (size_t i = 1; i < smoothed.size() - 1; ++i) {
            // Try to create shortcuts by skipping intermediate waypoints
            bool shortcut_found = false;
            
            for (size_t j = i + 2; j < smoothed.size(); ++j) {
                if (is_line_collision_free(smoothed[i-1].pose.position, smoothed[j].pose.position)) {
                    // Skip waypoints from i to j-1
                    i = j - 1;
                    shortcut_found = true;
                    break;
                }
            }
            
            if (!shortcut_found) {
                temp_path.push_back(smoothed[i]);
            }
        }
        
        temp_path.push_back(smoothed.back()); // Keep end
        smoothed = temp_path;
        
        // If no improvement, break early
        if (smoothed.size() == raw_path.size()) {
            break;
        }
    }
    
    return smoothed;
}

bool JPSGlobalNavigator::is_line_collision_free(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to) {
    double dist = distance(from, to);
    int num_samples = static_cast<int>(dist / (cost_map_data_.resolution * 0.5)) + 1;
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        geometry_msgs::msg::Point sample;
        sample.x = from.x + t * (to.x - from.x);
        sample.y = from.y + t * (to.y - from.y);
        sample.z = from.z + t * (to.z - from.z);
        
        GridCell cell = point_to_grid(sample);
        if (!is_valid_cell(cell) || is_obstacle(cell)) {
            return false;
        }
    }
    
    return true;
}

// Grid and utility functions
GridCell JPSGlobalNavigator::point_to_grid(const geometry_msgs::msg::Point& point) {
    int x = static_cast<int>((point.x - cost_map_data_.min_bounds.x) / cost_map_data_.resolution);
    int y = static_cast<int>((point.y - cost_map_data_.min_bounds.y) / cost_map_data_.resolution);
    int z = static_cast<int>((point.z - cost_map_data_.min_bounds.z) / cost_map_data_.resolution);
    return GridCell(x, y, z);
}

geometry_msgs::msg::Point JPSGlobalNavigator::grid_to_point(const GridCell& cell) {
    geometry_msgs::msg::Point point;
    point.x = cost_map_data_.min_bounds.x + cell.x * cost_map_data_.resolution;
    point.y = cost_map_data_.min_bounds.y + cell.y * cost_map_data_.resolution;
    point.z = cost_map_data_.min_bounds.z + cell.z * cost_map_data_.resolution;
    return point;
}

bool JPSGlobalNavigator::is_valid_cell(const GridCell& cell) {
    return cell.x >= 0 && cell.x < cost_map_data_.grid_size_x &&
           cell.y >= 0 && cell.y < cost_map_data_.grid_size_y &&
           cell.z >= 0 && cell.z < cost_map_data_.grid_size_z;
}

bool JPSGlobalNavigator::is_obstacle(const GridCell& cell) {
    return get_cost_at_cell(cell) > cost_threshold_;
}

double JPSGlobalNavigator::get_cost_at_cell(const GridCell& cell) {
    auto it = cost_map_data_.grid_costs.find(cell);
    if (it != cost_map_data_.grid_costs.end()) {
        return it->second;
    }
    return 0.0; // Free space if not found
}

double JPSGlobalNavigator::heuristic(const GridCell& a, const GridCell& b) {
    // 3D Euclidean distance
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz) * cost_map_data_.resolution;
}

double JPSGlobalNavigator::distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void JPSGlobalNavigator::initialize_directions() {
    // 26 directions for 3D movement (excluding center)
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                directions_.emplace_back(dx, dy, dz);
            }
        }
    }
}

void JPSGlobalNavigator::build_grid_from_cost_map() {
    // Calculate grid dimensions
    cost_map_data_.grid_size_x = static_cast<int>((cost_map_data_.max_bounds.x - cost_map_data_.min_bounds.x) / cost_map_data_.resolution) + 1;
    cost_map_data_.grid_size_y = static_cast<int>((cost_map_data_.max_bounds.y - cost_map_data_.min_bounds.y) / cost_map_data_.resolution) + 1;
    cost_map_data_.grid_size_z = static_cast<int>((cost_map_data_.max_bounds.z - cost_map_data_.min_bounds.z) / cost_map_data_.resolution) + 1;
    
    // Build grid from point cloud
    for (size_t i = 0; i < cost_map_data_.points.size(); ++i) {
        GridCell cell = point_to_grid(cost_map_data_.points[i]);
        cost_map_data_.grid_costs[cell] = cost_map_data_.costs[i];
    }
}

std::vector<geometry_msgs::msg::PoseStamped> JPSGlobalNavigator::extract_path(std::shared_ptr<JPSNode> goal_node) {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    
    auto current = goal_node;
    while (current) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position = grid_to_point(current->cell);
        pose.pose.orientation.w = 1.0;
        
        path.insert(path.begin(), pose);
        current = current->parent;
    }
    
    return path;
}

nav_msgs::msg::Path JPSGlobalNavigator::create_path_message(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    path_msg.poses = poses;
    return path_msg;
}

size_t JPSGlobalNavigator::get_current_goal_index(const geometry_msgs::msg::Point& current_pos) {
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = current_goal_index_;
    
    // Check from current goal index onwards
    for (size_t i = current_goal_index_; i < current_goal_poses_.size(); ++i) {
        double dist = distance(current_pos, current_goal_poses_[i].pose.position);
        if (dist < jps_goal_tolerance_) {
            return i + 1; // Move to next goal
        }
        if (dist < min_distance) {
            min_distance = dist;
            closest_index = i;
        }
    }
    
    return closest_index;
}

double JPSGlobalNavigator::calculate_distance_remaining(const geometry_msgs::msg::Point& current_pos) {
    if (current_goal_index_ >= current_goal_poses_.size()) {
        return 0.0;
    }
    
    double total_distance = 0.0;
    
    // Distance to current goal
    total_distance += distance(current_pos, current_goal_poses_[current_goal_index_].pose.position);
    
    // Distance between remaining goals
    for (size_t i = current_goal_index_ + 1; i < current_goal_poses_.size(); ++i) {
        total_distance += distance(
            current_goal_poses_[i-1].pose.position,
            current_goal_poses_[i].pose.position
        );
    }
    
    return total_distance;
}

void JPSGlobalNavigator::publish_jps_search_markers(const std::vector<std::shared_ptr<JPSNode>>& explored_nodes,
                                                     const GridCell& start, const GridCell& goal,
                                                     const std::vector<GridCell>& path) {
    if (!enable_debug_visualization_) return;
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "jps_search";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Explored nodes
    visualization_msgs::msg::Marker explored_marker;
    explored_marker.header.frame_id = "map";
    explored_marker.header.stamp = this->now();
    explored_marker.ns = "jps_search";
    explored_marker.id = 0;
    explored_marker.type = visualization_msgs::msg::Marker::POINTS;
    explored_marker.action = visualization_msgs::msg::Marker::ADD;
    explored_marker.scale.x = cost_map_data_.resolution * 0.5;
    explored_marker.scale.y = cost_map_data_.resolution * 0.5;
    explored_marker.color.r = 0.0;
    explored_marker.color.g = 0.0;
    explored_marker.color.b = 1.0;
    explored_marker.color.a = 0.5;
    
    for (const auto& node : explored_nodes) {
        geometry_msgs::msg::Point point = grid_to_point(node->cell);
        explored_marker.points.push_back(point);
    }
    marker_array.markers.push_back(explored_marker);
    
    // Start and goal markers
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = this->now();
    start_marker.ns = "jps_search";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position = grid_to_point(start);
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = cost_map_data_.resolution;
    start_marker.scale.y = cost_map_data_.resolution;
    start_marker.scale.z = cost_map_data_.resolution;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;
    marker_array.markers.push_back(start_marker);
    
    visualization_msgs::msg::Marker goal_marker = start_marker;
    goal_marker.id = 2;
    goal_marker.pose.position = grid_to_point(goal);
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    marker_array.markers.push_back(goal_marker);
    
    // Path markers
    if (!path.empty()) {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = "jps_search";
        path_marker.id = 3;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = cost_map_data_.resolution * 0.2;
        path_marker.color.r = 1.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;
        
        for (const auto& cell : path) {
            geometry_msgs::msg::Point point = grid_to_point(cell);
            path_marker.points.push_back(point);
        }
        marker_array.markers.push_back(path_marker);
    }
    
    jps_search_marker_publisher_->publish(marker_array);
}

void JPSGlobalNavigator::clear_jps_search_markers() {
    if (!enable_debug_visualization_) return;
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "jps_search";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    jps_search_marker_publisher_->publish(marker_array);
}