#include <droan_gl/rewind_monitor.hpp>

/**
 * @brief Constructor for RewindMonitor
 * @param node Pointer to ROS node for parameter access and logging
 * 
 * Initializes rewind monitor with ROS parameters for stuck detection:
 * - all_in_collision_* parameters for deadlock recovery
 * - stationary_* parameters for stuck-in-place detection
 * - Text marker configuration for rewind status visualization
 */
RewindMonitor::RewindMonitor(rclcpp::Node *node)
    : node(node)
{

  all_in_collision_duration_threshold = airstack::get_param(node, "all_in_collision_duration_threshold", 2.);
  all_in_collision_rewind_duration = airstack::get_param(node, "all_in_collision_rewind_duration", 6.);
  stationary_distance_threshold = airstack::get_param(node, "stationary_distance_threshold", 0.5);
  stationary_history_duration = airstack::get_param(node, "stationary_history_duration", 10.);
  stationary_rewind_distance = airstack::get_param(node, "stationary_rewind_distance", 10.);
  stationary_rewind_duration = airstack::get_param(node, "stationary_rewind_duration", 20.);

  min_dt = 0.2;
  all_in_collision_start_time = -1.f;
  should_do_stationary_check = true;

  rewind_info_marker.ns = "rewind_info";
  rewind_info_marker.id = 0;
  rewind_info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  rewind_info_marker.action = visualization_msgs::msg::Marker::ADD;
  rewind_info_marker.pose.position.z = 3.0;
  rewind_info_marker.pose.orientation.w = 1.0;
  rewind_info_marker.scale.z = 0.3;
  rewind_info_marker.color.r = 1.0;
  rewind_info_marker.color.g = 1.0;
  rewind_info_marker.color.b = 1.0;
  rewind_info_marker.color.a = 1.0;

  clear.ns = rewind_info_marker.ns;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
}

/**
 * @brief Update position history with new odometry
 * @param msg Odometry message with robot position
 * 
 * Maintains a sliding window of robot positions over time for
 * stationary detection. Removes old data outside the history duration
 * and adds new positions at the configured minimum time step.
 */
void RewindMonitor::update_odom(const airstack_msgs::msg::Odometry::SharedPtr msg)
{
  float time = rclcpp::Time(msg->header.stamp).seconds();
  if (positions.empty() || (time - positions.back().second) >= min_dt)
  {
    // remove old data
    bool updated = false;
    std::pair<tf2::Vector3, float> front;
    while (!positions.empty() && (time - positions.front().second) > stationary_history_duration)
    {
      front = positions.front();
      positions.pop_front();
      updated = true;
    }
    if (updated)
      positions.push_front(front);

    tf2::Vector3 pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    positions.push_back(std::pair<tf2::Vector3, float>(pos, time));
  }
}

/**
 * @brief Enable or disable stationary checking
 * @param b True to enable stationary checks, false to disable
 * 
 * Allows external control of whether stationary detection should be active.
 * Useful for disabling during initialization or specific flight modes.
 */
void RewindMonitor::do_stationary_check(bool b)
{
  should_do_stationary_check = b;
}

/**
 * @brief Update all-in-collision timer based on trajectory availability
 * @param b True if collision-free trajectory found, false if all in collision
 * 
 * Starts timing when all trajectories are in collision, resets when
 * a valid trajectory is found. Used to trigger rewind after prolonged deadlock.
 */
void RewindMonitor::found_trajectory(bool b)
{
  if (b)
    all_in_collision_start_time = -1.f;
  else if (all_in_collision_start_time < 0.f)
    all_in_collision_start_time = node->get_clock()->now().seconds();
}

/**
 * @brief Manually trigger a rewind operation
 * @param duration Duration to rewind in seconds (required)
 * @param distance Distance to rewind in meters (optional, default -1 = not used)
 * 
 * Initiates a rewind with specified duration and/or distance constraints.
 * Rewind ends when either condition is met (whichever comes first).
 */
void RewindMonitor::trigger_rewind(float duration, float distance)
{
  rewind = Rewind();
  rewind.valid = true;
  rewind.start_time = node->get_clock()->now().seconds();
  if (!positions.empty())
    rewind.start_position = positions.back().first;
  rewind.duration = duration;
  rewind.distance = distance;
}

/**
 * @brief Check if robot should rewind and update rewind state
 * @return True if rewind is active, false otherwise
 * 
 * Monitors two stuck conditions:
 * 1. All-in-collision: All trajectories blocked for duration threshold
 * 2. Stationary: Robot hasn't moved beyond distance threshold for history duration
 * 
 * Automatically triggers appropriate rewind when conditions are met.
 * Updates rewind state based on duration/distance progress.
 */
bool RewindMonitor::should_rewind()
{
  float now = node->get_clock()->now().seconds();

  // RCLCPP_INFO_STREAM(node->get_logger(), "REWIND INFO: "
  //		     << (now - all_in_collision_start_time) << " / " << all_in_collision_duration_threshold
  //		     << " " << all_in_collision_start_time << " " << now);
  if (rewind.valid)
  {
    bool valid_before = rewind.valid;
    bool duration_valid = rewind.duration > 0.f;
    bool distance_valid = rewind.distance > 0.f;
    bool duration_ongoing = now - rewind.start_time < rewind.duration;
    if (distance_valid)
    {
      bool distance_ongoing = rewind.start_position.distance(positions.back().first) < rewind.distance;
      rewind.valid = distance_ongoing && (duration_valid && duration_ongoing);
    }
    else if (duration_valid)
      rewind.valid = duration_ongoing;
    else
      rewind.valid = false;

    if (valid_before && !rewind.valid)
    {
      if (rewind.distance > 0.f)
        positions.clear();
      else
        all_in_collision_start_time = -1.f;
    }
  }
  else
  {
    if (all_in_collision_start_time > 0.f && (now - all_in_collision_start_time) > all_in_collision_duration_threshold)
      trigger_rewind(all_in_collision_rewind_duration);
    else if (should_do_stationary_check &&
             !positions.empty() &&
             (positions.back().second - positions.front().second) >= stationary_history_duration)
    {
      bool stationary = true;
      const tf2::Vector3 &front = positions.front().first;
      const tf2::Vector3 &back = positions.back().first;
      for (int i = 1; i < positions.size() - 1; i++)
      {
        const tf2::Vector3 &pos = positions[i].first;
        if (pos.distance(front) >= stationary_distance_threshold || pos.distance(back) >= stationary_distance_threshold)
        {
          stationary = false;
          break;
        }
      }
      if (stationary)
        trigger_rewind(stationary_rewind_duration, stationary_rewind_distance);
    }
  }

  return rewind.valid;
}

/**
 * @brief Clear all position history and reset rewind state
 * 
 * Resets the rewind monitor to initial state, clearing:
 * - Position history for stationary detection
 * - Active rewind status
 * - All-in-collision timer
 */
void RewindMonitor::clear_history()
{
  positions.clear();
  rewind.valid = false;
  all_in_collision_start_time = -1.f;
}

/**
 * @brief Publish rewind status visualization
 * @param pub Publisher for marker array messages
 * @param frame_id Frame ID for visualization markers
 * 
 * Publishes text marker showing:
 * - Rewind progress (time and/or distance remaining)
 * - All-in-collision timer status when not rewinding
 * 
 * Marker appears as floating text near the robot.
 */
void RewindMonitor::publish_vis(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, std::string frame_id)
{
  rclcpp::Time now = node->get_clock()->now();
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(clear);
  rewind_info_marker.header.frame_id = frame_id;
  rewind_info_marker.header.stamp = now;
  rewind_info_marker.text = "";
  if (rewind.valid)
  {
    if (rewind.duration > 0.f)
      rewind_info_marker.text += "time: " + std::to_string(now.seconds() - rewind.start_time) +
                                 " / " + std::to_string(rewind.duration);
    if (rewind.distance > 0.f && !positions.empty())
      rewind_info_marker.text += "\ndistance: " + std::to_string(rewind.start_position.distance(positions.back().first)) +
                                 " / " + std::to_string(rewind.distance);
  }
  else
  {
    rewind_info_marker.text += std::to_string(all_in_collision_start_time) + "\n" + std::to_string(now.seconds() - all_in_collision_start_time);
  }

  // RCLCPP_INFO_STREAM(node->get_logger(), "vis: " << rewind.valid << " time: " << (rewind.duration > 0.f) << " dist: " << (rewind.distance > 0.f)
  //		     << " " << (!positions.empty()) << " text: " << rewind_info_marker.text);
  marker_array.markers.push_back(rewind_info_marker);

  pub->publish(marker_array);
}
