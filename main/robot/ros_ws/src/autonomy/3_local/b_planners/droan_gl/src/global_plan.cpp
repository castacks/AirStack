#include <droan_gl/global_plan.hpp>

/**
 * @brief Constructor for GlobalPlan
 * @param node Pointer to ROS node for parameter access
 * @param tf_buffer Pointer to TF buffer for coordinate transformations
 * 
 * Initializes global plan manager with ROS parameters.
 */
GlobalPlan::GlobalPlan(rclcpp::Node *node, tf2_ros::Buffer *tf_buffer)
    : node(node), tf_buffer(tf_buffer)
{
  current_global_plan_id = -1;
  next_global_plan_id = -1;
  target_frame = airstack::get_param(node, "target_frame", std::string("map"));
}

/**
 * @brief Update internal global plan representation if changed
 * @return True if global plan is available, false otherwise
 * 
 * Converts ROS path message to internal trajectory representation
 * and transforms to target frame when a new plan is received.
 */
bool GlobalPlan::update_global_plan()
{
  if (next_global_plan_id == -1)
    return false;

  if (next_global_plan_id != current_global_plan_id)
  {
    global_plan = Trajectory(node, path);
    global_plan = global_plan.to_frame(target_frame, path.header.stamp);
    current_global_plan_id = next_global_plan_id;
  }

  return true;
}

/**
 * @brief Set a new global plan from ROS message
 * @param msg Path message containing the global plan
 * 
 * Stores the path and triggers update on next access.
 */
void GlobalPlan::set_global_plan(const nav_msgs::msg::Path::SharedPtr msg)
{
  path = *msg;
  next_global_plan_id = current_global_plan_id + 1;
}

/**
 * @brief Trim global plan based on current progress
 * @param msg Odometry of look-ahead point
 * 
 * Removes portions of the global plan that have already been traversed
 * based on the look-ahead position, maintaining only the remaining path ahead.
 */
void GlobalPlan::trim(const airstack_msgs::msg::Odometry &msg)
{
  if (!update_global_plan())
    return;

  tf2::Vector3 look_ahead_position = tflib::to_tf(msg.pose.position);
  bool success = tflib::to_frame(tf_buffer, look_ahead_position,
                                 msg.header.frame_id, global_plan.get_frame_id(),
                                 msg.header.stamp, &look_ahead_position);
  global_plan.trim(look_ahead_position);
}

/**
 * @brief Calculate deviation and path distance for a given point
 * @param x X coordinate of point in target frame
 * @param y Y coordinate of point in target frame
 * @param z Z coordinate of point in target frame
 * @return Tuple of (deviation from path, distance along path) in meters
 * 
 * Computes:
 * - deviation: Euclidean distance from point to closest point on global path
 * - path_distance: Arc length along global path to that closest point
 * 
 * Returns (-1, -1) if no valid global plan exists.
 */
std::tuple<float, float> GlobalPlan::get_distance(float x, float y, float z)
{
  if (!update_global_plan())
    return std::make_tuple(-1.f, -1.f);

  tf2::Vector3 p(x, y, z);
  auto [valid, wp, index, path_distance] = global_plan.get_closest_point(p);

  if (!valid)
    return std::make_tuple(-1.f, -1.f);

  return std::make_tuple(wp.position().distance(p), path_distance);
}

/**
 * @brief Publish visualization of global plan
 * @param pub Publisher for marker array messages
 * 
 * Generates and publishes visualization markers showing the
 * current global plan path.
 */
void GlobalPlan::publish_vis(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  if (!update_global_plan())
    return;

  pub->publish(global_plan.get_markers(node->now(), "global_plan", 0, 0, 1));
}

/**
 * @brief Apply smooth yaw angles to trajectory waypoints
 * @param best_traj_msg Trajectory message to modify (modified in-place)
 * @param look_ahead Odometry of look-ahead point for initial heading
 * 
 * Computes smooth yaw angles for trajectory waypoints using:
 * 1. Initial heading from look-ahead orientation
 * 2. Exponential smoothing (alpha=0.1) between consecutive waypoints
 * 3. Yaw calculated from velocity direction between waypoints
 * 
 * Ensures smooth heading changes along the trajectory for better tracking.
 */
void GlobalPlan::apply_smooth_yaw(airstack_msgs::msg::TrajectoryXYZVYaw &best_traj_msg, const airstack_msgs::msg::Odometry look_ahead)
{
  bool found_initial_heading = false;
  double initial_heading = 0;
  try
  {
    tf2::Stamped<tf2::Transform> transform;
    tf_buffer->canTransform(best_traj_msg.header.frame_id, look_ahead.header.frame_id,
                            look_ahead.header.stamp, rclcpp::Duration::from_seconds(0.1));
    auto transform_msg = tf_buffer->lookupTransform(best_traj_msg.header.frame_id,
                                                    look_ahead.header.frame_id,
                                                    look_ahead.header.stamp);
    tf2::fromMsg(transform_msg, transform);

    transform.setOrigin(tf2::Vector3(0, 0, 0)); // only care about rotation
    initial_heading =
        tf2::getYaw(transform * tflib::to_tf(look_ahead.pose.orientation));

    found_initial_heading = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get transform: %s", ex.what());
  }

  if (found_initial_heading)
  {
    best_traj_msg.waypoints[0].yaw = initial_heading;
    double alpha = 0.1;
    double sin_yaw_prev = sin(best_traj_msg.waypoints[0].yaw);
    double cos_yaw_prev = cos(best_traj_msg.waypoints[0].yaw);

    for (size_t i = 1; i < best_traj_msg.waypoints.size(); i++)
    {
      airstack_msgs::msg::WaypointXYZVYaw wp_prev = best_traj_msg.waypoints[i - 1];
      airstack_msgs::msg::WaypointXYZVYaw &wp_curr = best_traj_msg.waypoints[i];

      double yaw = atan2(wp_curr.position.y - wp_prev.position.y,
                         wp_curr.position.x - wp_prev.position.x);
      double cos_yaw = alpha * cos(yaw) + (1 - alpha) * cos_yaw_prev;
      double sin_yaw = alpha * sin(yaw) + (1 - alpha) * sin_yaw_prev;
      yaw = atan2(sin_yaw, cos_yaw);

      sin_yaw_prev = sin_yaw;
      cos_yaw_prev = cos_yaw;

      wp_curr.yaw = yaw;
    }
  }
}
