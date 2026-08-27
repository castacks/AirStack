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
  progress_s = 0.;
  progress_reseed_pending_ = false;
  progress_back_m = airstack::get_param(node, "progress_back_m", 5.0);
  yaw_smoothing_alpha = airstack::get_param(node, "yaw_smoothing_alpha", 0.4);
  progress_window_m = airstack::get_param(node, "progress_window_m", 25.0);
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
  // A replanning planner may republish a geometrically identical path at
  // a fixed rate; treating each copy as a new plan would reset the
  // monotonic progress tracker every cycle. Only accept genuinely new
  // geometry.
  if (!path.poses.empty() && msg->poses.size() == path.poses.size())
  {
    bool same = true;
    for (size_t i = 0; i < msg->poses.size(); i++)
    {
      const auto &a = msg->poses[i].pose.position;
      const auto &b = path.poses[i].pose.position;
      if (std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z) > 0.01)
      {
        same = false;
        break;
      }
    }
    if (same)
      return;
  }
  path = *msg;
  next_global_plan_id = current_global_plan_id + 1;
  // A replanning planner that re-anchors its path start to the vehicle's
  // current pose publishes "new" geometry every cycle; resetting progress
  // to zero would snap the deviation/progress window back to the route
  // start each time (observed as a permanent mid-route stall). Instead,
  // carry progress over: re-seed it on the new plan near the old value.
  progress_reseed_pending_ = true;
}

/**
 * @brief Closest point on the plan within the monotonic progress window
 * @param p Query point in the plan's frame
 * @param deviation Output distance from p to the windowed closest point
 * @param s_at Output arc length (from the plan start) of that point
 * @return False if the plan has fewer than two waypoints
 *
 * Only plan segments whose arc-length interval overlaps
 * [progress_s - progress_back_m, progress_s + progress_window_m] are
 * considered, so progress along self-approaching routes stays monotonic.
 */
bool GlobalPlan::windowed_closest(const tf2::Vector3 &p, double *deviation, double *s_at, tf2::Vector3 *closest)
{
  const std::vector<Waypoint> &wps = global_plan.get_waypoints();
  if (wps.size() < 2)
    return false;

  double lo = progress_s - progress_back_m;
  double hi = progress_s + progress_window_m;
  double best_d = std::numeric_limits<double>::max();
  double best_s = progress_s;
  double s = 0.;
  bool found = false;
  for (size_t i = 1; i < wps.size(); i++)
  {
    tf2::Vector3 a = wps[i - 1].position();
    tf2::Vector3 b = wps[i].position();
    double len = a.distance(b);
    double seg_start = s;
    s += len;
    if (len <= 0.)
      continue;
    if (s < lo || seg_start > hi)
      continue;
    double t = ((p - a).dot(b - a)) / ((b - a).dot(b - a));
    t = std::max(0., std::min(1., t));
    double d = (a + t * (b - a)).distance(p);
    if (d < best_d)
    {
      best_d = d;
      best_s = seg_start + t * len;
      if (closest)
        *closest = a + t * (b - a);
      found = true;
    }
  }
  if (!found)
    return false;
  *deviation = best_d;
  *s_at = best_s;
  return true;
}

/**
 * @brief Advance the monotonic progress tracker from the look-ahead point
 * @param msg Odometry of look-ahead point
 *
 * The plan itself is no longer destructively trimmed; all queries are
 * windowed around progress_s instead (see windowed_closest).
 */
void GlobalPlan::trim(const airstack_msgs::msg::Odometry &msg)
{
  if (!update_global_plan())
    return;

  tf2::Vector3 look_ahead_position = tflib::to_tf(msg.pose.position);
  bool success = tflib::to_frame(tf_buffer, look_ahead_position,
                                 msg.header.frame_id, global_plan.get_frame_id(),
                                 msg.header.stamp, &look_ahead_position);
  if (!success)
    return;
  double dev, s_at;
  if (progress_reseed_pending_)
  {
    // Carry progress onto the (possibly re-anchored) new plan: window
    // around the old value, assigned non-monotonically because arc
    // lengths shift when the plan's prepended first segment moves.
    if (windowed_closest(look_ahead_position, &dev, &s_at, nullptr))
      progress_s = s_at;
    else
    {
      auto [valid, wp, index, pd] =
          global_plan.get_closest_point(look_ahead_position);
      (void)wp;
      (void)index;
      if (valid)
        progress_s = pd;
    }
    progress_reseed_pending_ = false;
    return;
  }
  // Advance the monotonic progress tracker (the plan itself is no longer
  // destructively trimmed; queries are windowed around progress_s).
  if (windowed_closest(look_ahead_position, &dev, &s_at, nullptr))
    progress_s = std::max(progress_s, s_at);
}

/**
 * @brief Re-seed the monotonic progress tracker from a GLOBAL closest-point
 *        search (monotonicity escape hatch)
 * @param msg Odometry of the look-ahead point
 *
 * Recovery only: if the tracker ever runs ahead of the vehicle (so the
 * window slides out of reach and every candidate is rejected), planning
 * deadlocks. Called by the node after a sustained fully-blocked pause.
 */
void GlobalPlan::reseed_progress(const airstack_msgs::msg::Odometry &msg)
{
  if (!update_global_plan())
    return;
  tf2::Vector3 p = tflib::to_tf(msg.pose.position);
  if (!tflib::to_frame(tf_buffer, p, msg.header.frame_id,
                       global_plan.get_frame_id(), msg.header.stamp, &p))
    return;
  auto [valid, wp, index, path_distance] = global_plan.get_closest_point(p);
  if (valid)
    progress_s = path_distance;
}

/**
 * @brief Deviation components and progress for a given point
 * @param x X coordinate of point in target frame
 * @param y Y coordinate of point in target frame
 * @param z Z coordinate of point in target frame
 * @return Tuple of (lateral deviation, signed vertical offset above the
 *         plan, arc length along the plan) in meters, using the windowed
 *         closest point (see windowed_closest); (-1, -1, -1) if no valid
 *         global plan exists.
 */
std::tuple<float, float, float> GlobalPlan::get_distance(float x, float y, float z)
{
  if (!update_global_plan())
    return std::make_tuple(-1.f, -1.f, -1.f);

  tf2::Vector3 p(x, y, z);
  double deviation, s_at;
  tf2::Vector3 closest;
  if (!windowed_closest(p, &deviation, &s_at, &closest))
    return std::make_tuple(-1.f, -1.f, -1.f);

  float dev_xy = std::hypot(closest.x() - p.x(), closest.y() - p.y());
  // Signed: positive = the query point is ABOVE the plan.
  float dz_signed = (float)(p.z() - closest.z());
  return std::make_tuple(dev_xy, dz_signed, (float)s_at);
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
 * 2. Exponential smoothing (yaw_smoothing_alpha) between consecutive waypoints
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
    double alpha = yaw_smoothing_alpha;
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
