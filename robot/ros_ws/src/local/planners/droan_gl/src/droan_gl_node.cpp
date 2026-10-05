#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <task_msgs/action/navigate_task.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <airstack_common/vislib.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <airstack_common/tflib.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <EGL/egl.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <droan_gl/gl_interface.hpp>
#include <droan_gl/checked_prefix.hpp>
#include <droan_gl/global_plan.hpp>
#include <droan_gl/rewind_monitor.hpp>

class DisparityExpanderNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_stuck_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_map_sub;
  tf2_ros::Buffer *tf_buffer;
  tf2_ros::TransformListener *tf_listener;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fg_pub_, bg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_plan_vis_pub;
  rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rewind_info_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selection_diagnostic_pub_;

  rclcpp::TimerBase::SharedPtr timer;

  // Action server (NavigateTask)
  using NavigateTask = task_msgs::action::NavigateTask;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateTask>;
  rclcpp_action::Server<NavigateTask>::SharedPtr navigate_action_server_;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr trajectory_mode_client_;
  std::atomic<bool> task_active_{false};
  std::atomic<bool> cancel_requested_{false};
  airstack_msgs::msg::Odometry tracking_point_odom_;
  bool tracking_point_valid_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odometry_sub_;
  std::mutex robot_odometry_mutex_;
  nav_msgs::msg::Odometry robot_odometry_;
  bool robot_odometry_valid_ = false;
  std::optional<std::pair<double, double>> navigation_altitude_bounds_;
  SingletonGoalStop singleton_goal_stop_;
  std::mutex global_plan_mutex_;

  std::string target_frame, look_ahead_frame, rewind_info_frame;
  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::chrono::steady_clock::time_point look_ahead_received_;
  std::vector<TrajectoryPoint> trajectory_points;
  vis::MarkerArray traj_markers;
  bool visualize;

  GLInterface *gl_interface;
  GlobalPlan *global_plan;
  RewindMonitor *rewind_monitor;

public:
  DisparityExpanderNode()
      : Node("disparity_expander_node")
  {
    disp_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>("disparity", 10,
                                                                      std::bind(&DisparityExpanderNode::onDisparity,
                                                                                this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10,
                                                                     std::bind(&DisparityExpanderNode::onCameraInfo,
                                                                               this, std::placeholders::_1));
    look_ahead_sub = create_subscription<airstack_msgs::msg::Odometry>("look_ahead", 10,
                                                                       std::bind(&DisparityExpanderNode::look_ahead_callback,
                                                                                 this, std::placeholders::_1));
    tracking_point_sub = create_subscription<airstack_msgs::msg::Odometry>("tracking_point", 10,
                                                                           std::bind(&DisparityExpanderNode::tracking_point_callback,
                                                                                     this, std::placeholders::_1));
    robot_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "robot_odometry", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(robot_odometry_mutex_);
          robot_odometry_ = *msg;
          robot_odometry_valid_ = true;
        });
    global_plan_sub = create_subscription<nav_msgs::msg::Path>("global_plan", 1,
                                                               std::bind(&DisparityExpanderNode::global_plan_callback,
                                                                         this, std::placeholders::_1));
    reset_stuck_sub = this->create_subscription<std_msgs::msg::Empty>("reset_stuck", 1,
                                                                      std::bind(&DisparityExpanderNode::reset_stuck_callback,
                                                                                this, std::placeholders::_1));
    clear_map_sub = this->create_subscription<std_msgs::msg::Empty>("clear_map", 1,
                                                                    std::bind(&DisparityExpanderNode::clear_map_callback,
                                                                              this, std::placeholders::_1));

    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    fg_pub_ = create_publisher<sensor_msgs::msg::Image>("foreground_expanded", 1);
    bg_pub_ = create_publisher<sensor_msgs::msg::Image>("background_expanded", 1);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 1);
    traj_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("traj_debug", 1);
    graph_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("graph_vis", 1);
    global_plan_vis_pub = create_publisher<visualization_msgs::msg::MarkerArray>("local_planner_global_plan_vis", 1);
    traj_pub = create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_segment_to_add", 1);
    stuck_pub = create_publisher<std_msgs::msg::Bool>("stuck", 1);
    rewind_info_pub = create_publisher<visualization_msgs::msg::MarkerArray>("rewind_info", 1);
    selection_diagnostic_pub_ = create_publisher<std_msgs::msg::String>("selection_diagnostic", 1);

    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    look_ahead_frame = airstack::get_param(this, "look_ahead_frame", std::string("look_ahead_point_stabilized"));
    rewind_info_frame = airstack::get_param(this, "rewind_info_frame", std::string("base_link_stabilized"));
    visualize = airstack::get_param(this, "visualize", true);

    look_ahead_valid = false;

    gl_interface = new GLInterface(this, tf_buffer);
    global_plan = new GlobalPlan(this, tf_buffer);
    rewind_monitor = new RewindMonitor(this);

    // TODO make this time a parameter
    timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(2. * 1. / 5.),
                                 std::bind(&DisparityExpanderNode::timer_callback, this));

    trajectory_mode_client_ = create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode");

    navigate_action_server_ = rclcpp_action::create_server<NavigateTask>(
        this, "~/navigate_task",
        std::bind(&DisparityExpanderNode::handle_navigate_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DisparityExpanderNode::handle_navigate_cancel, this, std::placeholders::_1),
        std::bind(&DisparityExpanderNode::handle_navigate_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "DisparityExpanderNode initialized, waiting for NavigateTask goals");
  }

private:
  /**
   * @brief Callback for camera intrinsics messages
   * @param msg Camera info message containing intrinsic parameters
   * 
   * Forwards camera intrinsics to the GL interface for initialization of
   * GPU-based disparity expansion and collision checking.
   */
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    gl_interface->handle_camera_info(msg);
  }

  /**
   * @brief Callback for disparity image messages
   * @param msg Disparity image from stereo camera
   * 
   * Processes incoming disparity images through GPU-based expansion
   * and optionally publishes visualization of the expanded obstacles.
   */
  void onDisparity(const stereo_msgs::msg::DisparityImage::SharedPtr msg)
  {
    gl_interface->handle_disparity(msg);
    if (visualize)
      gl_interface->publish_viz(msg->header, fg_pub_, bg_pub_, fg_bg_cloud_pub_, graph_vis_pub_);
  }

  /**
   * @brief Main execution loop for trajectory planning
   * 
   * Runs at 2.5 Hz to:
   * 1. Evaluate trajectories using GPU-based collision checking
   * 2. Score collision-free trajectories based on global path alignment
   * 3. Select and publish the best trajectory
   * 4. Monitor for stuck conditions requiring rewind
   */
  void timer_callback()
  {
    const auto cycle_started = std::chrono::steady_clock::now();
    if (!look_ahead_valid)
      return;

    std_msgs::msg::Bool stuck_msg;
    stuck_msg.data = rewind_monitor->should_rewind();
    stuck_pub->publish(stuck_msg);
    rewind_monitor->publish_vis(rewind_info_pub, rewind_info_frame);

    if (!gl_interface->evaluate_trajectories(look_ahead, trajectory_points))
      return;
    if (trajectory_points.empty())
      return;

    std::lock_guard<std::mutex> plan_lock(global_plan_mutex_);
    global_plan->trim(look_ahead);

    traj_markers.overwrite();
    vis::Marker &free_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    free_markers.set_namespace("free_points");
    free_markers.set_color(0., 1., 0.);
    free_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker &free_traj_markers = traj_markers.add_line_list(target_frame, look_ahead.header.stamp,
                                                                0., 1., 0., 0.8,
                                                                0.1, 0);
    free_traj_markers.set_namespace("free_trajectories");

    vis::Marker &collision_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    collision_markers.set_namespace("collision_points");
    collision_markers.set_color(1., 0., 0.);
    collision_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker &collision_traj_markers = traj_markers.add_line_list(target_frame, look_ahead.header.stamp,
                                                                     1., 0., 0., 0.8,
                                                                     0.1, 0);
    collision_traj_markers.set_namespace("collision_trajectories");

    vis::Marker &unseen_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    unseen_markers.set_namespace("unseen_points");
    unseen_markers.set_color(0.7, 0.7, 0.7, 0.3);
    unseen_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker &unseen_traj_markers = traj_markers.add_line_list(target_frame, look_ahead.header.stamp,
                                                                  0.7, 0.7, 0.7, 0.3,
                                                                  0.1, 0);
    unseen_traj_markers.set_namespace("unseen_trajectories");

    int best_traj_index = -1;
    std::vector<TrajectoryPoint> best_points;
    std::optional<std::pair<double, double>> altitude_bounds;
    std::optional<tf2::Vector3> singleton_goal;
    bool allow_goal_height_stop = false;
    {
      std::lock_guard<std::mutex> lock(robot_odometry_mutex_);
      if (task_active_) {
        altitude_bounds = navigation_altitude_bounds_;
        singleton_goal = singleton_goal_stop_.goal();
        allow_goal_height_stop = singleton_goal_stop_.height_stop();
      }
    }
    if (task_active_ && !altitude_bounds) return;  // Goal initialization not complete.
    int safe_prefix_points = 0;
    int longest_prefix_points = 0;
    double longest_prefix_distance = 0.;
    float best_traj_cost = std::numeric_limits<float>::infinity();
    const bool record_selection = task_active_ && selection_diagnostic_pub_->get_subscription_count() > 0;
    std::ostringstream diagnostic;
    diagnostic << std::setprecision(9);
    if (record_selection) {
      diagnostic << "{\"schema_version\":\"droan-selection/v1\",\"source_stamp_ns\":"
                 << rclcpp::Time(look_ahead.header.stamp).nanoseconds()
                 << ",\"frame_id\":\"" << target_frame << "\",\"origin\":["
                 << look_ahead.pose.position.x << ',' << look_ahead.pose.position.y << ','
                 << look_ahead.pose.position.z << "],\"altitude_bounds\":["
                 << altitude_bounds->first << ',' << altitude_bounds->second
                 << "],\"candidates\":[";
    }
    bool is_traj_safe = true;
    int SEEN = 0;
    int UNSEEN = 1;
    int COLLISION = 2;
    int traj_status = SEEN;
    std::vector<tf2::Vector3> traj_points(gl_interface->get_traj_size());

    for (int i = 0; i < trajectory_points.size(); i++)
    {
      TrajectoryPoint &state = trajectory_points[i];
      int traj_index = i / gl_interface->get_traj_size();
      int point_index = i % gl_interface->get_traj_size();

      // int seen, unseen, collision;
      // get_counts(state.w(), &seen, &unseen, &collision);
      int seen = state.get_seen();
      int unseen = state.get_unseen();
      int collision = state.get_collision();

      const tf2::Vector3 target_point = state.position();
      traj_points[point_index] = target_point;

      if (collision > 0 && collision > seen)
      {
        is_traj_safe = false;
        collision_markers.add_point(target_point.x(), target_point.y(), target_point.z());
        traj_status = COLLISION;
      }
      else if (seen > 1)
        free_markers.add_point(target_point.x(), target_point.y(), target_point.z());
      else
      {
        is_traj_safe = false;
        unseen_markers.add_point(target_point.x(), target_point.y(), target_point.z());
        if (traj_status == SEEN)
          traj_status = UNSEEN;
      }
      if (is_traj_safe) ++safe_prefix_points;

      // if last waypoint in trajectory
      if (point_index == (gl_interface->get_traj_size() - 1))
      {
        if (safe_prefix_points > 1) {
          const double distance = traj_points[0].distance(traj_points[safe_prefix_points - 1]);
          if (distance > longest_prefix_distance) {
            longest_prefix_distance = distance;
            longest_prefix_points = safe_prefix_points;
          }
        }
        safe_prefix_points = 0;
        vis::Marker *tm = &free_traj_markers;
        if (traj_status == UNSEEN)
          tm = &unseen_traj_markers;
        else if (traj_status == COLLISION)
          tm = &collision_traj_markers;

        for (int j = 1; j < traj_points.size(); j++)
        {
          tf2::Vector3 &curr = traj_points[j];
          tf2::Vector3 &prev = traj_points[j - 1];
          tm->add_point(prev.x(), prev.y(), prev.z());
          tm->add_point(curr.x(), curr.y(), curr.z());
        }

        int traj_status_log = traj_status;

        traj_status = SEEN;
        std::vector<TrajectoryPoint> candidate;
        size_t before_goal_crop_count = 0;
        if (altitude_bounds) {
          candidate = checked_prefix(trajectory_points,
              traj_index * gl_interface->get_traj_size(), gl_interface->get_traj_size(),
              altitude_bounds->first, altitude_bounds->second, 0.5,
              singleton_goal ? &*singleton_goal : nullptr, &before_goal_crop_count,
              allow_goal_height_stop);
        } else if (is_traj_safe) {
          const auto start = trajectory_points.begin() + traj_index * gl_interface->get_traj_size();
          candidate.assign(start, start + gl_interface->get_traj_size());
        }
        is_traj_safe = true;
        if (record_selection && traj_index < 256) {
          if (traj_index) diagnostic << ',';
          const size_t offset = traj_index * gl_interface->get_traj_size();
          const size_t count = gl_interface->get_traj_size();
          size_t first = 0;
          const char* reason = nullptr;
          for (; first < count; ++first) {
            reason = checked_point_rejection(trajectory_points[offset + first],
                altitude_bounds->first, altitude_bounds->second);
            if (reason) break;
          }
          const auto& end = trajectory_points[offset + count - 1];
          const auto number = [&diagnostic](double value) {
            if (std::isfinite(value)) diagnostic << value; else diagnostic << "null";
          };
          diagnostic << "{\"index\":" << traj_index << ",\"full_endpoint\":[";
          number(end.v1.x); diagnostic << ','; number(end.v1.y); diagnostic << ','; number(end.v1.z);
          diagnostic << "],\"checked_count\":" << first << ",\"first_rejection\":\""
                     << (reason ? reason : "none") << "\",\"before_goal_crop_count\":"
                     << before_goal_crop_count << ",\"published_count\":" << candidate.size();
          if (first < count) {
            const auto& rejected = trajectory_points[offset + first];
            diagnostic << ",\"rejected_point\":[";
            number(rejected.v1.x); diagnostic << ','; number(rejected.v1.y); diagnostic << ','; number(rejected.v1.z);
            diagnostic << "],\"rejected_counts\":[";
            number(rejected.v2.x); diagnostic << ','; number(rejected.v2.y); diagnostic << ','; number(rejected.v2.z);
            diagnostic << ']';
          }
          if (candidate.size() >= 2) {
            const auto end_point = candidate.back().position();
            const auto [deviation, progress] = global_plan->get_distance(end_point.x(), end_point.y(), end_point.z());
            diagnostic << ",\"endpoint\":[" << end_point.x() << ',' << end_point.y() << ',' << end_point.z()
                       << "],\"deviation\":"; number(deviation);
            diagnostic << ",\"path_distance\":"; number(progress);
            diagnostic << ",\"cost\":"; number(deviation - progress);
          }
          diagnostic << '}';
        }
        if (candidate.size() < 2) continue;

        const auto endpoint = candidate.back().position();
        auto [deviation, path_distance] = global_plan->get_distance(
            endpoint.x(), endpoint.y(), endpoint.z());
        // RCLCPP_INFO_STREAM(get_logger(), i << " " << traj_status_log << " " << deviation << " " <<  path_distance);
        if (deviation >= 0 && path_distance >= 0)
        {
          // TODO add weights as ros parameters
          float cost = deviation - path_distance;
          if (cost < best_traj_cost)
          {
            best_traj_cost = cost;
            best_traj_index = traj_index;
            best_points = std::move(candidate);
          }
        }
      }
    }

    traj_debug_pub_->publish(traj_markers.get_marker_array());
    if (record_selection) {
      diagnostic << "],\"selected_index\":" << best_traj_index
                 << ",\"candidate_limit\":256,\"total_candidates\":"
                 << trajectory_points.size() / gl_interface->get_traj_size()
                 << ",\"cycle_elapsed_ms\":"
                 << std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - cycle_started).count() << '}';
      std_msgs::msg::String message;
      message.data = diagnostic.str();
      selection_diagnostic_pub_->publish(message);
    }
    global_plan->publish_vis(global_plan_vis_pub);

    if (best_traj_index < 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "No full safe trajectory: longest checked-safe prefix %.3f m (%d samples)",
          longest_prefix_distance, longest_prefix_points);
      rewind_monitor->found_trajectory(false);
      return;
    }
    rewind_monitor->found_trajectory(true);

    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    for (size_t i = 0; i < best_points.size(); i++)
    {
      airstack_msgs::msg::WaypointXYZVYaw wp;

      TrajectoryPoint &state = best_points[i];
      const tf2::Vector3 p = state.position();

      wp.position.x = p.x();
      wp.position.y = p.y();
      wp.position.z = p.z();
      wp.velocity = state.get_vel();

      traj.waypoints.push_back(wp);
    }

    traj.header.stamp = look_ahead.header.stamp;
    traj.header.frame_id = target_frame;
    global_plan->apply_smooth_yaw(traj, look_ahead);
    traj_pub->publish(traj);
  }

  /**
   * @brief Decode packed collision counts from a float value
   * @param w Packed float containing seen, unseen, and collision counts
   * @param seen Output parameter for number of seen graph nodes
   * @param unseen Output parameter for number of unseen graph nodes
   * @param collision Output parameter for number of collision graph nodes
   * 
   * Unpacks three integer counts from a single float using modulo arithmetic.
   * Format: seen * 1000000 + unseen * 1000 + collision
   * 
   * @note This function appears to be unused in favor of direct accessor methods
   */
  void get_counts(float w, int *seen, int *unseen, int *collision)
  {
    int i = w;
    // RCLCPP_INFO_STREAM(get_logger(), "i: " << i);
    *collision = i % 1000;
    i -= *collision;
    // RCLCPP_INFO_STREAM(get_logger(), "i: " << i << " collision: " << *collision);
    *unseen = (i % 1000000) / 1000;
    // RCLCPP_INFO_STREAM(get_logger(), "i: " << i << " unseen: " << *unseen);
    i -= *unseen * 1000;
    *seen = i / 1000000;
    // RCLCPP_INFO_STREAM(get_logger(), "i: " << i << " seen: " << *seen);
  }

  /**
   * @brief Callback for look-ahead position updates
   * @param msg Odometry message for the look-ahead planning point
   * 
   * Updates the look-ahead position used as the starting point for
   * trajectory generation.
   */
  void look_ahead_callback(const airstack_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(robot_odometry_mutex_);
    look_ahead = *msg;
    look_ahead_received_ = std::chrono::steady_clock::now();
    look_ahead_valid = true;
  }

  /**
   * @brief Callback for tracking point odometry updates
   * @param msg Odometry message for the actual robot tracking point
   * 
   * Updates the rewind monitor with the robot's current position
   * for stationary detection and rewind distance tracking.
   */
  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg)
  {
    rewind_monitor->update_odom(msg);
    tracking_point_odom_ = *msg;
    tracking_point_valid_ = true;
  }

  /**
   * @brief Callback for global plan updates
   * @param msg Path message containing the global plan
   * 
   * Updates the global plan used for scoring trajectories based on
   * path alignment and progress.
   */
  void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(global_plan_mutex_);
    std::lock_guard<std::mutex> state_lock(robot_odometry_mutex_);
    singleton_goal_stop_.route_replaced();
    global_plan->set_global_plan(msg);
  }

  // ---------------------------------------------------------------------------
  // NavigateTask action server
  // ---------------------------------------------------------------------------

  rclcpp_action::GoalResponse handle_navigate_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const NavigateTask::Goal> /*goal*/)
  {
    if (task_active_) {
      RCLCPP_WARN(get_logger(), "Rejecting NavigateTask goal: task already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    {
      std::lock_guard<std::mutex> lock(robot_odometry_mutex_);
      navigation_altitude_bounds_.reset();
    }
    task_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_navigate_cancel(
      std::shared_ptr<NavigateGoalHandle> /*goal_handle*/)
  {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_navigate_accepted(std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    std::thread{std::bind(&DisparityExpanderNode::execute_navigate, this,
                          std::placeholders::_1), goal_handle}.detach();
  }

  void execute_navigate(std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    cancel_requested_ = false;

    // Atomically bind the route and its MAP vertical corridor to this goal.
    try {
      if (goal->global_plan.poses.empty()) throw std::runtime_error("Empty navigation route");
      const auto route_to_map = tf_buffer->lookupTransform(
          target_frame, goal->global_plan.header.frame_id, goal->global_plan.header.stamp);
      tf2::Transform transform;
      tf2::fromMsg(route_to_map.transform, transform);
      double minimum = std::numeric_limits<double>::infinity();
      double maximum = -minimum;
      for (const auto& pose : goal->global_plan.poses) {
        const auto p = transform * tflib::to_tf(pose.pose.position);
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
          throw std::runtime_error("Nonfinite navigation route");
        minimum = std::min(minimum, p.z()); maximum = std::max(maximum, p.z());
      }
      std::lock_guard<std::mutex> plan_lock(global_plan_mutex_);
      std::lock_guard<std::mutex> state_lock(robot_odometry_mutex_);
      if (!robot_odometry_valid_ || robot_odometry_.header.frame_id != target_frame)
        throw std::runtime_error("MAP physical start unavailable");
      const double start_z = robot_odometry_.pose.pose.position.z;
      if (!std::isfinite(start_z)) throw std::runtime_error("Nonfinite physical start");
      const auto anchor_now = this->now();
      const auto anchor_stamp = rclcpp::Time(look_ahead.header.stamp);
      const double anchor_age = (anchor_now - anchor_stamp).seconds();
      const double anchor_receipt_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - look_ahead_received_).count();
      const char* anchor_rejection = commanded_anchor_rejection(
          look_ahead_valid, look_ahead.header.frame_id == target_frame,
          anchor_age, anchor_receipt_age,
          std::isfinite(look_ahead.pose.position.x) && std::isfinite(look_ahead.pose.position.y));
      std::ostringstream anchor_diagnostic;
      anchor_diagnostic << std::setprecision(12)
          << "reason=" << (anchor_rejection ? anchor_rejection : "accepted")
          << " source_age_s=" << anchor_age << " receipt_age_s=" << anchor_receipt_age
          << " now_ns=" << anchor_now.nanoseconds() << " stamp_ns=" << anchor_stamp.nanoseconds()
          << " valid=" << look_ahead_valid << " frame=" << std::quoted(look_ahead.header.frame_id);
      if (anchor_rejection)
        throw std::runtime_error("Fresh MAP commanded start unavailable: " + anchor_diagnostic.str());
      RCLCPP_INFO(get_logger(), "Navigation start anchor: %s", anchor_diagnostic.str().c_str());
      // Candidate trajectories originate at the controller's commanded anchor,
      // not physical odometry. Both belong to the existing start transition.
      navigation_altitude_bounds_ = navigation_start_corridor(
          minimum, maximum, start_z, look_ahead.pose.position.z);
      const auto physical_start = tflib::to_tf(robot_odometry_.pose.pose.position);
      singleton_goal_stop_.bind(goal->global_plan.poses.size(),
          transform * tflib::to_tf(goal->global_plan.poses.back().pose.position), &physical_start);
      global_plan->set_global_plan(std::make_shared<nav_msgs::msg::Path>(goal->global_plan));
    } catch (const std::exception& error) {
      auto result = std::make_shared<NavigateTask::Result>();
      result->success = false; result->message = error.what();
      task_active_ = false; goal_handle->abort(result); return;
    }

    // Set trajectory controller to ADD_SEGMENT mode
    auto mode_req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    mode_req->mode = airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT;
    if (trajectory_mode_client_->wait_for_service(std::chrono::seconds(2)))
      trajectory_mode_client_->async_send_request(mode_req);
    else
      RCLCPP_WARN(get_logger(), "set_trajectory_mode service not available");

    // Goal position is the last pose in the path
    geometry_msgs::msg::Point goal_pos;
    if (!goal->global_plan.poses.empty())
      goal_pos = goal->global_plan.poses.back().pose.position;

    rclcpp::Rate rate(1.0);

    while (rclcpp::ok()) {
      if (cancel_requested_) {
        restore_track_mode();
        auto result = std::make_shared<NavigateTask::Result>();
        result->success = false;
        result->message = "Canceled";
        task_active_ = false;
        goal_handle->canceled(result);
        return;
      }

      // The trajectory-controller tracking point can advance ahead of the
      // vehicle. Only physical map-frame odometry may complete NavigateTask.
      nav_msgs::msg::Odometry physical_odometry;
      bool physical_odometry_valid;
      {
        std::lock_guard<std::mutex> lock(robot_odometry_mutex_);
        physical_odometry = robot_odometry_;
        physical_odometry_valid = robot_odometry_valid_;
      }
      if (physical_odometry_valid && physical_odometry.header.frame_id == goal->global_plan.header.frame_id) {
        double dx = physical_odometry.pose.pose.position.x - goal_pos.x;
        double dy = physical_odometry.pose.pose.position.y - goal_pos.y;
        double dz = physical_odometry.pose.pose.position.z - goal_pos.z;
        float dist = static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz));

        auto feedback = std::make_shared<NavigateTask::Feedback>();
        feedback->status = "navigating";
        feedback->distance_to_goal = dist;
        feedback->current_position.x = physical_odometry.pose.pose.position.x;
        feedback->current_position.y = physical_odometry.pose.pose.position.y;
        feedback->current_position.z = physical_odometry.pose.pose.position.z;
        goal_handle->publish_feedback(feedback);

        if (dist < goal->goal_tolerance_m) {
          restore_track_mode();
          auto result = std::make_shared<NavigateTask::Result>();
          result->success = true;
          result->message = "Goal reached";
          task_active_ = false;
          goal_handle->succeed(result);
          return;
        }
      }

      rate.sleep();
    }

    restore_track_mode();
    auto result = std::make_shared<NavigateTask::Result>();
    result->success = false;
    result->message = "Node shutting down";
    task_active_ = false;
    goal_handle->abort(result);
  }

  void restore_track_mode()
  {
    auto mode_req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    mode_req->mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK;
    trajectory_mode_client_->async_send_request(mode_req);
  }

  /**
   * @brief Callback to manually reset stuck detection
   * @param msg Empty message trigger
   * 
   * Clears the rewind monitor's position history, resetting stuck
   * detection when manually commanded.
   */
  void reset_stuck_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    rewind_monitor->clear_history();
  }

  /**
   * @brief Callback to clear the obstacle map
   * @param msg Empty message trigger
   * 
   * Clears the rewind monitor history and should clear the GL interface
   * obstacle map (not yet implemented).
   */
  void clear_map_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    rewind_monitor->clear_history();
    // TODO gl_interface clear map
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparityExpanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
