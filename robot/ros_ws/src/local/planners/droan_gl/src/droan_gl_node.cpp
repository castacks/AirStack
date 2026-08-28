#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <mutex>
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
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

#include <droan_gl/gl_interface.hpp>
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

  rclcpp::TimerBase::SharedPtr timer;

  // Action server (NavigateTask)
  using NavigateTask = task_msgs::action::NavigateTask;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateTask>;
  rclcpp_action::Server<NavigateTask>::SharedPtr navigate_action_server_;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr trajectory_mode_client_;
  std::atomic<bool> task_active_{false};
  std::atomic<bool> cancel_requested_{false};
  // THE ONE ACTIVE NavigateTask. A new goal takes over under nav_mutex_: the
  // previous handle is aborted ("Preempted ...") right there, and the loop
  // serving it sees it is no longer active_nav_ and returns without touching
  // the plan, the track mode or the speed cap — those belong to the successor
  // now. That is what lets a planner change speed by re-sending its activator
  // instead of cancelling and racing the 1 Hz loop for the re-send.
  std::mutex nav_mutex_;
  std::shared_ptr<NavigateGoalHandle> active_nav_;
  airstack_msgs::msg::Odometry tracking_point_odom_;
  bool tracking_point_valid_ = false;

  std::string target_frame, look_ahead_frame, rewind_info_frame;
  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::vector<TrajectoryPoint> trajectory_points;
  vis::MarkerArray traj_markers;
  bool visualize;
  int min_seen_views;

  GLInterface *gl_interface;
  double max_velocity;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle;
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

    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    look_ahead_frame = airstack::get_param(this, "look_ahead_frame", std::string("look_ahead_point_stabilized"));
    rewind_info_frame = airstack::get_param(this, "rewind_info_frame", std::string("base_link_stabilized"));
    visualize = airstack::get_param(this, "visualize", true);
    // How many stored camera views must have SEEN a trajectory point (found it
    // in front of the observed surface) before it counts as free. 2 is the
    // historical hardcoded value. It deadlocks after a VERTICAL takeoff: the
    // graph then holds a column of views stacked under the hover point, the
    // airspace around the drone is inside at most ONE of them, every candidate
    // is "unseen", no trajectory is ever published — and no new view is
    // stored until the drone moves 1 m. Measured on the 250 m suburb bench:
    // 0 free trajectories for 40 min at hover with the plan 13 m away.
    // 1 lets the current view alone free a point, which is what breaks the
    // deadlock; raise it back once the drone is moving if two-view
    // confirmation matters more than getting off the pad.
    min_seen_views = airstack::get_param(this, "min_seen_views", 2);

    look_ahead_valid = false;

    gl_interface = new GLInterface(this, tf_buffer);
    // `max_velocity`: the rollout speed cap, LIVE. The search planners set it
    // per intent (3 m/s crossing the sector, 1.5 m/s approaching a person);
    // droan_gl used to hard-code 2 m/s in its trajectory params.
    max_velocity = airstack::get_param(this, "max_velocity", 2.0);
    gl_interface->set_vel_max((float)max_velocity);
    param_cb_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          for (const auto &p : params)
          {
            if (p.get_name() == "max_velocity")
            {
              double v = p.as_double();
              if (v <= 0.0 || v > 15.0)
              {
                res.successful = false;
                res.reason = "max_velocity must be in (0, 15] m/s";
                break;
              }
              max_velocity = v;
              gl_interface->set_vel_max((float)v);
            }
          }
          return res;
        });
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
    if (!look_ahead_valid)
      return;

    std_msgs::msg::Bool stuck_msg;
    stuck_msg.data = rewind_monitor->should_rewind();
    stuck_pub->publish(stuck_msg);
    rewind_monitor->publish_vis(rewind_info_pub, rewind_info_frame);

    tf2::Transform look_ahead_to_target_tf;
    gl_interface->evaluate_trajectories(look_ahead, trajectory_points, look_ahead_to_target_tf);
    if (trajectory_points.empty())
      return;

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
    float best_traj_cost = std::numeric_limits<float>::infinity();
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

      traj_points[point_index] = tf2::Vector3(state.x(), state.y(), state.z());

      if (collision > 0 && collision > seen)
      {
        is_traj_safe = false;
        collision_markers.add_point(state.x(), state.y(), state.z());
        traj_status = COLLISION;
      }
      else if (seen >= min_seen_views)
        free_markers.add_point(state.x(), state.y(), state.z());
      else
      {
        is_traj_safe = false;
        unseen_markers.add_point(state.x(), state.y(), state.z());
        if (traj_status == SEEN)
          traj_status = UNSEEN;
      }

      // if last waypoint in trajectory
      if (point_index == (gl_interface->get_traj_size() - 1))
      {
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
        if (!is_traj_safe)
        {
          is_traj_safe = true;
          continue;
        }

        auto [deviation, path_distance] = global_plan->get_distance(state.x(), state.y(), state.z());
        // RCLCPP_INFO_STREAM(get_logger(), i << " " << traj_status_log << " " << deviation << " " <<  path_distance);
        if (deviation >= 0 && path_distance >= 0)
        {
          // TODO add weights as ros parameters
          float cost = deviation - path_distance;
          if (cost < best_traj_cost)
          {
            best_traj_cost = cost;
            best_traj_index = traj_index;
          }
        }
      }
    }

    traj_debug_pub_->publish(traj_markers.get_marker_array());
    global_plan->publish_vis(global_plan_vis_pub);

    if (best_traj_index < 0)
    {
      rewind_monitor->found_trajectory(false);
      return;
    }
    rewind_monitor->found_trajectory(true);

    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    for (int i = 0; i < gl_interface->get_traj_size(); i++)
    {
      airstack_msgs::msg::WaypointXYZVYaw wp;

      TrajectoryPoint &state = trajectory_points[best_traj_index * gl_interface->get_traj_size() + i];
      tf2::Vector3 p(state.x(), state.y(), state.z());
      p = look_ahead_to_target_tf * p;

      wp.position.x = p.x();
      wp.position.y = p.y();
      wp.position.z = p.z();
      wp.velocity = state.get_vel();

      traj.waypoints.push_back(wp);
    }

    traj.header.stamp = look_ahead.header.stamp;
    traj.header.frame_id = look_ahead_frame;
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
    look_ahead = *msg;
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
    global_plan->set_global_plan(msg);
  }

  // ---------------------------------------------------------------------------
  // NavigateTask action server
  // ---------------------------------------------------------------------------

  rclcpp_action::GoalResponse handle_navigate_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const NavigateTask::Goal> goal)
  {
    if (goal->max_speed_mps < 0.f || goal->max_speed_mps > 15.f) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting NavigateTask goal: max_speed_mps %.1f must be 0 (node "
                  "default) or in (0, 15] m/s", goal->max_speed_mps);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (task_active_)
      RCLCPP_INFO(get_logger(),
                  "NavigateTask goal accepted while one is active: it PREEMPTS the "
                  "running goal");
    task_active_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_navigate_cancel(
      std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    // Only the ACTIVE goal can be cancelled into a stop; a cancel arriving for
    // a goal that was already preempted must not stop its successor.
    std::lock_guard<std::mutex> lk(nav_mutex_);
    if (goal_handle == active_nav_)
      cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_navigate_accepted(std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    std::thread{std::bind(&DisparityExpanderNode::execute_navigate, this,
                          std::placeholders::_1), goal_handle}.detach();
  }

  // Terminal transition for `goal_handle` — succeed (0), abort (1) or
  // canceled (2) — only if it is STILL the active goal. A preempted goal was
  // already aborted by its successor under the same lock, and then nothing
  // here may touch the handle, the track mode or the speed cap. Returns
  // whether this call owned the finish.
  bool finish_navigate(const std::shared_ptr<NavigateGoalHandle> &goal_handle,
                       int how, const std::string &message)
  {
    std::lock_guard<std::mutex> lk(nav_mutex_);
    if (active_nav_ != goal_handle)
      return false;
    auto result = std::make_shared<NavigateTask::Result>();
    result->success = (how == 0);
    result->message = message;
    if (how == 0)
      goal_handle->succeed(result);
    else if (how == 2)
      goal_handle->canceled(result);
    else
      goal_handle->abort(result);
    active_nav_ = nullptr;
    task_active_ = false;
    return true;
  }

  // The goal is done and nothing succeeded it: hand the controller back and
  // drop the rollout cap to the node's own `max_velocity`.
  void release_navigate(const char *why)
  {
    restore_track_mode();
    gl_interface->set_vel_max((float)max_velocity);
    RCLCPP_INFO(get_logger(), "NavigateTask %s: speed cap back to node default %.1f m/s",
                why, max_velocity);
  }

  void execute_navigate(std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    // TAKE OVER, under the lock, so the loop serving a running goal can never
    // race this on its handle (see active_nav_).
    bool preempted_prev = false;
    {
      std::lock_guard<std::mutex> lk(nav_mutex_);
      if (active_nav_) {
        auto result = std::make_shared<NavigateTask::Result>();
        result->success = false;
        result->message = "Preempted by a new NavigateTask goal";
        active_nav_->abort(result);
        preempted_prev = true;
      }
      active_nav_ = goal_handle;
      cancel_requested_ = false;
    }

    // A goal with a path sets the plan and completes on arrival. An EMPTY goal
    // is a pure activator: leave the plan alone (steer by the global_plan topic,
    // e.g. raven) and run until cancelled — keeps a single steering source.
    const bool have_goal_plan = !goal->global_plan.poses.empty();
    if (have_goal_plan)
      global_plan->set_global_plan(
          std::make_shared<nav_msgs::msg::Path>(goal->global_plan));

    // THE SPEED RIDES ON THE GOAL. max_speed_mps > 0 caps this goal's rollouts
    // (applied on the GL thread, which logs "rollout speed cap -> N m/s");
    // 0 leaves the node's `max_velocity` parameter in charge. Logged here so a
    // run's log shows what every goal asked for and whether it came from the
    // goal or the default.
    const float cap = goal->max_speed_mps > 0.f ? goal->max_speed_mps
                                               : (float)max_velocity;
    gl_interface->set_vel_max(cap);
    RCLCPP_INFO(get_logger(),
                "NavigateTask accepted: %s, max_speed %.1f m/s (%s)%s",
                have_goal_plan ? "plan with poses" : "activator (steer by /global_plan)",
                cap,
                goal->max_speed_mps > 0.f ? "from the goal" : "node default max_velocity",
                preempted_prev ? " — preempting the previous goal" : "");

    // Set trajectory controller to ADD_SEGMENT mode
    auto mode_req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    mode_req->mode = airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT;
    if (trajectory_mode_client_->wait_for_service(std::chrono::seconds(2)))
      trajectory_mode_client_->async_send_request(mode_req);
    else
      RCLCPP_WARN(get_logger(), "set_trajectory_mode service not available");

    // Goal position is the last pose in the path (only meaningful when the goal
    // carries one; an activator goal runs until cancelled).
    geometry_msgs::msg::Point goal_pos;
    if (have_goal_plan)
      goal_pos = goal->global_plan.poses.back().pose.position;

    rclcpp::Rate rate(1.0);

    while (rclcpp::ok()) {
      {
        // Preempted: the successor owns the plan, the mode and the cap.
        std::lock_guard<std::mutex> lk(nav_mutex_);
        if (active_nav_ != goal_handle)
          return;
      }

      if (cancel_requested_) {
        if (finish_navigate(goal_handle, 2, "Canceled"))
          release_navigate("canceled");
        return;
      }

      if (tracking_point_valid_) {
        double dx = tracking_point_odom_.pose.position.x - goal_pos.x;
        double dy = tracking_point_odom_.pose.position.y - goal_pos.y;
        double dz = tracking_point_odom_.pose.position.z - goal_pos.z;
        float dist = static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz));

        auto feedback = std::make_shared<NavigateTask::Feedback>();
        feedback->status = "navigating";
        feedback->distance_to_goal = dist;
        feedback->current_position.x = tracking_point_odom_.pose.position.x;
        feedback->current_position.y = tracking_point_odom_.pose.position.y;
        feedback->current_position.z = tracking_point_odom_.pose.position.z;
        {
          std::lock_guard<std::mutex> lk(nav_mutex_);
          if (active_nav_ != goal_handle)
            return;
          goal_handle->publish_feedback(feedback);
        }

        if (have_goal_plan && dist < goal->goal_tolerance_m) {
          if (finish_navigate(goal_handle, 0, "Goal reached"))
            release_navigate("goal reached");
          return;
        }
      }

      rate.sleep();
    }

    if (finish_navigate(goal_handle, 1, "Node shutting down"))
      release_navigate("node shutting down");
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
