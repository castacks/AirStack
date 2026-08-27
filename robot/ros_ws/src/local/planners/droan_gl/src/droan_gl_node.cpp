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
#include <unordered_set>

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
  airstack_msgs::msg::Odometry tracking_point_odom_;
  bool tracking_point_valid_ = false;

  std::string target_frame, look_ahead_frame, rewind_info_frame;
  float deviation_weight, z_deviation_weight, progress_weight, collision_seen_ratio, max_deviation, max_below_plan;
  int collision_min_votes;
  float close_vote_range;
  bool auto_rewind, auto_pause;
  float auto_rewind_blocked_threshold, auto_rewind_duration;
  bool rewinding_ = false;
  double no_safe_traj_since_ = -1., rewind_until_ = -1.;
  bool first_tracking_valid_ = false;
  tf2::Vector3 first_tracking_position_;
  int consecutive_rewinds_ = 0;
  bool paused_ = false;
  double paused_since_ = -1.;
  double no_pause_until_ = -1.;
  double rewind_cooldown_until_ = -1.;
  // Breadcrumbs: recently flown positions (target frame). Physically
  // verified free space — candidates through them are never blocked as
  // 'unseen', which makes retreat/turn arcs plannable with a
  // forward-only camera (collision flags still override).
  std::deque<tf2::Vector3> breadcrumbs_;
  float breadcrumb_radius, breadcrumb_spacing;
  int breadcrumb_max;
  // 360-degree LiDAR close-range veto: the forward-only stereo camera
  // has transient FOV holes (turns, close range, downward escapes); the
  // vehicle's LiDAR sees all around. The latest filtered cloud is
  // voxelized in the target frame and any candidate point within
  // lidar_veto_radius of an occupied voxel is treated as a collision.
  // LiDAR only ever ADDS collision evidence - camera logic is untouched.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  bool lidar_veto;
  float lidar_veto_radius, lidar_voxel;
  std::unordered_set<int64_t> lidar_cells_;
  rclcpp::Time lidar_stamp_{0, 0, RCL_ROS_TIME};
  float lidar_min_range_ = std::numeric_limits<float>::infinity();
  float auto_brake_range;
  std::vector<std::array<int, 3>> lidar_offsets_;
  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
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
    deviation_weight = airstack::get_param(this, "deviation_weight", 2.0);
    z_deviation_weight = airstack::get_param(this, "z_deviation_weight", 4.0);
    progress_weight = airstack::get_param(this, "progress_weight", 1.0);
    collision_seen_ratio = airstack::get_param(this, "collision_seen_ratio", 0.45);
    collision_min_votes = airstack::get_param(this, "collision_min_votes", 2);
    close_vote_range = airstack::get_param(this, "close_vote_range", 8.0);
    max_deviation = airstack::get_param(this, "max_deviation", 10.0);
    max_below_plan = airstack::get_param(this, "max_below_plan", 3.0);
    auto_rewind = airstack::get_param(this, "auto_rewind", false);
    auto_pause = airstack::get_param(this, "auto_pause", true);
    breadcrumb_radius = airstack::get_param(this, "breadcrumb_radius", 1.5);
    breadcrumb_spacing = airstack::get_param(this, "breadcrumb_spacing", 1.0);
    breadcrumb_max = airstack::get_param(this, "breadcrumb_max", 150);
    lidar_veto = airstack::get_param(this, "lidar_veto", true);
    lidar_veto_radius = airstack::get_param(this, "lidar_veto_radius", 2.0);
    lidar_voxel = airstack::get_param(this, "lidar_voxel", 1.0);
    auto_brake_range = airstack::get_param(this, "auto_brake_range", 4.0);
    {
      int r = (int)std::ceil(lidar_veto_radius / lidar_voxel);
      for (int dx = -r; dx <= r; dx++)
        for (int dy = -r; dy <= r; dy++)
          for (int dz = -r; dz <= r; dz++)
            if (std::sqrt(dx * dx + dy * dy + dz * dz) * lidar_voxel <= lidar_veto_radius)
              lidar_offsets_.push_back({dx, dy, dz});
    }
    if (lidar_veto)
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "lidar_cloud", rclcpp::SensorDataQoS(),
          std::bind(&DisparityExpanderNode::lidar_callback, this, std::placeholders::_1));
    auto_rewind_blocked_threshold = airstack::get_param(this, "all_in_collision_duration_threshold", 2.0);
    auto_rewind_duration = airstack::get_param(this, "all_in_collision_rewind_duration", 6.0);

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
  int64_t lidar_key(int x, int y, int z)
  {
    return ((int64_t)(x & 0x1FFFFF) << 42) | ((int64_t)(y & 0x1FFFFF) << 21) |
           (int64_t)(z & 0x1FFFFF);
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    tf2::Stamped<tf2::Transform> to_target;
    try
    {
      auto t = tf_buffer->lookupTransform(target_frame, msg->header.frame_id,
                                          tf2::TimePointZero);
      tf2::fromMsg(t, to_target);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "lidar veto: transform failed: %s", ex.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    lidar_cells_.clear();
    float min_r2 = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < cloud.points.size(); i += 2)
    {
      const auto &p = cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      min_r2 = std::min(min_r2, p.x * p.x + p.y * p.y + p.z * p.z);
      tf2::Vector3 w = to_target * tf2::Vector3(p.x, p.y, p.z);
      lidar_cells_.insert(lidar_key((int)std::floor(w.x() / lidar_voxel),
                                    (int)std::floor(w.y() / lidar_voxel),
                                    (int)std::floor(w.z() / lidar_voxel)));
    }
    lidar_min_range_ = std::sqrt(min_r2);
    lidar_stamp_ = msg->header.stamp;
  }

  bool lidar_occupied(float x, float y, float z)
  {
    if (lidar_cells_.empty() ||
        (get_clock()->now() - lidar_stamp_).seconds() > 2.0)
      return false;
    int cx = (int)std::floor(x / lidar_voxel);
    int cy = (int)std::floor(y / lidar_voxel);
    int cz = (int)std::floor(z / lidar_voxel);
    for (const auto &o : lidar_offsets_)
      if (lidar_cells_.count(lidar_key(cx + o[0], cy + o[1], cz + o[2])))
        return true;
    return false;
  }

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

    // Auto-rewind recovery (boxed state only): when every candidate has
    // been collision/unseen/out-of-bounds for auto_rewind_blocked_threshold
    // seconds, command the trajectory controller's REWIND mode (replay the
    // flown - and therefore known-free - trajectory backwards) for
    // auto_rewind_duration, then resume ADD_SEGMENT. The stuck topic still
    // reports both stuck conditions for an executive; this in-node fallback
    // only fires once the vehicle has displaced >8 m from startup so it can
    // never interfere with takeoff (mission executives own the modes until
    // real navigation has begun).
    double now_s = get_clock()->now().seconds();
    if (rewinding_)
    {
      if (now_s >= rewind_until_)
      {
        send_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT);
        rewinding_ = false;
        paused_ = false;
        no_safe_traj_since_ = -1.;
        rewind_monitor->clear_history();
        RCLCPP_INFO(get_logger(), "auto-rewind finished, resuming ADD_SEGMENT");
      }
      else
      {
        // Safety abort: never rewind below the plan band (a long rewind
        // can replay the takeoff climb backwards, descending into the
        // ground) — resume planning instead.
        if (!breadcrumbs_.empty())
        {
          auto [dxy, dzs, pd] = global_plan->get_distance(
              breadcrumbs_.back().x(), breadcrumbs_.back().y(),
              breadcrumbs_.back().z());
          if (dxy >= 0.f && -dzs > max_below_plan)
          {
            RCLCPP_WARN(get_logger(),
                        "auto-rewind descended below the plan band — aborting");
            send_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT);
            rewinding_ = false;
            no_safe_traj_since_ = -1.;
            rewind_monitor->clear_history();
          }
        }
        return;
      }
    }

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

      // A point is unsafe once collision detections outweigh
      // collision_seen_ratio * free-space observations: stale or
      // long-range keyframes must not outvote a fresh close-range
      // detection (ratio 1.0 restores the old strict-majority rule).
      bool lidar_hit = lidar_veto &&
                       lidar_occupied(state.x(), state.y(), state.z());
      // Range-aware vote: near the vehicle, stereo detections are reliable
      // (large disparity, low error) and stale seen-votes must not dilute
      // them — collision_min_votes vetoes outright. Far away, individual
      // votes are noise-dominated (error grows ~quadratically with range),
      // so free-space observations may outvote them (collision_seen_ratio).
      bool close = !breadcrumbs_.empty() &&
                   breadcrumbs_.back().distance(
                       tf2::Vector3(state.x(), state.y(), state.z())) <
                       close_vote_range;
      bool cam_hit = collision >= collision_min_votes &&
                     (close || collision > collision_seen_ratio * seen);
      if (lidar_hit || cam_hit)
      {
        is_traj_safe = false;
        collision_markers.add_point(state.x(), state.y(), state.z());
        traj_status = COLLISION;
      }
      else if (seen > 1)
        free_markers.add_point(state.x(), state.y(), state.z());
      else
      {
        bool on_breadcrumb = false;
        tf2::Vector3 pt(state.x(), state.y(), state.z());
        for (const auto &b : breadcrumbs_)
          if (b.distance(pt) < breadcrumb_radius)
          {
            on_breadcrumb = true;
            break;
          }
        if (on_breadcrumb)
          free_markers.add_point(state.x(), state.y(), state.z());
        else
        {
          is_traj_safe = false;
          unseen_markers.add_point(state.x(), state.y(), state.z());
          if (traj_status == SEEN)
            traj_status = UNSEEN;
        }
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

        auto [dev_xy, dz_signed, path_distance] = global_plan->get_distance(state.x(), state.y(), state.z());
        // Candidates ending far off the global plan are never eligible,
        // even when they are the only collision-free ones: with a
        // forward-only sensor, the space behind the vehicle goes unseen
        // and an unbounded cost tradeoff can walk the vehicle away from
        // the plan indefinitely (fly-away). Holding position is safer.
        // Hard eligibility bounds (costs alone cannot prevent selection:
        // when every other candidate is collision/unseen, the sole safe one
        // wins regardless of cost). Lateral: cap distance from the plan
        // (anti-fly-away). Vertical: cap how far BELOW the plan a candidate
        // may end — descending escapes leave the camera's downward FOV and
        // textureless ground reads as free, so cheap dives are the recurring
        // collision mechanism. Above-plan is not capped or costed: the judge
        // may command an altitude before the mission engages, legitimately
        // placing the vehicle above a ground-anchored plan.
        float below_plan = std::max(0.f, -dz_signed);
        if (dev_xy > max_deviation || below_plan > max_below_plan)
          continue;
        // RCLCPP_INFO_STREAM(get_logger(), i << " " << traj_status_log << " " << deviation << " " <<  path_distance);
        if (dev_xy >= 0 && path_distance >= 0)
        {
          float cost = deviation_weight * dev_xy + z_deviation_weight * below_plan - progress_weight * path_distance;
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
      if (no_safe_traj_since_ < 0.)
        no_safe_traj_since_ = now_s;
      bool displaced = first_tracking_valid_ && tracking_point_valid_ &&
                       tflib::to_tf(tracking_point_odom_.pose.position)
                           .distance(first_tracking_position_) > 8.0;
      // Collision brake: with no safe candidate the controller would
      // otherwise fly the previously committed segment to its end, through
      // space now known to be occupied (committed-path flyout). Brake ONLY
      // when the LiDAR reports something genuinely close — pausing on every
      // boxed state freezes the momentum/overshoot dynamics that sweep the
      // camera through turns, deadlocking them (turn directions stay
      // 'unseen' while the vehicle is frozen).
      if (auto_pause && displaced && !paused_ && now_s >= no_pause_until_ &&
          lidar_min_range_ < auto_brake_range)
      {
        send_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::PAUSE);
        paused_ = true;
        paused_since_ = now_s;
      }
      // Unstick: a paused, boxed vehicle would hover indefinitely (the
      // forward-only camera cannot resolve 'unseen' without motion).
      // Retreat a few meters along the breadcrumbs — the just-flown,
      // physically verified-free corridor — as an ORDINARY trajectory
      // segment (never the controller's stateful REWIND mode), which
      // moves the camera and reopens planning. Also re-seed the plan
      // progress in case the window ran ahead of the vehicle.
      if (paused_ && now_s - paused_since_ > 5. && breadcrumbs_.size() >= 2)
      {
        // Re-seed plan progress ONLY if the current window is genuinely
        // unreachable (ran ahead of the vehicle): an unconditional global
        // reseed here would jump progress to a later leg whenever route
        // legs pass near each other, deleting the excursion between them.
        {
          tf2::Vector3 cur = breadcrumbs_.back();
          auto [pdx, pdz, ps] = global_plan->get_distance(cur.x(), cur.y(), cur.z());
          (void)pdz;
          (void)ps;
          if (pdx < 0.f || pdx > max_deviation)
            global_plan->reseed_progress(look_ahead);
        }
        airstack_msgs::msg::TrajectoryXYZVYaw retreat;
        retreat.header.stamp = look_ahead.header.stamp;
        retreat.header.frame_id = target_frame;
        double cur_yaw = tf2::getYaw(
            tflib::to_tf(tracking_point_odom_.pose.orientation));
        tf2::Vector3 start = tflib::to_tf(tracking_point_odom_.pose.position);
        double dist = 0.;
        tf2::Vector3 prev = start;
        airstack_msgs::msg::WaypointXYZVYaw w0;
        w0.position.x = start.x();
        w0.position.y = start.y();
        w0.position.z = start.z();
        w0.yaw = cur_yaw;
        w0.velocity = 1.0;
        retreat.waypoints.push_back(w0);
        for (auto it = breadcrumbs_.rbegin();
             it != breadcrumbs_.rend() && dist < 6.0; ++it)
        {
          dist += prev.distance(*it);
          prev = *it;
          airstack_msgs::msg::WaypointXYZVYaw w;
          w.position.x = it->x();
          w.position.y = it->y();
          w.position.z = it->z();
          w.yaw = cur_yaw;
          w.velocity = 1.0;
          retreat.waypoints.push_back(w);
        }
        RCLCPP_WARN(get_logger(),
                    "boxed while paused for 10s — retreating %.1fm along "
                    "the flown corridor", dist);
        send_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT);
        traj_pub->publish(retreat);
        paused_ = false;
        no_pause_until_ = now_s + 8.;
        no_safe_traj_since_ = -1.;
      }
      if (auto_rewind && displaced &&
          now_s - no_safe_traj_since_ > auto_rewind_blocked_threshold &&
          now_s >= rewind_cooldown_until_)
      {
        if (++consecutive_rewinds_ > 3)
        {
          // Repeated rewinds without progress escalate the retreat instead
          // of resolving it — back off for a while.
          rewind_cooldown_until_ = now_s + 60.;
          consecutive_rewinds_ = 0;
          return;
        }
        RCLCPP_WARN(get_logger(),
                    "no safe trajectory for %.1fs — auto-rewinding %.1fs",
                    now_s - no_safe_traj_since_, auto_rewind_duration);
        send_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::REWIND);
        rewinding_ = true;
        paused_ = false;
        rewind_until_ = now_s + auto_rewind_duration;
      }
      return;
    }
    no_safe_traj_since_ = -1.;
    consecutive_rewinds_ = 0;
    if (paused_)
    {
      send_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT);
      paused_ = false;
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
    if (!first_tracking_valid_)
    {
      first_tracking_position_ = tflib::to_tf(msg->pose.position);
      first_tracking_valid_ = true;
    }
    tf2::Vector3 p_map;
    if (tflib::to_frame(tf_buffer, tflib::to_tf(msg->pose.position),
                        msg->header.frame_id, target_frame,
                        msg->header.stamp, &p_map))
    {
      if (breadcrumbs_.empty() ||
          breadcrumbs_.back().distance(p_map) >= breadcrumb_spacing)
      {
        breadcrumbs_.push_back(p_map);
        if ((int)breadcrumbs_.size() > breadcrumb_max)
          breadcrumbs_.pop_front();
      }
    }
  }

  void send_trajectory_mode(uint8_t mode)
  {
    auto req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    req->mode = mode;
    if (trajectory_mode_client_->service_is_ready())
      trajectory_mode_client_->async_send_request(req);
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
      std::shared_ptr<const NavigateTask::Goal> /*goal*/)
  {
    if (task_active_) {
      RCLCPP_WARN(get_logger(), "Rejecting NavigateTask goal: task already active");
      return rclcpp_action::GoalResponse::REJECT;
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

    // Feed the goal path into the planner
    global_plan->set_global_plan(
        std::make_shared<nav_msgs::msg::Path>(goal->global_plan));

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
