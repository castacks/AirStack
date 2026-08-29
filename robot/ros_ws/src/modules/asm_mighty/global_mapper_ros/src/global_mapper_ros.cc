// Copyright 2017 Massachusetts Institute of Technology
#include "global_mapper_ros/global_mapper_ros.h"

using namespace std::chrono_literals;

namespace global_mapper_ros
{
  GlobalMapperRos::GlobalMapperRos()
      : Node("global_mapper_ros"), publish_occupancy_grid_(false), publish_distance_grid_(false), publish_cost_grid_(false), publish_path_(false), publish_dynamic_grid_(false), publish_obstacle_tracking_(false), clear_unknown_distance_(0.0), target_altitude_(0.0), start_time_(this->now().seconds()), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {

    // og code did not define buffer for some reason
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
    name_drone = this->get_namespace();
    name_drone.erase(std::remove(name_drone.begin(), name_drone.end(), '/'), name_drone.end()); // remove slashes
    lidar_frame_ = name_drone + "/" + name_drone + "_livox";
    // lidar_frame_ = name_drone + "/init_pose";
    // drone_frame_id_ = name_drone + "/base_link";

    // Instantiate cloud pointer to empty cloud message 
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  }

  void GlobalMapperRos::GetParams()
  {

    // --- declare all parameters with sensible defaults:
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("pose_type", "state");  // "state" or "pose_stamped"
    this->declare_parameter<std::string>("drone_frame", "");     // override lidar_frame_ (empty = auto from namespace)
    this->declare_parameter<std::vector<double>>("origin", {1.53, -3.17, 0.82});
    this->declare_parameter<std::vector<double>>("world_dimensions", {16.0, 16.0, 10.0});
    this->declare_parameter<double>("resolution", 0.4);
    this->declare_parameter<double>("clear_unknown_radius", -1.0);
    this->declare_parameter<double>("clear_unknown_height", -1.0);
    this->declare_parameter<double>("clear_occupied_radius", -1.0);
    this->declare_parameter<double>("clear_occupied_height", -1.0);
    this->declare_parameter<double>("z_ground", 0.1);
    this->declare_parameter<int>("skip", 0);
    this->declare_parameter<double>("depth_max", 8.0);
    this->declare_parameter<double>("r1", 0.8);
    this->declare_parameter<double>("r2", 8.0);
    this->declare_parameter<double>("z_min_unknown", 0.1);
    this->declare_parameter<double>("z_max_unknown", 5.0);
    this->declare_parameter<bool>("lock_origin", false);

    // namespaced ones:
    this->declare_parameter<double>("occupancy_grid.init_value", 0.0);
    this->declare_parameter<double>("occupancy_grid.hit_inc", 0.4);
    this->declare_parameter<double>("occupancy_grid.miss_inc", -0.4);
    this->declare_parameter<double>("occupancy_grid.occupancy_threshold", 0.6);
    this->declare_parameter<bool>("occupancy_grid.publish_occupancy_grid", true);
    this->declare_parameter<double>("occupancy_grid.downsample_resolution", 0.0);
    this->declare_parameter<double>("occupancy_grid.occupancy_publish_rate", 5.0);
    this->declare_parameter<double>("occupancy_grid.downsampled_occupancy_publish_rate", 5.0);
    this->declare_parameter<double>("occupancy_grid.unknown_publish_rate", 5.0);
    this->declare_parameter<bool>("occupancy_grid.publish_unknown_grid", true);
    this->declare_parameter<int>("occupancy_grid.min_occupied_neighbors", 0);  // asm_mighty: was read but never declared
    this->declare_parameter<double>("occupancy_grid.clear_unknown_distance", 5.0);

    this->declare_parameter<int>("distance_grid.truncation_distance", 6);
    this->declare_parameter<bool>("distance_grid.publish_distance_grid", false);

    // distance_grid_2d
    this->declare_parameter<bool>("distance_grid_2d.enabled", false);
    this->declare_parameter<int>("distance_grid_2d.truncation_distance", 10);
    this->declare_parameter<bool>("distance_grid_2d.publish", false);
    this->declare_parameter<double>("distance_grid_2d.publish_rate", 5.0);
    this->declare_parameter<double>("distance_grid_2d.height_diff_threshold", 0.3);
    this->declare_parameter<double>("distance_grid_2d.agent_height", 1.0);
    this->declare_parameter<double>("distance_grid_2d.min_obstacle_height", 0.3);
    this->declare_parameter<int>("distance_grid_2d.inflation_cells", 0);
    this->declare_parameter<double>("distance_grid_2d.visibility_check_z", 0.5);

    this->declare_parameter<bool>("cost_grid.publish_cost_grid", false);
    this->declare_parameter<bool>("cost_grid.publish_path", false);
    this->declare_parameter<int>("cost_grid.inflation_distance", 4);
    this->declare_parameter<int>("cost_grid.altitude_weight", 20);
    this->declare_parameter<int>("cost_grid.inflation_weight", 0);
    this->declare_parameter<int>("cost_grid.unknown_weight", 20);
    this->declare_parameter<int>("cost_grid.obstacle_weight", 10000);
    this->declare_parameter<double>("cost_grid.target_altitude", 2.0);
    this->declare_parameter<bool>("temporal_grid.use_temporal_grid", true);
    this->declare_parameter<bool>("temporal_grid.publish_dynamic_grid", true);
    this->declare_parameter<double>("temporal_grid.occupied_thresh", 3.0);
    this->declare_parameter<double>("temporal_grid.unoccupied_thresh", 0.1);
    this->declare_parameter<int>("temporal_grid.dynamic_neighbor_thresh", 5);
    this->declare_parameter<double>("temporal_grid.dynamic_persistence", 1.0);
    this->declare_parameter<double>("temporal_grid.dynamic_maturity_time", 0.0);

    // obstacle tracker parameters
    this->declare_parameter<bool>("obstacle_tracker.enabled", false);
    this->declare_parameter<bool>("obstacle_tracker.use_adaptive_kf", true);
    this->declare_parameter<double>("obstacle_tracker.adaptive_kf_alpha", 0.90);
    this->declare_parameter<double>("obstacle_tracker.adaptive_kf_dt", 0.1);
    this->declare_parameter<double>("obstacle_tracker.cluster_tolerance", 1.0);
    this->declare_parameter<int>("obstacle_tracker.min_cluster_size", 20);
    this->declare_parameter<int>("obstacle_tracker.max_cluster_size", 2000);
    this->declare_parameter<double>("obstacle_tracker.prediction_horizon", 1.0);
    this->declare_parameter<double>("obstacle_tracker.prediction_dt", 0.1);
    this->declare_parameter<double>("obstacle_tracker.time_to_delete_old_obstacles", 5.0);
    this->declare_parameter<double>("obstacle_tracker.cluster_bbox_cutoff_size", 2.0);
    this->declare_parameter<double>("obstacle_tracker.velocity_threshold", 0.8);
    this->declare_parameter<double>("obstacle_tracker.acceleration_threshold", 2.0);
    this->declare_parameter<double>("obstacle_tracker.cutoff_length_threshold", 0.1);
    this->declare_parameter<int>("obstacle_tracker.degree_for_pwp", 3);
    this->declare_parameter<int>("obstacle_tracker.degree_for_poly", 5);
    this->declare_parameter<int>("obstacle_tracker.max_history_size", 30);
    this->declare_parameter<int>("obstacle_tracker.min_observations_for_prediction", 5);
    this->declare_parameter<double>("obstacle_tracker.max_obstacle_velocity", 2.0);

    fla_utils::SafeGetParam(*this, "global_frame", params_.global_frame);
    fla_utils::SafeGetParam(*this, "origin", params_.origin);
    fla_utils::SafeGetParam(*this, "world_dimensions", params_.world_dimensions);
    fla_utils::SafeGetParam(*this, "resolution", params_.resolution);
    fla_utils::SafeGetParam(*this, "clear_unknown_radius", params_.clear_unknown_radius);
    fla_utils::SafeGetParam(*this, "clear_unknown_height", params_.clear_unknown_height);
    fla_utils::SafeGetParam(*this, "clear_occupied_radius", params_.clear_occupied_radius);
    fla_utils::SafeGetParam(*this, "clear_occupied_height", params_.clear_occupied_height);
    fla_utils::SafeGetParam(*this, "z_ground", params_.z_ground);
    fla_utils::SafeGetParam(*this, "skip", params_.skip);
    fla_utils::SafeGetParam(*this, "depth_max", params_.depth_max);
    fla_utils::SafeGetParam(*this, "r1", params_.r1);
    fla_utils::SafeGetParam(*this, "r2", params_.r2);
    fla_utils::SafeGetParam(*this, "z_min_unknown", params_.z_min_unknown);
    fla_utils::SafeGetParam(*this, "z_max_unknown", params_.z_max_unknown);
    fla_utils::SafeGetParam(*this, "lock_origin", params_.lock_origin);

    // occupancy_grid
    fla_utils::SafeGetParam(*this, "occupancy_grid.init_value", params_.init_value);
    fla_utils::SafeGetParam(*this, "occupancy_grid.hit_inc", params_.hit_inc);
    fla_utils::SafeGetParam(*this, "occupancy_grid.miss_inc", params_.miss_inc);
    fla_utils::SafeGetParam(*this, "occupancy_grid.occupancy_threshold", params_.occupancy_threshold);
    fla_utils::SafeGetParam(*this, "occupancy_grid.publish_unknown_grid", publish_unknown_grid_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.publish_occupancy_grid", publish_occupancy_grid_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.downsample_resolution", params_.downsample_resolution);
    fla_utils::SafeGetParam(*this, "occupancy_grid.occupancy_publish_rate", occupancy_publish_rate_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.downsampled_occupancy_publish_rate", downsampled_occupancy_publish_rate_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.min_occupied_neighbors", min_occupied_neighbors_pub_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.unknown_publish_rate", unknown_publish_rate_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.clear_unknown_distance", clear_unknown_distance_);

    // distance_grid
    fla_utils::SafeGetParam(*this, "distance_grid.truncation_distance", params_.truncation_distance);
    fla_utils::SafeGetParam(*this, "distance_grid.publish_distance_grid", publish_distance_grid_);

    // distance_grid_2d
    fla_utils::SafeGetParam(*this, "distance_grid_2d.enabled", params_.use_distance_grid_2d);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.truncation_distance", params_.truncation_distance_2d);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.publish", publish_esdf_2d_);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.publish_rate", esdf_2d_publish_rate_);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.height_diff_threshold", params_.height_diff_threshold);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.agent_height", params_.agent_height);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.min_obstacle_height", params_.min_obstacle_height);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.inflation_cells", params_.inflation_cells_2d);
    fla_utils::SafeGetParam(*this, "distance_grid_2d.visibility_check_z", params_.visibility_check_z);

    // cost_grid
    fla_utils::SafeGetParam(*this, "cost_grid.publish_cost_grid", publish_cost_grid_);
    fla_utils::SafeGetParam(*this, "cost_grid.inflation_distance", params_.inflation_distance);
    fla_utils::SafeGetParam(*this, "cost_grid.publish_path", publish_path_);
    fla_utils::SafeGetParam(*this, "cost_grid.altitude_weight", params_.altitude_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.inflation_weight", params_.inflation_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.unknown_weight", params_.unknown_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.obstacle_weight", params_.obstacle_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.target_altitude", target_altitude_);

    // temporal_grid
    fla_utils::SafeGetParam(*this, "temporal_grid.use_temporal_grid", params_.use_temporal_grid);
    fla_utils::SafeGetParam(*this, "temporal_grid.publish_dynamic_grid", publish_dynamic_grid_);
    fla_utils::SafeGetParam(*this, "temporal_grid.occupied_thresh", params_.temporal_occupied_thresh);
    fla_utils::SafeGetParam(*this, "temporal_grid.unoccupied_thresh", params_.temporal_unoccupied_thresh);
    fla_utils::SafeGetParam(*this, "temporal_grid.dynamic_neighbor_thresh", params_.dynamic_neighbor_thresh);
    fla_utils::SafeGetParam(*this, "temporal_grid.dynamic_persistence", params_.dynamic_persistence);
    fla_utils::SafeGetParam(*this, "temporal_grid.dynamic_maturity_time", params_.dynamic_maturity_time);

    // If temporal grid is off, disable dynamic grid publishing and obstacle tracking
    if (!params_.use_temporal_grid)
    {
      publish_dynamic_grid_ = false;
      RCLCPP_INFO(this->get_logger(), "Temporal grid disabled — dynamic grid and obstacle tracker will not run");
    }

    // obstacle tracker
    fla_utils::SafeGetParam(*this, "obstacle_tracker.enabled", obstacle_tracker_params_.enabled);
    // Obstacle tracker requires temporal grid for dynamic cloud
    if (!params_.use_temporal_grid)
      obstacle_tracker_params_.enabled = false;
    publish_obstacle_tracking_ = obstacle_tracker_params_.enabled;
    fla_utils::SafeGetParam(*this, "obstacle_tracker.use_adaptive_kf", obstacle_tracker_params_.use_adaptive_kf);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.adaptive_kf_alpha", obstacle_tracker_params_.adaptive_kf_alpha);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.adaptive_kf_dt", obstacle_tracker_params_.adaptive_kf_dt);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.cluster_tolerance", obstacle_tracker_params_.cluster_tolerance);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.min_cluster_size", obstacle_tracker_params_.min_cluster_size);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.max_cluster_size", obstacle_tracker_params_.max_cluster_size);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.prediction_horizon", obstacle_tracker_params_.prediction_horizon);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.prediction_dt", obstacle_tracker_params_.prediction_dt);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.time_to_delete_old_obstacles", obstacle_tracker_params_.time_to_delete_old_obstacles);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.cluster_bbox_cutoff_size", obstacle_tracker_params_.cluster_bbox_cutoff_size);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.velocity_threshold", obstacle_tracker_params_.velocity_threshold);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.acceleration_threshold", obstacle_tracker_params_.acceleration_threshold);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.cutoff_length_threshold", obstacle_tracker_params_.cutoff_length_threshold);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.degree_for_pwp", obstacle_tracker_params_.degree_for_pwp);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.degree_for_poly", obstacle_tracker_params_.degree_for_poly);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.max_history_size", obstacle_tracker_params_.max_history_size);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.min_observations_for_prediction", obstacle_tracker_params_.min_observations_for_prediction);
    fla_utils::SafeGetParam(*this, "obstacle_tracker.max_obstacle_velocity", obstacle_tracker_params_.max_obstacle_velocity);

    // Override lidar_frame_ if drone_frame parameter is set
    std::string drone_frame = this->get_parameter("drone_frame").as_string();
    if (!drone_frame.empty())
    {
      lidar_frame_ = drone_frame;
    }

    // Print the parameters to the console
    RCLCPP_INFO(this->get_logger(), "Global Mapper Parameters:");
    RCLCPP_INFO(this->get_logger(), "  drone_frame (lidar_frame_): %s", lidar_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  global_frame: %s", params_.global_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  origin: [%f, %f, %f]", params_.origin[0], params_.origin[1], params_.origin[2]);
    RCLCPP_INFO(this->get_logger(), "  world_dimensions: [%f, %f, %f]", params_.world_dimensions[0], params_.world_dimensions[1], params_.world_dimensions[2]);
    RCLCPP_INFO(this->get_logger(), "  resolution: %f", params_.resolution);
    RCLCPP_INFO(this->get_logger(), "  clear_unknown_radius: %f (effective: %f)", params_.clear_unknown_radius, (params_.clear_unknown_radius >= 0.0) ? params_.clear_unknown_radius : 0.5); // default to 0.5m if not set
    RCLCPP_INFO(this->get_logger(), "  clear_unknown_height: %f (effective: %f)", params_.clear_unknown_height, (params_.clear_unknown_height >= 0.0) ? params_.clear_unknown_height : 0.5); // default to 0.5m if not set
    RCLCPP_INFO(this->get_logger(), "  clear_occupied_radius: %f (disabled if <0)", params_.clear_occupied_radius);
    RCLCPP_INFO(this->get_logger(), "  clear_occupied_height: %f (disabled if <0)", params_.clear_occupied_height);
    RCLCPP_INFO(this->get_logger(), "  z_ground: %f", params_.z_ground);
    RCLCPP_INFO(this->get_logger(), "  skip: %d", params_.skip);
    RCLCPP_INFO(this->get_logger(), "  depth_max: %f", params_.depth_max);
    RCLCPP_INFO(this->get_logger(), "  r1: %f", params_.r1);
    RCLCPP_INFO(this->get_logger(), "  r2: %f", params_.r2);
    RCLCPP_INFO(this->get_logger(), "  z_min_unknown: %f", params_.z_min_unknown);
    RCLCPP_INFO(this->get_logger(), "  z_max_unknown: %f", params_.z_max_unknown);
    RCLCPP_INFO(this->get_logger(), "  lock_origin: %s (true = persistent global map; false = sliding window)",
                params_.lock_origin ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.init_value: %f", params_.init_value);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.hit_inc: %f", params_.hit_inc);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.miss_inc: %f", params_.miss_inc);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.occupancy_threshold: %f", params_.occupancy_threshold);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.publish_unknown_grid: %s", publish_unknown_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.publish_occupancy_grid: %s", publish_occupancy_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.downsample_resolution: %f", params_.downsample_resolution);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.clear_unknown_distance: %f", clear_unknown_distance_);
    RCLCPP_INFO(this->get_logger(), "  distance_grid.truncation_distance: %d", params_.truncation_distance);
    RCLCPP_INFO(this->get_logger(), "  distance_grid.publish_distance_grid: %s", publish_distance_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.enabled: %s", params_.use_distance_grid_2d ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.truncation_distance: %d", params_.truncation_distance_2d);
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.publish: %s", publish_esdf_2d_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.publish_rate: %f", esdf_2d_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.height_diff_threshold: %f", params_.height_diff_threshold);
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.agent_height: %f", params_.agent_height);
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.min_obstacle_height: %f", params_.min_obstacle_height);
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.inflation_cells: %d", params_.inflation_cells_2d);
    RCLCPP_INFO(this->get_logger(), "  distance_grid_2d.visibility_check_z: %f", params_.visibility_check_z);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.publish_cost_grid: %s", publish_cost_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.inflation_distance: %d", params_.inflation_distance);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.publish_path: %s", publish_path_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.altitude_weight: %d", params_.altitude_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.inflation_weight: %d", params_.inflation_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.unknown_weight: %d", params_.unknown_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.obstacle_weight: %d", params_.obstacle_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.target_altitude: %f", target_altitude_);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.use_temporal_grid: %s", params_.use_temporal_grid ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.publish_dynamic_grid: %s", publish_dynamic_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.occupied_thresh: %f", params_.temporal_occupied_thresh);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.unoccupied_thresh: %f", params_.temporal_unoccupied_thresh);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.dynamic_neighbor_thresh: %d", params_.dynamic_neighbor_thresh);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.dynamic_persistence: %f", params_.dynamic_persistence);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.dynamic_maturity_time: %f", params_.dynamic_maturity_time);
    RCLCPP_INFO(this->get_logger(), "  obstacle_tracker.enabled: %s", publish_obstacle_tracking_ ? "true" : "false");
  }

  void GlobalMapperRos::InitSubscribers()
  {
    std::string pose_type = this->get_parameter("pose_type").as_string();
    if (pose_type == "pose_stamped")
    {
      pose_stamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "pose_topic", 1, std::bind(&GlobalMapperRos::PoseStampedCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribing to pose as geometry_msgs/PoseStamped");
    }
    else
    {
      pose_sub_ = this->create_subscription<dynus_interfaces::msg::State>(
          "pose_topic", 1, std::bind(&GlobalMapperRos::PoseCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribing to pose as dynus_interfaces/State");
    }
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_topic", 1, std::bind(&GlobalMapperRos::GoalCallback, this, std::placeholders::_1));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("depth_pointcloud_topic", rclcpp::SensorDataQoS(), std::bind(&GlobalMapperRos::PointCloudCallback, this, std::placeholders::_1));
  }

  void GlobalMapperRos::InitPublishers()
  {

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
    sensor_qos.best_effort().durability_volatile();

    if (publish_occupancy_grid_)
    {
      occ_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupancy_grid_topic", sensor_qos);
      if (params_.downsample_resolution > 0.0)
      {
        downsampled_occ_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("downsampled_occupancy_grid", sensor_qos);
      }
    }

    if (publish_unknown_grid_)
    {
      unknown_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("unknown_grid_topic", sensor_qos);
      frontier_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("frontier_grid_topic", sensor_qos);
    }

    if (publish_distance_grid_)
    {
      dist_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("distance_grid_topic", 10);
    }

    if (publish_esdf_2d_)
    {
      esdf_2d_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("esdf_2d_topic", 10);
      occ_2d_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occ_2d_topic", 10);
    }

    if (publish_cost_grid_)
    {
      cost_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cost_grid_topic", 10);
    }

    if (publish_path_)
    {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_topic", 10);
      sparse_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sparse_path_topic", 10);
    }

    if (publish_dynamic_grid_)
    {
      dynamic_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_grid_topic", 10);
      static_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("static_grid_topic", 10);
    }

    if (publish_obstacle_tracking_)
    {
      predicted_traj_pub_ = this->create_publisher<dynus_interfaces::msg::DynTraj>("predicted_trajs", 10);
      tracker_bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_bounding_boxes", 10);
      tracker_prediction_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacles", 10);
    }

    grid_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&GlobalMapperRos::Publish, this));
  }

  void GlobalMapperRos::PopulateUnknownPointCloudMsg(const voxel_grid::VoxelGrid<float> &occupancy_grid,
                                                     sensor_msgs::msg::PointCloud2 *pointcloud)
  {
    // check for bad input
    if (pointcloud == nullptr)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    Eigen::Vector3d transform;

    try
    {
      // std::cout << "Lidar frame: " << lidar_frame_ << std::endl;
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, lidar_frame_, rclcpp::Time(0), 20ms);
      transform(0) = transform_stamped.transform.translation.x;
      transform(1) = transform_stamped.transform.translation.y;
      transform(2) = transform_stamped.transform.translation.z;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "lookupTransform(%s -> %s) failed: %s", params_.global_frame.c_str(), lidar_frame_.c_str(), ex.what());

      transform(0) = std::numeric_limits<double>::quiet_NaN();
      transform(1) = std::numeric_limits<double>::quiet_NaN();
      transform(2) = std::numeric_limits<double>::quiet_NaN();
    }

    double xyz[3] = {transform(0), transform(1), transform(2)};
    int slice_ixyz[3];
    occupancy_grid.WorldToGrid(xyz, slice_ixyz);

    int grid_dimensions[3];
    occupancy_grid.GetGridDimensions(grid_dimensions);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    double origin[3];
    occupancy_grid.GetOrigin(origin);
    const double r1_sq = params_.r1 * params_.r1;
    const double r2_sq = params_.r2 * params_.r2;

    for (int x = 0; x < grid_dimensions[0]; ++x)
    {
      for (int y = 0; y < grid_dimensions[1]; ++y)
      {
        for (int z = 0; z < grid_dimensions[2]; ++z)
        {
          int ixyz[3] = {x, y, z};
          float occupancy_value = occupancy_grid.ReadValue(ixyz);
          if (global_mapper_ptr_->occupancy_grid_.IsUnknown(occupancy_value))
          {
            occupancy_grid.GridToWorld(ixyz, xyz);
            if (xyz[2] > params_.z_min_unknown && xyz[2] < params_.z_max_unknown)
            {
              double dx = xyz[0] - origin[0];
              double dy = xyz[1] - origin[1];
              double dz = xyz[2] - origin[2];
              double dist_sq = dx * dx + dy * dy + dz * dz;

              if (dist_sq < r2_sq && dist_sq > r1_sq)
              {
                cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
              }
            }
          }
        }
      }
    }

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = rclcpp::Clock().now();
  }

  void GlobalMapperRos::PopulateOccupancyPointCloudMsg(const voxel_grid::VoxelGrid<float> &occupancy_grid,
                                                       sensor_msgs::msg::PointCloud2 *pointcloud)
  {
    // check for bad input
    if (pointcloud == nullptr)
    {
      return;
    }

    int grid_dimensions[3];
    occupancy_grid.GetGridDimensions(grid_dimensions);
    const int min_neighbors = min_occupied_neighbors_pub_;

    double xyz[3] = {0.0};
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        for (int z = 0; z < grid_dimensions[2]; z++)
        {
          int ixyz[3] = {x, y, z};
          float occupancy_value = occupancy_grid.ReadValue(ixyz);
          if (global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_value))
          {
            occupancy_grid.GridToWorld(ixyz, xyz);
            if (xyz[2] <= params_.z_ground)
              continue;

            // Noise filter: drop isolated occupied voxels (26-connectivity neighbor count).
            if (min_neighbors > 0)
            {
              int neighbor_count = 0;
              bool enough = false;
              for (int dx = -1; dx <= 1 && !enough; ++dx)
              {
                for (int dy = -1; dy <= 1 && !enough; ++dy)
                {
                  for (int dz = -1; dz <= 1 && !enough; ++dz)
                  {
                    if (dx == 0 && dy == 0 && dz == 0)
                      continue;
                    const int nx = x + dx, ny = y + dy, nz = z + dz;
                    if (nx < 0 || nx >= grid_dimensions[0] ||
                        ny < 0 || ny >= grid_dimensions[1] ||
                        nz < 0 || nz >= grid_dimensions[2])
                      continue;
                    int nixyz[3] = {nx, ny, nz};
                    if (global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_grid.ReadValue(nixyz)))
                    {
                      if (++neighbor_count >= min_neighbors)
                        enough = true;
                    }
                  }
                }
              }
              if (!enough)
                continue;
            }

            cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
          }
        }
      }
    }

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = rclcpp::Clock().now();
  }

  void GlobalMapperRos::PopulateDistancePointCloudMsg(const voxel_grid::VoxelGrid<int> &distance_grid,
                                                      sensor_msgs::msg::PointCloud2 *pointcloud)
  {
    // check for bad input
    if (pointcloud == nullptr)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    Eigen::Vector3d transform;

    try
    {
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, lidar_frame_, rclcpp::Time(0), 20ms);
      transform(0) = transform_stamped.transform.translation.x;
      transform(1) = transform_stamped.transform.translation.y;
      transform(2) = transform_stamped.transform.translation.z;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "lookupTransform(%s -> %s) failed: %s", params_.global_frame.c_str(), lidar_frame_.c_str(), ex.what());
      transform(0) = std::numeric_limits<double>::quiet_NaN();
      transform(1) = std::numeric_limits<double>::quiet_NaN();
      transform(2) = std::numeric_limits<double>::quiet_NaN();
    }

    int grid_dimensions[3];
    distance_grid.GetGridDimensions(grid_dimensions);

    double xyz[3] = {transform(0), transform(1), transform(2)};
    int slice_ixyz[3];
    distance_grid.WorldToGrid(xyz, slice_ixyz);

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    cloud.reserve(grid_dimensions[0] * grid_dimensions[1]);
    const double max_dist = params_.truncation_distance * params_.truncation_distance;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        int ixyz[3] = {x, y, slice_ixyz[2]};
        int cost = distance_grid.ReadValue(ixyz);
        distance_grid.GridToWorld(ixyz, xyz);
        pcl::PointXYZRGBA point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        point.r = static_cast<uint8_t>((max_dist - cost) / max_dist * 255);
        point.g = 0;
        point.b = 0;
        point.a = 255;
        cloud.push_back(point);
      }
    }

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = this->now();
  }

  void GlobalMapperRos::PopulateCostPointCloudMsg(const voxel_grid::VoxelGrid<int> &cost_grid,
                                                  sensor_msgs::msg::PointCloud2 *pointcloud)
  {
    // check for bad input
    if (pointcloud == nullptr)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    Eigen::Vector3d transform;

    try
    {
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, lidar_frame_, rclcpp::Time(0), 20ms);
      transform(0) = transform_stamped.transform.translation.x;
      transform(1) = transform_stamped.transform.translation.y;
      transform(2) = transform_stamped.transform.translation.z;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "lookupTransform(%s -> %s) failed: %s", params_.global_frame.c_str(), lidar_frame_.c_str(), ex.what());

      transform(0) = std::numeric_limits<double>::quiet_NaN();
      transform(1) = std::numeric_limits<double>::quiet_NaN();
      transform(2) = std::numeric_limits<double>::quiet_NaN();
    }

    int grid_dimensions[3];
    cost_grid.GetGridDimensions(grid_dimensions);

    double xyz[3] = {transform(0), transform(1), transform(2)};
    int slice_ixyz[3];
    cost_grid.WorldToGrid(xyz, slice_ixyz);

    const int num_cells_2d = grid_dimensions[0] * grid_dimensions[1];
    std::vector<int> costs(num_cells_2d);
    std::vector<double> world_x(num_cells_2d);
    std::vector<double> world_y(num_cells_2d);

    double max_cost = 0;
    double min_cost = std::numeric_limits<double>::max();
    int idx = 0;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        int ixyz[3] = {x, y, slice_ixyz[2]};
        int cost = cost_grid.ReadValue(ixyz);
        cost_grid.GridToWorld(ixyz, xyz);
        costs[idx] = cost;
        world_x[idx] = xyz[0];
        world_y[idx] = xyz[1];
        if (cost != cost_grid::MAX_COST)
        {
          if (cost > max_cost) max_cost = cost;
          if (cost < min_cost) min_cost = cost;
        }
        idx++;
      }
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(num_cells_2d);
    const double cost_range = (max_cost > min_cost) ? (max_cost - min_cost) : 1.0;
    const int half_z = grid_dimensions[2] >> 1;
    for (int i = 0; i < num_cells_2d; i++)
    {
      if (costs[i] == cost_grid::MAX_COST) continue;
      pcl::PointXYZ point;
      point.x = world_x[i];
      point.y = world_y[i];
      point.z = (costs[i] - min_cost) / cost_range * half_z;
      cloud.push_back(point);
    }

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = this->now();
  }

  void GlobalMapperRos::PopulateEsdf2DMsg(const distance_grid_2d::DistanceGrid2D& grid,
                                          nav_msgs::msg::OccupancyGrid* msg)
  {
    if (msg == nullptr) return;

    int dims[2];
    grid.GetGridDimensions(dims);
    double resolution = grid.GetResolution();
    int dmax = grid.GetMaxSquaredDistance();

    msg->header.stamp = this->now();
    msg->header.frame_id = params_.global_frame;

    msg->info.resolution = static_cast<float>(resolution);
    msg->info.width = dims[0];
    msg->info.height = dims[1];

    // Origin shifted by -0.5*resolution so cell centers align with 3D voxel positions
    double wx, wy;
    grid.GridToWorld(0, 0, wx, wy);
    msg->info.origin.position.x = wx - 0.5 * resolution;
    msg->info.origin.position.y = wy - 0.5 * resolution;
    msg->info.origin.position.z = params_.z_ground;
    msg->info.origin.orientation.w = 1.0;

    msg->data.resize(dims[0] * dims[1]);
    for (int iy = 0; iy < dims[1]; iy++)
    {
      for (int ix = 0; ix < dims[0]; ix++)
      {
        int dist_sq = grid.ReadValue(ix, iy);
        // Map: 0 (obstacle) → 100, dmax (far) → 0, so higher = closer to obstacle.
        // ReadValue stores *squared* voxel distance; take sqrt before normalizing
        // or the gradient falls off quadratically and nearby cells look saturated.
        int value;
        if (dist_sq == 0)
        {
          value = 100;  // obstacle cell
        }
        else if (dist_sq >= dmax)
        {
          value = 0;  // far from obstacles
        }
        else
        {
          const double d = std::sqrt(static_cast<double>(dist_sq));
          const double d_max = std::sqrt(static_cast<double>(dmax));
          value = static_cast<int>(100.0 * (1.0 - d / d_max));
        }
        // nav_msgs::OccupancyGrid data is row-major: index = iy * width + ix
        msg->data[iy * dims[0] + ix] = static_cast<int8_t>(value);
      }
    }
  }

  void GlobalMapperRos::PopulateOcc2DMsg(const occupancy_grid_2d::OccupancyGrid2D& grid,
                                         nav_msgs::msg::OccupancyGrid* msg)
  {
    if (msg == nullptr) return;

    int dims[2];
    grid.GetGridDimensions(dims);
    double resolution = grid.GetResolution();

    msg->header.stamp = this->now();
    msg->header.frame_id = params_.global_frame;

    msg->info.resolution = static_cast<float>(resolution);
    msg->info.width = dims[0];
    msg->info.height = dims[1];

    double wx, wy;
    grid.GridToWorld(0, 0, wx, wy);
    msg->info.origin.position.x = wx - 0.5 * resolution;
    msg->info.origin.position.y = wy - 0.5 * resolution;
    msg->info.origin.position.z = params_.z_ground;
    msg->info.origin.orientation.w = 1.0;

    msg->data.resize(dims[0] * dims[1]);
    for (int iy = 0; iy < dims[1]; iy++)
    {
      for (int ix = 0; ix < dims[0]; ix++)
      {
        // Cell already holds the nav_msgs/OccupancyGrid code: -1=unknown,
        // 0=free, 100=occupied. Frontier detection downstream needs the
        // unknown distinction.
        msg->data[iy * dims[0] + ix] = grid.GetCell(ix, iy);
      }
    }
  }

  void GlobalMapperRos::PopulatePathMsg(const std::vector<std::array<double, 3>> &path, nav_msgs::msg::Path *path_msg)
  {
    path_msg->header.stamp = this->now();
    path_msg->header.frame_id = params_.global_frame;
    for (const auto &point : path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.position.z = point[2];
      pose.pose.orientation.w = 1.0;
      path_msg->poses.push_back(pose);
    }
  }

void GlobalMapperRos::PopulateDynamicPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                                   const voxel_grid::VoxelGrid<std::array<double, 7>>& temporal_grid,
                                                   float occ_threshold, double dynamic_persistence, double temporal_timestamp,
                                                   const std::vector<TrackedObstacle>& tracked_obstacles,
                                                   sensor_msgs::msg::PointCloud2* dynamic_pointcloud,
                                                   sensor_msgs::msg::PointCloud2* static_pointcloud,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud_out)
{
    if (dynamic_pointcloud == nullptr || static_pointcloud == nullptr)
    {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;
    pcl::PointCloud<pcl::PointXYZ> static_cloud;
    const double z_ground = params_.z_ground;

    int grid_dimensions[3];
    occupancy_grid.GetGridDimensions(grid_dimensions);

    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        for (int z = 0; z < grid_dimensions[2]; z++)
        {
          int ixyz[3] = {x, y, z};

          float occupancy_value = occupancy_grid.ReadValue(ixyz);
          bool is_occupied = (occupancy_value >= occ_threshold);
          if (is_occupied)
          {
            double xyz[3];
            occupancy_grid.GridToWorld(ixyz, xyz);
            if (xyz[2] > z_ground)
            {
              // Check 1: temporal grid says dynamic (free→occupied transition)
              const std::array<double, 7>& voxel = temporal_grid.ReadValue(ixyz);
              bool is_dynamic = (voxel[5] != 0.0) &&
                                (voxel[6] > 0.0) &&
                                (temporal_timestamp - voxel[6]) < dynamic_persistence;

              // Check 2: tracker-driven persistence — voxel is near an actively-tracked obstacle
              if (!is_dynamic)
              {
                for (const auto& obs : tracked_obstacles)
                {
                  if (std::abs(xyz[0] - obs.position.x()) <= obs.bbox.x() &&
                      std::abs(xyz[1] - obs.position.y()) <= obs.bbox.y() &&
                      std::abs(xyz[2] - obs.position.z()) <= obs.bbox.z())
                  {
                    is_dynamic = true;
                    break;
                  }
                }
              }

              if (is_dynamic)
              {
                dynamic_cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
              }
              else
              {
                static_cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
              }
            }
          }
        }
      }
    }

    // Provide raw PCL cloud to caller (for obstacle tracking)
    if (dynamic_cloud_out)
      *dynamic_cloud_out = dynamic_cloud;

    pcl::toROSMsg(dynamic_cloud, *dynamic_pointcloud);
    dynamic_pointcloud->header.frame_id = params_.global_frame;
    dynamic_pointcloud->header.stamp = rclcpp::Clock().now();

    pcl::toROSMsg(static_cloud, *static_pointcloud);
    static_pointcloud->header.frame_id = params_.global_frame;
    static_pointcloud->header.stamp = rclcpp::Clock().now();
}


  void GlobalMapperRos::Publish()
  {
    // Per-grid rate throttling
    auto now = this->now();
    auto elapsed_sec = [&](const rclcpp::Time &last) -> double {
      return (now - last).seconds();
    };
    bool pub_occ = publish_occupancy_grid_ && (occupancy_publish_rate_ <= 0.0 ||
                   elapsed_sec(last_occ_pub_time_) >= 1.0 / occupancy_publish_rate_);
    bool pub_ds_occ = publish_occupancy_grid_ && params_.downsample_resolution > 0.0 &&
                      downsampled_occ_grid_pub_ &&
                      (downsampled_occupancy_publish_rate_ <= 0.0 ||
                       elapsed_sec(last_ds_occ_pub_time_) >= 1.0 / downsampled_occupancy_publish_rate_);
    bool pub_unknown = publish_unknown_grid_ && (unknown_publish_rate_ <= 0.0 ||
                       elapsed_sec(last_unknown_pub_time_) >= 1.0 / unknown_publish_rate_);
    bool pub_esdf_2d = publish_esdf_2d_ && (esdf_2d_publish_rate_ <= 0.0 ||
                       elapsed_sec(last_esdf_2d_pub_time_) >= 1.0 / esdf_2d_publish_rate_);

    // Only copy grids that are actually needed for publishing
    const bool need_occupancy = pub_occ || pub_ds_occ || pub_unknown;
    const bool need_distance = publish_distance_grid_;
    const bool need_cost = publish_cost_grid_;
    const bool need_esdf_2d = pub_esdf_2d;
    const bool need_dynamic = publish_dynamic_grid_ || publish_obstacle_tracking_;

    // Single lock acquisition: snapshot ALL needed grids at once, then release.
    // This minimizes contention with the Spin thread.
    voxel_grid::VoxelGrid<float> occupancy_grid;
    voxel_grid::VoxelGrid<int> distance_grid;
    voxel_grid::VoxelGrid<int> cost_grid;
    voxel_grid::VoxelGrid<std::array<double, 7>> temporal_grid;
    distance_grid_2d::DistanceGrid2D distance_grid_2d;
    occupancy_grid_2d::OccupancyGrid2D occupancy_grid_2d;
    float occ_threshold = 0.0f;
    double dynamic_persistence = 0.0;
    double temporal_timestamp = 0.0;

    {
      std::lock_guard<std::mutex> lock(global_mapper_ptr_->output_mutex_);
      if (need_occupancy || need_dynamic) occupancy_grid = global_mapper_ptr_->occupancy_grid_;
      if (need_distance) distance_grid = global_mapper_ptr_->distance_grid_;
      if (need_cost) cost_grid = global_mapper_ptr_->cost_grid_;
      if (need_esdf_2d) {
        distance_grid_2d = global_mapper_ptr_->distance_grid_2d_;
        occupancy_grid_2d = global_mapper_ptr_->occupancy_grid_2d_;
      }
      if (need_dynamic)
      {
        temporal_grid = global_mapper_ptr_->temporal_grid_;
        occ_threshold = global_mapper_ptr_->occupancy_grid_.GetThreshold();
        dynamic_persistence = global_mapper_ptr_->params_.dynamic_persistence;
        temporal_timestamp = global_mapper_ptr_->timestamp_;
      }
    }

    // Populate occupancy pointcloud once if any occupancy-based grid needs it
    sensor_msgs::msg::PointCloud2 occ_pointcloud_msg;
    if (pub_occ || pub_ds_occ)
    {
      PopulateOccupancyPointCloudMsg(occupancy_grid, &occ_pointcloud_msg);
    }

    if (pub_occ)
    {
      occ_grid_pub_->publish(occ_pointcloud_msg);
      last_occ_pub_time_ = now;
    }

    if (pub_ds_occ)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(occ_pointcloud_msg, *full_cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(full_cloud);
      const float leaf = static_cast<float>(params_.downsample_resolution);
      voxel_filter.setLeafSize(leaf, leaf, leaf);
      voxel_filter.filter(*ds_cloud);

      sensor_msgs::msg::PointCloud2 ds_msg;
      pcl::toROSMsg(*ds_cloud, ds_msg);
      ds_msg.header = occ_pointcloud_msg.header;
      downsampled_occ_grid_pub_->publish(ds_msg);
      last_ds_occ_pub_time_ = now;
    }

    if (pub_unknown)
    {
      sensor_msgs::msg::PointCloud2 unknown_pointcloud_msg;
      PopulateUnknownPointCloudMsg(occupancy_grid, &unknown_pointcloud_msg);
      unknown_grid_pub_->publish(unknown_pointcloud_msg);
      last_unknown_pub_time_ = now;
    }

    if (publish_distance_grid_)
    {
      sensor_msgs::msg::PointCloud2 dist_pointcloud_msg;
      PopulateDistancePointCloudMsg(distance_grid, &dist_pointcloud_msg);
      dist_grid_pub_->publish(dist_pointcloud_msg);
    }

    if (pub_esdf_2d)
    {
      nav_msgs::msg::OccupancyGrid esdf_2d_msg;
      PopulateEsdf2DMsg(distance_grid_2d, &esdf_2d_msg);
      esdf_2d_pub_->publish(esdf_2d_msg);

      nav_msgs::msg::OccupancyGrid occ_2d_msg;
      PopulateOcc2DMsg(occupancy_grid_2d, &occ_2d_msg);
      occ_2d_pub_->publish(occ_2d_msg);

      last_esdf_2d_pub_time_ = now;
    }

    if (publish_cost_grid_)
    {
      sensor_msgs::msg::PointCloud2 cost_pointcloud_msg;
      PopulateCostPointCloudMsg(cost_grid, &cost_pointcloud_msg);
      cost_grid_pub_->publish(cost_pointcloud_msg);
    }

    if (publish_path_)
    {
      std::vector<std::array<double, 3>> dense_path, sparse_path;
      global_mapper_ptr_->GetPaths(&dense_path, &sparse_path);

      nav_msgs::msg::Path dense_path_msg, sparse_path_msg;
      PopulatePathMsg(dense_path, &dense_path_msg);
      PopulatePathMsg(sparse_path, &sparse_path_msg);

      path_pub_->publish(dense_path_msg);
      sparse_path_pub_->publish(sparse_path_msg);
    }

    if (publish_dynamic_grid_ || publish_obstacle_tracking_)
    {
      sensor_msgs::msg::PointCloud2 dynamic_pointcloud_msg;
      sensor_msgs::msg::PointCloud2 static_pointcloud_msg;

      // Only create the PCL output cloud if obstacle tracking needs it
      pcl::PointCloud<pcl::PointXYZ>::Ptr dyn_cloud_pcl;
      if (publish_obstacle_tracking_)
        dyn_cloud_pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

      // Get tracked obstacles from previous frame for tracker-driven persistence.
      // This feeds back tracker state so occupied voxels near tracked obstacles
      // stay classified as dynamic even when the temporal grid says static.
      std::vector<TrackedObstacle> tracked_obstacles;
      if (obstacle_tracker_)
        tracked_obstacles = obstacle_tracker_->getTrackedObstacles();

      PopulateDynamicPointCloudMsg(occupancy_grid, temporal_grid,
                                   occ_threshold, dynamic_persistence, temporal_timestamp,
                                   tracked_obstacles,
                                   &dynamic_pointcloud_msg, &static_pointcloud_msg,
                                   dyn_cloud_pcl);

      if (publish_dynamic_grid_)
      {
        dynamic_grid_pub_->publish(dynamic_pointcloud_msg);
        static_grid_pub_->publish(static_pointcloud_msg);
      }

      // Run obstacle tracker on the dynamic cloud
      if (publish_obstacle_tracking_ && obstacle_tracker_)
      {
        double now_sec = this->now().seconds();
        auto tracking_result = obstacle_tracker_->update(dyn_cloud_pcl, now_sec);

        // Publish predicted trajectories
        for (auto& traj : tracking_result.trajectories)
        {
          traj.header.stamp = this->now();
          predicted_traj_pub_->publish(traj);
        }

        // Publish visualization markers
        if (!tracking_result.bbox_markers.markers.empty())
          tracker_bbox_pub_->publish(tracking_result.bbox_markers);
        if (!tracking_result.prediction_markers.markers.empty())
          tracker_prediction_pub_->publish(tracking_result.prediction_markers);
      }

    }
  }

  // Callback for Odometry (jackal)
  void GlobalMapperRos::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
    // std::cout << "In odom Callback########################" << std::endl;
    RCLCPP_INFO(this->get_logger(), "In OdomCallback ########################");
    double xyz[3] = {odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y, odom_ptr->pose.pose.position.z};

    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
    {
      RCLCPP_WARN(this->get_logger(), "Received invalid odometry position. Skipping update.");
      return;
    }

    if (!got_pose_)
    {
      got_pose_ = true;
    }
    global_mapper_ptr_->UpdateOrigin(xyz);
  }

  void GlobalMapperRos::PoseCallback(const dynus_interfaces::msg::State::SharedPtr pose_ptr)
  {
    double xyz[3] = {pose_ptr->pos.x, pose_ptr->pos.y, pose_ptr->pos.z};
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
    {
      RCLCPP_WARN(this->get_logger(), "Received invalid pose position. Skipping update.");
      return;
    }
    if (!got_pose_)
    {
      got_pose_ = true;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received pose: [%.2f, %.2f, %.2f]", xyz[0], xyz[1], xyz[2]);
    global_mapper_ptr_->UpdateOrigin(xyz);
  }

  void GlobalMapperRos::PoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_ptr)
  {
    double xyz[3] = {pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z};
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
    {
      RCLCPP_WARN(this->get_logger(), "Received invalid PoseStamped position. Skipping update.");
      return;
    }
    if (!got_pose_)
    {
      got_pose_ = true;
    }
    global_mapper_ptr_->UpdateOrigin(xyz);
  }

  void GlobalMapperRos::GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_ptr)
  {
    double xyz[3] = {goal_ptr->pose.position.x, goal_ptr->pose.position.y, target_altitude_};
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
    {
      RCLCPP_WARN(this->get_logger(), "Received invalid goal coordinates. Skipping.");
      return;
    }
    if (!got_goal_)
    {
      got_goal_ = true;
    }
    RCLCPP_DEBUG(this->get_logger(), "Goal set to: [%.2f, %.2f, %.2f]", xyz[0], xyz[1], xyz[2]);
    global_mapper_ptr_->SetGoal(xyz);
  }

  void GlobalMapperRos::PointCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
  {
    // 1) Receipt log
    // RCLCPP_INFO(this->get_logger(), "Mapper:: PointCloud received"); // TODO: Uncomment
    if (!got_depth_image_)
    {
      got_depth_image_ = true;
    }

    // 2) Convert ROS2 PointCloud2 → PCL
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(*cloud_msg, tmp);

    // 3) Look up cloud → map transform
    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_ptr_->lookupTransform(
          params_.global_frame,
          cloud_msg->header.frame_id,
          rclcpp::Time(0),
          rclcpp::Duration(std::chrono::milliseconds(20)));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PointCloudCallback] lookupTransform failed: %s", ex.what());
      return;
    }

    // 4) Build Eigen matrix, guard NaN/Inf, cast to float
    Eigen::Matrix4d mat_d = tf2::transformToEigen(tf_stamped).matrix();
    if (!mat_d.allFinite())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Transform matrix contains NaN/Inf, skipping cloud");
      return;
    }
    Eigen::Matrix4f mat_f = mat_d.cast<float>();

    // 5) Convert and transform in a single pass
    auto world_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    world_cloud->points.reserve(tmp.size());
    for (const auto &pt : tmp.points)
    {
      Eigen::Vector4f vt = mat_f * Eigen::Vector4f(pt.x, pt.y, pt.z, 1.0f);
      pcl::PointXYZI wpt;
      wpt.x = vt.x();
      wpt.y = vt.y();
      wpt.z = vt.z();
      wpt.intensity = 1.0f;
      world_cloud->points.push_back(wpt);
    }

    // 6) Fill sensor origin for ray tracing
    world_cloud->sensor_origin_ << tf_stamped.transform.translation.x,
        tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z,
        1.0f;

    // 7) Push into mapper
    global_mapper_ptr_->PushPointCloud(world_cloud, this->now().seconds());

    // 8) Update last-fused timestamp
    tstampLastPclFused_ = cloud_msg->header.stamp;

    // 9) Copy cloud pointer 
    pcl::copyPointCloud(*world_cloud, *cloud_);
  }

  void GlobalMapperRos::Run()
  {
    GetParams();
    InitSubscribers();
    InitPublishers();

    // ── create & store the ProcessStatus on the heap ──
    // `shared_from_this()` is your node pointer
    this->process_status_ =
        std::make_shared<fla_utils::ProcessStatus>(
            this->shared_from_this(), // your node::SharedPtr
            44,                       // ID
            2.0                       // publish rate (Hz)
        );

    // initial state
    this->process_status_->SetStatus(
        fla_interfaces::msg::ProcessStatus::READY);
    this->process_status_->SetArg(0);

    // start your mapping thread
    global_mapper_ptr_ = std::make_unique<
        global_mapper::GlobalMapper>(params_);

    global_mapper_ptr_->Run();

    // Instantiate obstacle tracker if enabled
    if (publish_obstacle_tracking_)
    {
      obstacle_tracker_ = std::make_unique<ObstacleTracker>(
          obstacle_tracker_params_, this->get_logger());
      RCLCPP_INFO(this->get_logger(), "Obstacle tracker enabled and initialized");
    }

    // ── spin loop ──
    rclcpp::Rate spin_rate(100.0);
    while (rclcpp::ok())
    {      
      if (!got_pose_)
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::ALARM);
        this->process_status_->SetArg(ProcessArgs::NO_POSE);
      }
      else if (!got_goal_)
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::ALARM);
        this->process_status_->SetArg(ProcessArgs::NO_GOAL);
      }
      else if (!got_depth_image_)
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::ALARM);
        this->process_status_->SetArg(ProcessArgs::NO_DEPTH_IMAGE);
      }
      else
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::READY);
        this->process_status_->SetArg(ProcessArgs::NOMINAL);
      }

      rclcpp::spin_some(this->shared_from_this());
      spin_rate.sleep();
    }

    
  }

} // namespace global_mapper_ros

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<global_mapper_ros::GlobalMapperRos>();
  RCLCPP_INFO(node->get_logger(), "Global Mapper ROS Loop Started.");
  node->Run();

  rclcpp::shutdown();
  return 0;
}