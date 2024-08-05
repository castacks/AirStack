#ifndef _LOCAL_PLANNER_H_
#define _LOCAL_PLANNER_H_

// #include <base/BaseNode.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/Range.h>
// #include <string>
// #include <vector>
// #include <list>
// #include <core_trajectory_library/trajectory_library.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Bool.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PointStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <disparity_graph/disparity_graph.h>
// #include <disparity_map_representation/disparity_map_representation.h>
// #include <pointcloud_map_representation/pointcloud_map_representation.h>
// #include <core_map_representation_interface/map_representation.h>
// #include <disparity_map_representation/disparity_map_representation.h>
// #include <tf/transform_listener.h>
// #include <core_trajectory_controller/TrajectoryMode.h>

// #include <actionlib/server/simple_action_server.h>
// #include <behavior_tree/behavior_tree.h>

// #include <pluginlib/class_loader.h>

class LocalPlanner : public BaseNode {
   private:
    TrajectoryLibrary* traj_lib;

    std::string map_representation;
    bool got_global_plan;
    core_trajectory_msgs::msg::TrajectoryXYZVYaw global_plan;
    double global_plan_trajectory_distance;
    bool got_look_ahead, got_tracking_point;
    nav_msgs::msg::Odometry look_ahead_odom, tracking_point_odom;

    std::vector<Trajectory> static_trajectories;

    double waypoint_spacing, obstacle_check_radius, obstacle_penalty_weight,
        forward_progress_penalty_weight;
    double robot_radius;
    int obstacle_check_points;

    double look_past_distance;

    float waypoint_buffer_duration, waypoint_spacing_threshold, waypoint_angle_threshold;
    std::list<geometry_msgs::msg::PointStamped> waypoint_buffer;

    // bool use_fixed_height;
    const int GLOBAL_PLAN_HEIGHT = 0;
    const int FIXED_HEIGHT = 1;
    const int RANGE_SENSOR_HEIGHT = 2;
    int height_mode;
    double height_above_ground;
    double fixed_height;
    bool got_range_up, got_range_down;
    sensor_msgs::msg::Range range_up, range_down;

    const int TRAJECTORY_YAW = 0;
    const int SMOOTH_YAW = 1;
    int yaw_mode;

    // custom waypoint params
    enum GoalMode { CUSTOM_WAYPOINT, AUTO_WAYPOINT, TRAJECTORY };
    GoalMode goal_mode;
    double custom_waypoint_timeout_factor, custom_waypoint_distance_threshold;

    // MapRepresentationDeprecated* map;
    // MapRepresentation* pc_map;
    std::shared_ptr<MapRepresentation> pc_map;

    rclcpp::Subscription<core_trajectory_msgs::msg::TrajectoryXYZVYaw>::SharedPtr global_plan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_up_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_down_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr custom_waypoint_sub;
    // subscribers
    // ros::Subscriber global_plan_sub, waypoint_sub, look_ahead_sub, tracking_point_sub,
    // range_up_sub,
    //    range_down_sub, custom_waypoint_sub;
    // tf::TransformListener* listener;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // publishers
    // ros::Publisher vis_pub, traj_pub, traj_track_pub, obst_vis_pub, global_plan_vis_pub,
    //    look_past_vis_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr vis_pub;
    rclcpp::Publisher<core_trajectory_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
    rclcpp::Publisher<core_trajectory_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_track_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr obst_vis_pub;
    rclcpp::Publisher<core_trajectory_msgs::msg::TrajectoryXYZVYaw>::SharedPtr global_plan_vis_pub;
    rclcpp::Publisher<core_trajectory_msgs::msg::TrajectoryXYZVYaw>::SharedPtr look_past_vis_pub;

    // services
    // ros::ServiceClient traj_mode_client;
    rclcpp::Client<core_trajectory_controller::msg::TrajectoryMode>::SharedPtr traj_mode_client;

    bool get_best_trajectory(std::vector<Trajectory> trajs, Trajectory global_plan,
                             Trajectory* best_traj);
    void update_waypoint_mode();

   public:
    LocalPlanner(const std::string node_name);

    virtual bool initialize();
    virtual bool execute();
    virtual ~LocalPlanner();

    // subscriber callbacks
    void global_plan_callback(
        const core_trajectory_msgs::msg::TrajectoryXYZVYaw::SharedPtr global_plan);
    void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr wp);
    void custom_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr wp);
    void look_ahead_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void tracking_point_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void range_up_callback(const sensor_msgs::msg::Range::SharedPtr range);
    void range_down_callback(const sensor_msgs::msg::Range::SharedPtr range);
};

#endif
