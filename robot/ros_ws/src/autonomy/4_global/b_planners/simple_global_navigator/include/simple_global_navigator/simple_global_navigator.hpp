#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <random>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "task_msgs/action/navigation_task.hpp"

struct RRTNode {
    geometry_msgs::msg::Point position;
    std::shared_ptr<RRTNode> parent;
    double cost;
    std::vector<std::shared_ptr<RRTNode>> children;
    
    RRTNode(const geometry_msgs::msg::Point& pos, std::shared_ptr<RRTNode> par = nullptr, double c = 0.0)
        : position(pos), parent(par), cost(c) {}
};

struct CostMapData {
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<float> costs;
    double resolution;
    geometry_msgs::msg::Point min_bounds;
    geometry_msgs::msg::Point max_bounds;
    bool valid;
    
    CostMapData() : resolution(0.1), valid(false) {}
};

class SimpleGlobalNavigator : public rclcpp::Node {
   public:
    using NavigationTask = task_msgs::action::NavigationTask;
    using GoalHandleNavigationTask = rclcpp_action::ServerGoalHandle<NavigationTask>;

    SimpleGlobalNavigator(const rclcpp::NodeOptions& options)
        : Node("simple_global_navigator", options), 
          rng_(std::random_device{}()),
          current_goal_index_(0) {
        
        RCLCPP_INFO(this->get_logger(), "Simple Global Navigator initialized.");

        // Declare parameters
        this->declare_parameter("rrt_max_iterations", 5000);
        this->declare_parameter("rrt_step_size", 5.0);
        this->declare_parameter("rrt_goal_tolerance", 2.0);
        this->declare_parameter("rrt_rewire_radius", 2.0);
        this->declare_parameter("cost_map_topic", "/cost_map");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("enable_debug_visualization", true);

        // Get parameters
        rrt_max_iterations_ = this->get_parameter("rrt_max_iterations").as_int();
        rrt_step_size_ = this->get_parameter("rrt_step_size").as_double();
        rrt_goal_tolerance_ = this->get_parameter("rrt_goal_tolerance").as_double();
        rrt_rewire_radius_ = this->get_parameter("rrt_rewire_radius").as_double();
        enable_debug_visualization_ = this->get_parameter("enable_debug_visualization").as_bool();

        // Publishers
        global_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "global_plan", rclcpp::QoS(10).transient_local());
        
        if (enable_debug_visualization_) {
            rrt_tree_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "rrt_tree_markers", rclcpp::QoS(10));
            RCLCPP_INFO(this->get_logger(), "RRT tree debug visualization enabled");
        }

        // Subscribers
        cost_map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cost_map_topic").as_string(), 
            rclcpp::QoS(10),
            std::bind(&SimpleGlobalNavigator::cost_map_callback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(),
            rclcpp::QoS(10),
            std::bind(&SimpleGlobalNavigator::odom_callback, this, std::placeholders::_1));

        // Action server
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<NavigationTask>(
            this, 
            "simple_navigator", 
            std::bind(&SimpleGlobalNavigator::handle_goal, this, _1, _2),
            std::bind(&SimpleGlobalNavigator::handle_cancel, this, _1),
            std::bind(&SimpleGlobalNavigator::handle_accepted, this, _1));
    }

    ~SimpleGlobalNavigator() {
        // Ensure action server is properly shut down
        if (action_server_) {
            action_server_.reset();
        }
    }

   private:
    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_tree_marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cost_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp_action::Server<NavigationTask>::SharedPtr action_server_;

    // State variables
    CostMapData cost_map_data_;
    nav_msgs::msg::Odometry current_odom_;
    std::vector<geometry_msgs::msg::PoseStamped> current_goal_poses_;
    size_t current_goal_index_;
    std::mutex state_mutex_;

    // RRT* parameters
    int rrt_max_iterations_;
    double rrt_step_size_;
    double rrt_goal_tolerance_;
    double rrt_rewire_radius_;
    bool enable_debug_visualization_;
    std::mt19937 rng_;

    // Callback functions
    void cost_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Action server handlers
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const NavigationTask::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigationTask> action_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNavigationTask> action_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigationTask>>& action_handle);

    // RRT* algorithm functions
    std::vector<geometry_msgs::msg::PoseStamped> plan_rrt_star_path(
        const geometry_msgs::msg::Point& start, 
        const geometry_msgs::msg::Point& goal,
        double max_planning_time = -1.0);
    
    std::shared_ptr<RRTNode> get_nearest_node(
        const std::vector<std::shared_ptr<RRTNode>>& nodes, 
        const geometry_msgs::msg::Point& point);
    
    geometry_msgs::msg::Point steer(
        const geometry_msgs::msg::Point& from, 
        const geometry_msgs::msg::Point& to, 
        double step_size);
    
    bool is_collision_free(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to);
    double get_cost_at_point(const geometry_msgs::msg::Point& point);
    double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    
    std::vector<std::shared_ptr<RRTNode>> get_near_nodes(
        const std::vector<std::shared_ptr<RRTNode>>& nodes, 
        const geometry_msgs::msg::Point& point, 
        double radius);
    
    void rewire(std::shared_ptr<RRTNode> new_node, 
                const std::vector<std::shared_ptr<RRTNode>>& near_nodes);
    
    std::vector<geometry_msgs::msg::PoseStamped> extract_path(std::shared_ptr<RRTNode> goal_node);
    nav_msgs::msg::Path create_path_message(const std::vector<geometry_msgs::msg::PoseStamped>& poses);

    // Utility functions
    geometry_msgs::msg::Point get_random_point();
    size_t get_current_goal_index(const geometry_msgs::msg::Point& current_pos);
    double calculate_distance_remaining(const geometry_msgs::msg::Point& current_pos);
    
    // Visualization functions
    void publish_rrt_tree_markers(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                                  const geometry_msgs::msg::Point& start,
                                  const geometry_msgs::msg::Point& goal);
    void clear_rrt_tree_markers();
};