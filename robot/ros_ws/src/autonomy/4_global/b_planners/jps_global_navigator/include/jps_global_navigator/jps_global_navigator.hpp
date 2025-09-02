#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>

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

// Grid cell structure for JPS
struct GridCell {
    int x, y, z;
    
    GridCell(int x_ = 0, int y_ = 0, int z_ = 0) : x(x_), y(y_), z(z_) {}
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    bool operator<(const GridCell& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

// Hash function for GridCell
struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return std::hash<int>()(cell.x) ^ 
               (std::hash<int>()(cell.y) << 1) ^ 
               (std::hash<int>()(cell.z) << 2);
    }
};

// JPS node structure
struct JPSNode {
    GridCell cell;
    std::shared_ptr<JPSNode> parent;
    double g_cost;  // Cost from start
    double h_cost;  // Heuristic cost to goal
    double f_cost;  // Total cost (g + h)
    
    JPSNode(const GridCell& c, std::shared_ptr<JPSNode> p = nullptr, 
            double g = 0.0, double h = 0.0) 
        : cell(c), parent(p), g_cost(g), h_cost(h), f_cost(g + h) {}
};

// Comparison for priority queue (min-heap based on f_cost)
struct JPSNodeComparator {
    bool operator()(const std::shared_ptr<JPSNode>& a, const std::shared_ptr<JPSNode>& b) const {
        return a->f_cost > b->f_cost;
    }
};

// Direction vectors for 3D movement (26 directions)
struct Direction {
    int dx, dy, dz;
    Direction(int x = 0, int y = 0, int z = 0) : dx(x), dy(y), dz(z) {}
    
    bool operator==(const Direction& other) const {
        return dx == other.dx && dy == other.dy && dz == other.dz;
    }
};

// Cost map data structure
struct CostMapData {
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<float> costs;
    double resolution;
    geometry_msgs::msg::Point min_bounds;
    geometry_msgs::msg::Point max_bounds;
    bool valid;
    
    // Grid representation for fast lookup
    std::unordered_map<GridCell, float, GridCellHash> grid_costs;
    int grid_size_x, grid_size_y, grid_size_z;
    
    CostMapData() : resolution(0.5), valid(false), 
                    grid_size_x(0), grid_size_y(0), grid_size_z(0) {}
};

class JPSGlobalNavigator : public rclcpp::Node {
   public:
    using NavigationTask = task_msgs::action::NavigationTask;
    using GoalHandleNavigationTask = rclcpp_action::ServerGoalHandle<NavigationTask>;

    JPSGlobalNavigator(const rclcpp::NodeOptions& options)
        : Node("jps_global_navigator", options), 
          current_goal_index_(0),
          odom_received_(false) {
        
        RCLCPP_INFO(this->get_logger(), "JPS Global Navigator initialized.");

        // Declare parameters
        this->declare_parameter("jps_max_iterations", 10000);
        this->declare_parameter("jps_goal_tolerance", 2.0);
        this->declare_parameter("cost_threshold", 50.0);
        this->declare_parameter("smoothing_iterations", 5);
        this->declare_parameter("smoothing_step_size", 0.1);
        this->declare_parameter("cost_map_topic", "/cost_map");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("enable_debug_visualization", true);
        this->declare_parameter("resolution", 0.5);

        // Get parameters
        jps_max_iterations_ = this->get_parameter("jps_max_iterations").as_int();
        jps_goal_tolerance_ = this->get_parameter("jps_goal_tolerance").as_double();
        cost_threshold_ = this->get_parameter("cost_threshold").as_double();
        smoothing_iterations_ = this->get_parameter("smoothing_iterations").as_int();
        smoothing_step_size_ = this->get_parameter("smoothing_step_size").as_double();
        enable_debug_visualization_ = this->get_parameter("enable_debug_visualization").as_bool();
        cost_map_data_.resolution = this->get_parameter("resolution").as_double();

        // Publishers
        global_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "global_plan", rclcpp::QoS(10).transient_local());
        
        if (enable_debug_visualization_) {
            jps_search_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "jps_search_markers", rclcpp::QoS(10));
            RCLCPP_INFO(this->get_logger(), "JPS search debug visualization enabled");
        }

        // Subscribers
        cost_map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cost_map_topic").as_string(), 
            rclcpp::QoS(10),
            std::bind(&JPSGlobalNavigator::cost_map_callback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(),
            rclcpp::QoS(10),
            std::bind(&JPSGlobalNavigator::odom_callback, this, std::placeholders::_1));

        // Action server
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<NavigationTask>(
            this, 
            "jps_navigator", 
            std::bind(&JPSGlobalNavigator::handle_goal, this, _1, _2),
            std::bind(&JPSGlobalNavigator::handle_cancel, this, _1),
            std::bind(&JPSGlobalNavigator::handle_accepted, this, _1));
            
        // Initialize direction vectors for 3D JPS (26 directions)
        initialize_directions();
    }

    ~JPSGlobalNavigator() {
        if (action_server_) {
            action_server_.reset();
        }
    }

   private:
    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr jps_search_marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cost_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp_action::Server<NavigationTask>::SharedPtr action_server_;

    // State variables
    CostMapData cost_map_data_;
    nav_msgs::msg::Odometry current_odom_;
    std::vector<geometry_msgs::msg::PoseStamped> current_goal_poses_;
    size_t current_goal_index_;
    bool odom_received_;
    std::mutex state_mutex_;

    // JPS parameters
    int jps_max_iterations_;
    double jps_goal_tolerance_;
    double cost_threshold_;
    int smoothing_iterations_;
    double smoothing_step_size_;
    bool enable_debug_visualization_;
    
    // Direction vectors for 3D movement
    std::vector<Direction> directions_;

    // Callback functions
    void cost_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Action server handlers
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const NavigationTask::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigationTask> action_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNavigationTask> action_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigationTask>>& action_handle);

    // JPS algorithm functions
    std::vector<geometry_msgs::msg::PoseStamped> plan_jps_path(
        const geometry_msgs::msg::Point& start, 
        const geometry_msgs::msg::Point& goal,
        double max_planning_time = -1.0);
    
    std::vector<GridCell> jps_search(const GridCell& start, const GridCell& goal, double max_planning_time);
    std::shared_ptr<JPSNode> jump(const GridCell& current, const Direction& direction, const GridCell& goal);
    std::vector<Direction> get_natural_neighbors(const GridCell& current, const Direction& parent_direction);
    std::vector<Direction> get_forced_neighbors(const GridCell& current, const Direction& direction);
    bool has_forced_neighbor(const GridCell& current, const Direction& direction);
    
    // Path smoothing functions
    std::vector<geometry_msgs::msg::PoseStamped> smooth_path(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path);
    bool is_line_collision_free(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to);
    
    // Grid and cost functions
    GridCell point_to_grid(const geometry_msgs::msg::Point& point);
    geometry_msgs::msg::Point grid_to_point(const GridCell& cell);
    bool is_valid_cell(const GridCell& cell);
    bool is_obstacle(const GridCell& cell);
    double get_cost_at_cell(const GridCell& cell);
    double heuristic(const GridCell& a, const GridCell& b);
    double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    
    // Utility functions
    void initialize_directions();
    void build_grid_from_cost_map();
    std::vector<geometry_msgs::msg::PoseStamped> extract_path(std::shared_ptr<JPSNode> goal_node);
    nav_msgs::msg::Path create_path_message(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
    size_t get_current_goal_index(const geometry_msgs::msg::Point& current_pos);
    double calculate_distance_remaining(const geometry_msgs::msg::Point& current_pos);
    
    // Visualization functions
    void publish_jps_search_markers(const std::vector<std::shared_ptr<JPSNode>>& explored_nodes,
                                    const GridCell& start, const GridCell& goal,
                                    const std::vector<GridCell>& path = {});
    void clear_jps_search_markers();
};