#pragma once

#include <yaml-cpp/yaml.h>

#include <airstack_common/ros2_helper.hpp>
#include <airstack_common/tflib.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/msg/waypoint_xyzv_yaw.hpp>
#include <algorithm>
#include <cctype>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Trajectory;

class Waypoint {
   private:
    double x_, y_, z_, yaw_, vx_, vy_, vz_, ax_, ay_, az_, jx_, jy_, jz_, time_;

   public:
    Waypoint(double x, double y, double z, double yaw, double vx, double vy, double vz, double ax,
             double ay, double az, double jx, double jy, double jz, double time = 0);

    double get_x() const { return x_; }
    double get_y() const { return y_; }
    double get_z() const { return z_; }
    double get_vx() const { return vx_; }
    double get_vy() const { return vy_; }
    double get_vz() const { return vz_; }
    double get_ax() const { return ax_; }
    double get_ay() const { return ay_; }
    double get_az() const { return az_; }
    double get_jx() const { return jx_; }
    double get_jy() const { return jy_; }
    double get_jz() const { return jz_; }
    double get_yaw() const { return yaw_; }
    double get_time() const { return time_; }

    void set_time(double time) { time_ = time; }

    tf2::Quaternion quaternion() const;
    tf2::Vector3 position() const;
    tf2::Vector3 velocity() const;
    tf2::Vector3 acceleration() const;
    tf2::Vector3 jerk() const;

    /**
     * @brief Convert Waypoint to Odometry message
     */
    airstack_msgs::msg::Odometry as_odometry_msg(rclcpp::Time stamp, std::string frame_id) const;

    Waypoint interpolate(Waypoint wp, double t);

    friend class Trajectory;

    friend std::ostream& operator<<(std::ostream& os, const Waypoint& wp);
};

// Create one TransformListener instance to be used by every Trajectory instance
// TODO: change to smart pointer
static tf2_ros::Buffer* buffer = NULL;
static tf2_ros::TransformListener* listener = NULL;

class Trajectory {
   private:
    std::string frame_id;
    rclcpp::Time stamp;
    std::vector<Waypoint> waypoints;

    bool generated_waypoint_times;
    void generate_waypoint_times();

    // void init_listener();

    std::string marker_namespace;

   public:
    Trajectory();
    Trajectory(rclcpp::Node* node_ptr, std::string frame_id);
    Trajectory(rclcpp::Node* node_ptr, airstack_msgs::msg::TrajectoryXYZVYaw path);
    Trajectory(rclcpp::Node* node_ptr, nav_msgs::msg::Path path);
    // Trajectory(airstack_msgs::msg::TrajectoryXYZVYaw path);

    void clear();
    /**
     * @brief Get the closest Waypoint of this trajectory to a given target point
     *
     * @param point the target point
     * @return std::tuple<bool, Waypoint, size_t, double> whether any are valid, the closest
     * waypoint, the index of the closest waypoint, the distance along the path
     */
    std::tuple<bool, Waypoint, size_t, double> get_closest_point(tf2::Vector3 point);
    bool get_trajectory_distance_at_closest_point(tf2::Vector3 point, double* trajectory_distance);
    bool merge(Trajectory traj, double min_time = 0.0);

    bool get_closest_waypoint(tf2::Vector3 point, double initial_time, double end_time,
                              Waypoint* waypoint);
    bool get_waypoint_distance_ahead(double initial_time, double distance, Waypoint* waypoint);
    bool get_waypoint_sphere_intersection(double initial_time, double ahead_distance,
                                          double time_end, tf2::Vector3 sphere_center,
                                          double sphere_radius, double min_velocity,
                                          Waypoint* waypoint, Waypoint* end_waypoint);
    bool get_waypoint(double time, Waypoint* waypoint);

    const std::vector<Waypoint>& get_waypoints() const;

    double get_duration();
    bool get_odom(double time, airstack_msgs::msg::Odometry* odom, rclcpp::Time stamp);
    Trajectory to_frame(std::string target_frame, rclcpp::Time time);
    // Trajectory respace(double spacing);
    // Trajectory shorten(double new_length); // replace shorten with get_subtraj_dist
    Trajectory trim_trajectory_between_distances(double start_dist, double end_dist);
    Trajectory get_reversed_trajectory();
    float get_skip_ahead_time(float start_time, float max_velocity, float max_distance);

    void set_fixed_height(double height);

    size_t get_num_waypoints() const;
    const Waypoint& get_waypoint(int index) const;
    const std::string& get_frame_id() const;
    const rclcpp::Time& get_stamp() const { return stamp; }
    airstack_msgs::msg::TrajectoryXYZVYaw get_TrajectoryXYZVYaw_msg();
    std::vector<geometry_msgs::msg::PointStamped> get_vector_PointStamped_msg();

    visualization_msgs::msg::MarkerArray get_markers(rclcpp::Time stamp,
                                                     const std::string& marker_namespace,
                                                     float r = 1, float g = 0, float b = 0,
                                                     float a = 1, bool show_poses = false,
                                                     bool show_velocity = false,
                                                     float thickness = 0.03f);
};

//===================================================================================
//--------------------------------- Dynamic Trajectories ----------------------------
//===================================================================================

class DynamicTrajectory {
   private:
   public:
    virtual airstack_msgs::msg::TrajectoryXYZVYaw get_trajectory(
        airstack_msgs::msg::Odometry odom) = 0;
};

class AccelerationTrajectory : public DynamicTrajectory {
   private:
    // subscribers
    // tf2_ros::Buffer* buffer;
    // ros::Subscriber set_max_velocity_sub;

    double ax, ay, az;
    double dt, ht;
    double max_velocity;
    std::string frame;

    // callbacks
    // void set_max_velocity_callback(std_msgs::Float32 msg);

   public:
    AccelerationTrajectory(tf2_ros::Buffer* buffer, std::string frame, double ax, double ay,
                           double az, double dt, double ht, double max_velocity);
    virtual airstack_msgs::msg::TrajectoryXYZVYaw get_trajectory(airstack_msgs::msg::Odometry odom);
};

class TakeoffTrajectory : public DynamicTrajectory {
   private:
    double height, velocity;
    double path_roll, path_pitch;
    bool relative_to_orientation;

   public:
    TakeoffTrajectory(double height, double velocity, double path_roll = 0., double path_pitch = 0.,
                      bool relative_to_orientation = false);
    airstack_msgs::msg::TrajectoryXYZVYaw get_trajectory(airstack_msgs::msg::Odometry odom);
};

//===================================================================================
//--------------------------------- Static Trajectories -----------------------------
//===================================================================================

class StaticTrajectory {
   private:
   public:
    virtual airstack_msgs::msg::TrajectoryXYZVYaw get_trajectory() = 0;
};

class CurveTrajectory : public StaticTrajectory {
   private:
    airstack_msgs::msg::TrajectoryXYZVYaw trajectory;

    float linear_velocity, angular_velocity, dt, time, distance, yaw;
    std::string frame;
    bool use_heading;

    void generate_trajectory();

   public:
    CurveTrajectory(float linear_velocity, float angular_velocity, std::string frame, float time,
                    float dt, bool use_heading, float yaw = 0.f);

    virtual airstack_msgs::msg::TrajectoryXYZVYaw get_trajectory();
};

//===================================================================================
//--------------------------------- Trajectory Library ------------------------------
//===================================================================================

class TrajectoryLibrary {
   private:
    std::vector<StaticTrajectory*> static_trajectories;
    std::vector<DynamicTrajectory*> dynamic_trajectories;
    rclcpp::Node::SharedPtr node_ptr;

   public:
    TrajectoryLibrary(std::string config_filename,
                      rclcpp::Node::SharedPtr node_ptr);  // tf2_ros::Buffer*

    std::vector<Trajectory> get_static_trajectories();
    std::vector<Trajectory> get_dynamic_trajectories(airstack_msgs::msg::Odometry odom);
    visualization_msgs::msg::MarkerArray get_markers(
        std::vector<airstack_msgs::msg::TrajectoryXYZVYaw> trajectories);

   private:
    std::string trim(std::string str) {
        int trim_start = 0;
        int trim_end = 0;
        for (size_t i = 0; i < str.size(); i++)
            if (std::isspace(str[i]))
                trim_start++;
            else
                break;

        for (int i = str.size() - 1; i >= 0; i--)
            if (std::isspace(str[i]))
                trim_end++;
            else
                break;

        std::string trimmed = str.substr(trim_start, str.size() - trim_end - trim_start);
        return trimmed;
    }

    template <typename T>
    std::string get_string(T t) {
        return std::to_string(t);
    }

    std::string get_string(std::string t) { return t; }

    template <typename T>
    T parse(YAML::Node yamlnode) {
        // std::cout << "trim test |" << trim("  asdjfaksfdj   jdfkj jakdsf  ") << "|" << std::endl;

        std::string str = yamlnode.as<std::string>();
        std::size_t start;

        std::string param_label = "$(param";

        // std::cout << "string: " << str << std::endl;
        while ((start = str.find(param_label)) != std::string::npos) {
            std::size_t end = str.find(")");
            // std::cout << start << " " << end << std::endl;
            if (end == std::string::npos) {
                RCLCPP_DEBUG_STREAM(
                    node_ptr->get_logger(),
                    "TRAJECTORY LIBRARY PARSING ERROR: No closing parenthesis for $(param");
                break;
            }

            std::string parameter_name =
                trim(str.substr(start + param_label.size(), end - (start + param_label.size())));
            RCLCPP_DEBUG(node_ptr->get_logger(), "parameter_name: %s", parameter_name.c_str());
            T parameter_value;
            if (!this->node_ptr->get_parameter(parameter_name, parameter_value)) {
                RCLCPP_INFO(node_ptr->get_logger(),
                            "Couldn't find parameter '%s'. This is either because it doesn't exist "
                            "or the type is incorrect in the launch file.",
                            parameter_name.c_str());
            }
            RCLCPP_DEBUG_STREAM(node_ptr->get_logger(), "parameter_value: " << parameter_value);
            RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
                                str << " " << start << " " << end << std::endl);
            str = str.substr(0, start) + get_string(parameter_value) +
                  str.substr(end + 1, str.size() - (end + 1));
            RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
                                str << " " << str.find(param_label) << std::endl);
            RCLCPP_DEBUG(node_ptr->get_logger(), "str: %s", str.c_str());
        }

        // std::cout << "final: " << str << std::endl;
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "key";
        out << YAML::Value << str;
        out << YAML::EndMap;
        std::string yaml = out.c_str();
        YAML::Node n = YAML::Load(out.c_str());
        // std::cout << "yaml: " << yaml << " " << n["key"].as<T>() << std::endl;

        return n["key"].as<T>();
    }
};
