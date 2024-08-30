#pragma once

#include <core_map_representation_interface/map_representation.h>

#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <disparity_graph/disparity_graph.hpp>
#include <disparity_map_representation/disparity_map_representation.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <trajectory_library/trajectory_library.hpp>
// #include <pointcloud_map_representation/pointcloud_map_representation.hpp>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class LocalPlanner : rclcpp::Node {
   private:
    std::unique_ptr<TrajectoryLibrary> traj_lib;

    std::string map_representation;
    bool got_global_plan;
    airstack_msgs::msg::TrajectoryXYZVYaw global_plan;
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

    rclcpp::Subscription<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr global_plan_sub;
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

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // publishers
    // ros::Publisher vis_pub, traj_pub, traj_track_pub, obst_vis_pub, global_plan_vis_pub,
    //    look_past_vis_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr vis_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_track_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr obst_vis_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr global_plan_vis_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr look_past_vis_pub;

    // services
    // ros::ServiceClient traj_mode_client;
    rclcpp::Client<airstack_msgs::msg::TrajectoryMode>::SharedPtr traj_mode_client;

   public:
    LocalPlanner(const std::string node_name)
        : Node("droan_local_planner"),
          tf_listener(tf_buffer),
          goal_mode(TRAJECTORY)

    {
        // subscribers
        global_plan_sub = this->create_subscription<airstack_msgs::msg::TrajectoryXYZVYaw>(
            "global_plan", 10, std::bind(&LocalPlanner::global_plan_callback, this, _1));
        waypoint_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "way_point", 10, std::bind(&LocalPlanner::waypoint_callback, this, _1));
        look_ahead_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "look_ahead", 10, std::bind(&LocalPlanner::look_ahead_callback, this, _1));
        tracking_point_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "tracking_point", 10, std::bind(&LocalPlanner::tracking_point_callback, this, _1));
        range_up_sub = this->create_subscription<sensor_msgs::msg::Range>(
            "range_up", 1, std::bind(&LocalPlanner::range_up_callback, this, _1));
        range_down_sub = this->create_subscription<sensor_msgs::msg::Range>(
            "range_down", 1, std::bind(&LocalPlanner::range_down_callback, this, _1));
        custom_waypoint_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "custom_waypoint", 1, std::bind(&LocalPlanner::custom_waypoint_callback, this, _1));

        // publishers

        vis_pub =
            this->create_publisher<geometry_msgs::msg::PointStamped>("trajectory_library_vis", 10);
        obst_vis_pub = this->create_publisher<sensor_msgs::msg::Range>("obstaccle_vis", 10);
        global_plan_vis_pub = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(
            "local_planner_global_plan_vis", 10);
        look_past_vis_pub =
            this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("look_past", 10);
        traj_pub = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory", 10);
        traj_track_pub =
            this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_track", 10);

        // services
        traj_mode_client =
            this->create_client<trajectory_controller::msg::TrajectoryMode>("set_trajectory_mode");

        // init parameters
        this->declare_parameter("waypoint_spacing", 0.5);
        waypoint_spacing = this->get_parameter("waypoint_spacing").as_double();
        this->declare_parameter("obstacle_penalty_weight", 1.);
        obstacle_penalty_weight = this->get_parameter("obstacle_penalty_weight").as_double();
        this->declare_parameter("forward_progress_penalty_weight", 0.5);
        forward_progress_penalty_weight =
            this->get_parameter("forward_progress_penalty_weight").as_double();
        this->declare_parameter("robot_radius", 0.75);
        robot_radius = this->get_parameter("robot_radius").as_double();
        this->declare_parameter("look_past_distance", 0);
        look_past_distance = this->get_parameter("look_past_distance").as_double();
        this->declare_parameter("height_mode", 0);
        height_mode = this->get_parameter("height_mode").as_int();
        this->declare_parameter("height_above_ground", 1.);
        height_above_ground = this->get_parameter("height_above_ground").as_double();
        this->declare_parameter("fixed_height", 1.);
        fixed_height = this->get_parameter("fixed_height").as_double();
        this->declare_parameter("yaw_mode", 0);
        yaw_mode = this->get_parameter("yaw_mode").as_int();
        this->declare_parameter("map_representation", std::string("PointCloudMapRepresentation"));
        map_representation = this->get_parameter("map_representation").as_string();
        this->declare_parameter("waypoint_buffer_duration", 30.);
        waypoint_buffer_duration = this->get_parameter("waypoint_buffer_duration").as_double();
        this->declare_parameter("waypoint_spacing_threshold", 0.5);
        waypoint_spacing_threshold = this->get_parameter("waypoint_spacing_threshold").as_double();
        this->declare_parameter("waypoint_angle_threshold", 30. * M_PI / 180.);
        waypoint_angle_threshold = this->get_parameter("waypoint_angle_threshold").as_double();

        this->declare_parameter("custom_waypoint_timeout_factor", 0.3);
        custom_waypoint_timeout_factor =
            this->get_parameter("custom_waypoint_timeout_factor").as_double();
        this->declare_parameter("custom_waypoint_distance_threshold", 0.5);
        custom_waypoint_distance_threshold =
            this->get_parameter("custom_waypoint_distance_threshold").as_double();

        this->declare_parameter("trajectory_library_config", std::string(""));
        traj_lib = std::make_unique<TrajectoryLibrary>(
            this->get_parameter("trajectory_library_config").as_string(), tf_buffer);
    }
    virtual ~LocalPlanner();

    virtual bool execute() {
        update_waypoint_mode();

        if (!got_global_plan) return true;

        Trajectory gp(global_plan);

        // set the height of the global plan
        if (height_mode == FIXED_HEIGHT) {
            gp.set_fixed_height(fixed_height);
        } else if (height_mode == RANGE_SENSOR_HEIGHT) {
            if (!got_range_up || !got_range_down) return true;

            try {
                tf2::Stamped<tf2::Transform> transform_up, transform_down;

                geometry_msgs::msg::TransformStamped tf_up_msg, tf_down_msg;
                tf_up_msg = tf_buffer.lookupTransform(range_up.header.frame_id, range_up.header_id,
                                                      range_up.header.stamp);
                tf2::fromMsg(tf_up_msg, transform_up);
                tf_down_msg = tf_buffer.lookupTransform(
                    range_down.header.frame_id, range_down.header_id, range_down.header.stamp);
                tf2::fromMsg(tf_down_msg, transform_down);

                tf2::Vector3 range_up_gp_frame = transform_up * tf2::Vector3(range_up.range, 0, 0);
                tf2::Vector3 range_down_gp_frame =
                    transform_down * tf2::Vector3(range_down.range, 0, 0);

                double tunnel_height = range_up_gp_frame.z() - range_down_gp_frame.z();
                double z_setpoint = (range_up_gp_frame.z() + range_down_gp_frame.z()) / 2.0;
                if (tunnel_height / 2.0 >= height_above_ground) {
                    z_setpoint = range_down_gp_frame.z() + height_above_ground;
                }

                gp.set_fixed_height(z_setpoint);

            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR("LocalPlanner", "Failed to get transform: %s", ex.what());
                return true;
            }
        }

        // transform the look ahead point to the global plan frame
        nav_msgs::msg::Odometry look_ahead_odom_global;
        // translate to global frame
        tf_buffer.transform(look_ahead_odom, look_ahead_odom_global, gp.get_frame_id());

        tf2::Vector3 look_ahead_position_global;  // global frame
        tf2::fromMsg(look_ahead_odom.pose.pose.position, look_ahead_position_global);

        // increment how far along the global plan we are
        double trajectory_distance;
        bool valid = gp.get_trajectory_distance_at_closest_point(look_ahead_position_global,
                                                                 &trajectory_distance);
        if (valid) {
            global_plan_trajectory_distance += trajectory_distance;
            gp = gp.get_subtrajectory_distance(global_plan_trajectory_distance,
                                               global_plan_trajectory_distance + 10.0);
        } else {
            RCLCPP_INFO(this->get_logger(), "invalid");
        }

        // publish the segment of the global plan currently being used, for visualization
        visualization_msgs::msg::MarkerArray global_markers = gp.get_markers(0, 0, 1);
        global_plan_vis_pub->publish(global_markers);

        // get the dynamic trajectories
        std::vector<Trajectory> dynamic_trajectories =
            traj_lib->get_dynamic_trajectories(look_ahead_odom);

        // pick the best trajectory
        Trajectory best_traj;
        bool all_in_collision = this->get_best_trajectory(dynamic_trajectories, gp, &best_traj);

        // publish the trajectory
        if (!all_in_collision) {
            airstack_msgs::msg::TrajectoryXYZVYaw path = best_traj.get_TrajectoryXYZVYaw();

            // set yaw
            if (yaw_mode == SMOOTH_YAW && path.waypoints.size() > 0) {
                bool found_initial_heading = false;
                double initial_heading = 0;
                try {
                    // get the initial heading of the look_ahead_odom in best_traj frame
                    best_traj.get_frame_id();

                    nav_msgs::msg::Odometry look_ahead_odom_best_traj_frame =
                        tf_buffer.transform(look_ahead_odom, best_traj.get_frame_id());

                    initial_heading =
                        tf2::getYaw(look_ahead_odom_best_traj_frame.pose.pose.orientation);
                    found_initial_heading = true;

                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR("LocalPlanner", "Failed to get transform: %s", ex.what());
                }

                if (found_initial_heading) {
                    path.waypoints[0].yaw = initial_heading;
                    double alpha = 0.1;
                    double sin_yaw_prev = sin(path.waypoints[0].yaw);
                    double cos_yaw_prev = cos(path.waypoints[0].yaw);

                    for (int i = 1; i < path.waypoints.size(); i++) {
                        airstack_msgs::msg::WaypointXYZVYaw wp_prev = path.waypoints[i - 1];
                        airstack_msgs::msg::WaypointXYZVYaw& wp_curr = path.waypoints[i];

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
            path.header.stamp = this->now();
            traj_pub->publish(path);
        }
        return true;
    }

    bool get_best_trajectory(std::vector<Trajectory> trajs, Trajectory global_plan,
                             Trajectory* best_traj_ret) {
        double min_cost = std::numeric_limits<double>::max();
        int best_traj_index = 0;
        bool all_in_collision = true;

        auto now = this->now();

        visualization_msgs::msg::MarkerArray traj_lib_markers, look_past_markers;

        std::vector<std::vector<geometry_msgs::msg::PointStamped>> trajectories;
        for (int i = 0; i < trajs.size(); i++) {
            trajectories.push_back(trajs[i].get_vector_PointStamped());
        }

        std::vector<std::vector<double>> values = pc_map->get_values(trajectories);

        for (int i = 0; i < trajectories.size(); ++i) {
            Trajectory traj = trajectories[i];
            double average_distance = std::numeric_limits<double>::infinity();
            double closest_obstacle_distance = std::numeric_limits<double>::infinity();

            Trajectory global_plan_in_traj_frame;
            try {
                global_plan_in_traj_frame = global_plan.transform_to_frame(traj.get_frame_id());
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR("LocalPlanner",
                             "Failed to transform global plan to trajectory frame: %s", ex.what());
                return true;
            }

            for (int j = 0; j < traj.waypoint_count(); j++) {
                Waypoint wp = traj.get_waypoint(j);

                nav_msgs::msg::Odometry odom = wp.odometry(now, traj.get_frame_id());
                geometry_msgs::msg::PoseStamped pose;
                pose.header = odom.header;
                pose.pose = odom.pose.pose;
                closest_obstacle_distance = std::min(closest_obstacle_distance, values[i][j]);

                tf2::Vector3 closest_point;
                int wp_index;
                double path_distance;
                bool valid = global_plan_in_traj_frame.get_closest_point(
                    wp.position(), &closest_point, &wp_index, &path_distance);
                double forward_progress_penalty = -forward_progress_penalty_weight * path_distance;

                if (valid) {
                    if (!std::isfinite(average_distance)) {
                        average_distance = 0;
                    }
                    average_distance +=
                        closest_point.distance(wp.position()) + forward_progress_penalty;
                }
            }

            average_distance /= traj.waypoint_count();
            bool collision = closest_obstacle_distance <= robot_radius;
            if (!collision) {
                all_in_collision = false;
            }

            visualization_msgs::msg::MarkerArray traj_markers;
            if (collision) {
                // red for collision
                traj_markers = traj.get_markers(1, 0, 0, .5);
            } else {
                traj_markers = traj.get_markers(0, 1, 0, .5);
            }

            if (look_past_distance > 0) {
                if (traj.waypoint_count() >= 2) {
                    Waypoint curr_wp = traj.get_waypoint(traj.waypoint_count() - 1);
                    Waypoint prev_wp = traj.get_waypoint(traj.waypoint_count() - 2);

                    tf2::Vector3 segment = curr_wp.position() - prev_wp.position();
                    segment.normalize();

                    tf2::Vector3 position = curr_wp.position() + look_past_distance * segment;

                    tf2::Vector3 closest_point;
                    int wp_index;
                    bool valid = global_plan_in_traj_frame.get_closest_point(
                        position, &closest_point, &wp_index);

                    if (!valid) {
                        collision = true;
                    } else {
                        average_distance = position.distance(closest_point);
                    }

                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = traj.get_frame_id();
                    marker.header.stamp = now;
                    marker.ns = "look_past";
                    marker.id = i;
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.action = visualization_msgs::msg::Marker::ADD;

                    marker.pose.position.x = position.x();
                    marker.pose.position.y = position.y();
                    marker.pose.position.z = position.z();
                    marker.pose.orientation.w = 1;
                    marker.scale.x = 0.3;
                    marker.scale.y = 0.3;
                    marker.scale.z = 0.3;
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.color.a = 1.0;
                    look_past_markers.markers.push_back(marker);
                }
            }
            traj_lib_markers.markers.insert(traj_lib_markers.markers.end(),
                                            traj_markers.markers.begin(),
                                            traj_markers.markers.end());
            double cost =
                average_distance - obstacle_penalty_weight *
                                       std::min(closest_obstacle_distance, obstacle_check_radius);
            if (!collision && cost < min_cost) {
                min_cost = cost;
                best_traj_index = i;
                *best_traj_ret = traj;
            }
        }
        if (best_traj_index < look_past_markers.markers.size()) {
            look_past_markers.markers[best_traj_index].color.r = 0;
            look_past_markers.markers[best_traj_index].color.g = 1;
            look_past_markers.markers[best_traj_index].color.b = 0;
            look_past_markers.markers[best_traj_index].color.a = 1;
        }

        vis_pub->publish(traj_lib_markers);
        pc_map->publish_debug();
        look_past_vis_pub->publish(look_past_markers);

        return all_in_collision;
    }

    // subscriber callbacks
    void global_plan_callback(const airstack_msgs::msg::TrajectoryXYZVYaw::SharedPtr global_plan) {
        RCLCPP_INFO_STREAM(this->get_logger(), "GOT GLOBAL PLAN, goal_mode: " << goal_mode);
        if (goal_mode != TRAJECTORY) return;

        this->global_plan = global_plan;
        got_global_plan = true;
        global_plan_trajectory_distance = 0;
    }

    void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr wp) {
        if (goal_mode != AUTO_WAYPOINT) {
            waypoint_buffer.clear();
            waypoint_buffer.push_back(wp);
            return;
        }

        // remove old waypoints if necessary
        waypoint_buffer.push_back(wp);
        if ((wp.header.stamp - waypoint_buffer.front().header.stamp).toSec() >
            waypoint_buffer_duration) {
            waypoint_buffer.pop_front();
        }

        // stitch together the history of waypoints
        airstack_msgs::msg::TrajectoryXYZVYaw global_plan;
        global_plan.header.frame_id = wp.header.frame_id;
        global_plan.header.stamp = wp.header.stamp;

        std::vector<airstack_msgs::msg::WaypointXYZVYaw> backwards_global_plan;

        std::vector<geometry_msgs::msg::PointStamped> prev_wps;
        for (auto it = waypoint_buffer.rbegin(); it != waypoint_buffer.rend(); it++) {
            geometry_msgs::msg::PointStamped curr_wp = *it;
            airstack_msgs::msg::WaypointXYZVYaw waypoint;
            waypoint.position.x = curr_wp.point.x;
            waypoint.position.y = curr_wp.point.y;
            waypoint.position.z = curr_wp.point.z;

            if (prev_wps.size() > 1) {
                geometry_msgs::msg::PointStamped prev_wp = prev_wps[prev_wps.size() - 1];
                geometry_msgs::msg::PointStamped prev2_wp = prev_wps[prev_wps.size() - 2];
                float distance = sqrt(pow(curr_wp.point.x - prev_wp.point.x, 2) +
                                      pow(curr_wp.point.y - prev_wp.point.y, 2));

                float a = pow(prev_wp.point.x - curr_wp.point.x, 2) +
                          pow(prev_wp.point.y - curr_wp.point.y, 2);
                float b = pow(prev_wp.point.x - prev2_wp.point.x, 2) +
                          pow(prev_wp.point.y - prev2_wp.point.y, 2);
                float c = pow(prev2_wp.point.x - curr_wp.point.x, 2) +
                          pow(prev2_wp.point.y - curr_wp.point.y, 2);
                float angle = acos((a + b - c) / sqrt(4 * a * b));

                float angle_diff = fabs(atan2(sin(angle - M_PI), cos(angle - M_PI)));

                // ROS_INFO_STREAM("\tdistance: " << distance << " angle: " << angle*180./M_PI << "
                // angle_diff: " << angle_diff*180./M_PI);

                if (distance >= waypoint_spacing_threshold &&
                    angle_diff < waypoint_angle_threshold) {
                    // ROS_INFO_STREAM("\tADDING wp: " << curr_wp.point.x << " " << curr_wp.point.y
                    // << " " << curr_wp.point.z);
                    backwards_global_plan.push_back(waypoint);
                    prev_wps.push_back(curr_wp);
                }
            } else {
                if (prev_wps.size() == 0) {
                    prev_wps.push_back(curr_wp);
                    backwards_global_plan.push_back(waypoint);
                } else if (prev_wps.size() == 1) {
                    geometry_msgs::msg::PointStamped prev_wp = prev_wps[prev_wps.size() - 1];
                    float distance = sqrt(pow(curr_wp.point.x - prev_wp.point.x, 2) +
                                          pow(curr_wp.point.y - prev_wp.point.y, 2));
                    if (distance >= waypoint_spacing_threshold) {
                        prev_wps.push_back(curr_wp);
                        backwards_global_plan.push_back(waypoint);
                    }
                }
            }
        }
        for (int i = 0; i < backwards_global_plan.size(); i++) {
            global_plan.waypoints.push_back(
                backwards_global_plan[backwards_global_plan.size() - 1 - i]);
        }
        if (got_tracking_point) {
            try {
                tf2::Vector3 tp_pos = tf2::Vector3(tracking_point_odom.pose.pose.position.x,
                                                   tracking_point_odom.pose.pose.position.y,
                                                   tracking_point_odom.pose.pose.position.z);

                // translate wp to tracking point frame
                geometry_msgs::msg::PointStamped wp_msg_in_tp_frame =
                    tf_buffer.transform(wp, tracking_point_odom.header.frame_id);
                tf2::Vector3 wp_pos_in_tp_frame =
                    tf2::Vector3(wp_msg_in_tp_frame.point.x, wp_msg_in_tp_frame.point.y,
                                 wp_msg_in_tp_frame.point.z);

                tf2::Vector3 direction = (wp_pos_in_tp_frame - tp_pos).normalized() * 3;
                tf2::Vector3 wp2_pos = wp_msg_in_tp_frame + direction;

                airstack_msgs::msg::WaypointXYZVYaw wp1, wp2;
                wp1.position.x = wp_pos_in_tp_frame.x();
                wp1.position.y = wp_pos_in_tp_frame.y();
                wp1.position.z = wp_pos_in_tp_frame.z();
                wp2.position.x = wp2_pos.x();
                wp2.position.y = wp2_pos.y();
                wp2.position.z = wp2_pos.z();
                global_plan.waypoints.push_back(wp1);
                global_plan.waypoints.push_back(wp2);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR("LocalPlanner", "Failed to get transform: %s", ex.what());
            }
        }

        global_plan_trajectory_distance = 0;
        this->global_plan = global_plan;
        this->got_global_plan = true;
    }

    void custom_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr wp) {
        if (!got_look_ahead) return;

        try {
            auto wp_msg_in_la_frame =
                tf_buffer.transform(wp.pose.position, look_ahead_odom.header.frame_id);

            tf2::Vector3 la_position = tf2::Vector3(look_ahead_odom.pose.pose.position.x,
                                                    look_ahead_odom.pose.pose.position.y,
                                                    look_ahead_odom.pose.pose.position.z);
            tf2::Vector3 wp_position =
                tf2::Vector3(wp_msg_in_la_frame.x, wp_msg_in_la_frame.y, wp_msg_in_la_frame.z);

            airstack_msgs::msg::TrajectoryXYZVYaw global_plan;
            global_plan.header.frame_id = look_ahead_odom.header.frame_id;
            global_plan.header.stamp = this->now();

            airstack_msgs::msg::WaypointXYZVYaw wp1, wp2;
            wp1.position.x = la_position.x();
            wp1.position.y = la_position.y();
            wp1.position.z = la_position.z();
            wp2.position.x = wp_position.x();
            wp2.position.y = wp_position.y();
            wp2.position.z = wp_position.z();
            global_plan.waypoints.push_back(wp1);
            global_plan.waypoints.push_back(wp2);

            global_plan_trajectory_distance = 0;
            this->global_plan = global_plan;
            this->got_global_plan = true;

            goal_mode = CUSTOM_WAYPOINT;
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR("LocalPlanner", "Failed to get transform: %s", ex.what());
        }
    }
    void update_waypoint_mode() {
        if (goal_mode == CUSTOM_WAYPOINT) {
            if (global_plan.waypoints.size() < 2) goal_mode = AUTO_WAYPOINT;

            // check if the time limit for reaching the waypoint has elapsed
            double elapsed_time = (this->now() - global_plan.header.stamp).toSec();
            double distance = 0;
            for (int i = 1; i < global_plan.waypoints.size(); i++) {
                airstack_msgs::msg::WaypointXYZVYaw prev_wp, curr_wp;
                prev_wp = global_plan.waypoints[i - 1];
                curr_wp = global_plan.waypoints[i];

                distance += sqrt(pow(prev_wp.position.x - curr_wp.position.x, 2) +
                                 pow(prev_wp.position.y - curr_wp.position.y, 2));
            }

            // ROS_INFO_STREAM("elapsed: " << elapsed_time << " / " <<
            // distance/custom_waypoint_timeout_factor << " distance: " << distance);
            if (elapsed_time >= distance / custom_waypoint_timeout_factor) {
                // ROS_INFO_STREAM("CUSTOM WAYPOINT TIMEOUT REACHED");
                goal_mode = AUTO_WAYPOINT;
            }

            // check if we are close enough to the waypoint
            if (got_tracking_point) {
                try {
                    tf2::Vector3 tp_pos = tf2::Vector3(tracking_point_odom.pose.pose.position.x,
                                                       tracking_point_odom.pose.pose.position.y,
                                                       tracking_point_odom.pose.pose.position.z);

                    // translate wp to tracking point frame
                    geometry_msgs::msg::PointStamped wp_msg_in_tp_frame = tf_buffer.transform(
                        global_plan.waypoints.back().position, tracking_point_odom.header.frame_id);
                    tf2::Vector3 wp_pos_in_tp_frame =
                        tf2::Vector3(wp_msg_in_tp_frame.point.x, wp_msg_in_tp_frame.point.y,
                                     wp_msg_in_tp_frame.point.z);

                    if (tp_position.distance(wp_pos_in_tp_frame) <
                        custom_waypoint_distance_threshold) {
                        goal_mode = AUTO_WAYPOINT;
                    }
                } catch (tf2::TransformException& ex) {
                    ROS_ERROR_STREAM("TransformException in update_waypoint_mode: " << ex.what());
                }
            }
        }
    }
    void look_ahead_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        got_look_ahead = true;
        look_ahead_odom = odom;
    }
    void tracking_point_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        got_tracking_point = true;
        tracking_point_odom = odom;
    }
    void range_up_callback(const sensor_msgs::msg::Range::SharedPtr range) {
        got_range_up = true;
        range_up = range;
    }
    void range_down_callback(const sensor_msgs::msg::Range::SharedPtr range) {
        got_range_down = true;
        range_down = range;
    }
};
