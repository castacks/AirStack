#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cost_map_interface/cost_map_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <list>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <trajectory_library/trajectory_library.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

class DroanLocalPlanner : public rclcpp::Node {
   private:
    std::unique_ptr<TrajectoryLibrary> traj_lib;

    std::string map_representation_class_string;
    bool is_global_plan_received;
    nav_msgs::msg::Path global_plan_msg;
    double global_plan_trajectory_distance;
    bool is_look_ahead_received, is_tracking_point_received;

    // look ahead is
    airstack_msgs::msg::Odometry look_ahead_odom, tracking_point_odom;

    double execute_rate, obstacle_check_radius, safety_cost_weight,
        forward_progress_forgiveness_weight;

    float auto_waypoint_buffer_duration, auto_waypoint_spacing_threshold,
        auto_waypoint_angle_threshold;
    std::list<geometry_msgs::msg::PointStamped> waypoint_buffer;

    enum YawMode { TRAJECTORY_YAW, SMOOTH_YAW };
    YawMode yaw_mode;

    // whether to follow the global plan, a custom waypoint, or automatically interpolated waypoints
    enum GoalMode { GLOBAL_PLAN, CUSTOM_WAYPOINT, AUTO_WAYPOINT };
    GoalMode goal_mode;
    double custom_waypoint_timeout_factor, custom_waypoint_distance_threshold;

    std::shared_ptr<cost_map_interface::CostMapInterface> cost_map;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
    tf2_ros::TransformListener tf_listener;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
    rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
    rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr custom_waypoint_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_lib_vis_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_track_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr obst_vis_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_plan_vis_pub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_debug_pub;

    rclcpp::TimerBase::SharedPtr execute_timer;

   public:
    DroanLocalPlanner()
        : Node("droan_local_planner"),
          goal_mode(GLOBAL_PLAN),
          tf_buffer_ptr(new tf2_ros::Buffer(this->get_clock())),
          tf_listener(*tf_buffer_ptr) {
        // follow the global plan
        global_plan_sub = this->create_subscription<nav_msgs::msg::Path>(
            "global_plan", 10, std::bind(&DroanLocalPlanner::global_plan_callback, this, _1));
        waypoint_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "way_point", 10, std::bind(&DroanLocalPlanner::waypoint_callback, this, _1));
        // from the trajectory controller, the expected look ahead point to start the next
        // trajectory
        look_ahead_sub = this->create_subscription<airstack_msgs::msg::Odometry>(
            "look_ahead", 10, std::bind(&DroanLocalPlanner::look_ahead_callback, this, _1));
        // from the tracking controller, the current position of the drone
        tracking_point_sub = this->create_subscription<airstack_msgs::msg::Odometry>(
            "tracking_point", 10, std::bind(&DroanLocalPlanner::tracking_point_callback, this, _1));
        custom_waypoint_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "custom_waypoint", 1,
            std::bind(&DroanLocalPlanner::custom_waypoint_callback, this, _1));

        // publishers

        traj_lib_vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "trajectory_library_vis", 10);
        obst_vis_pub = this->create_publisher<sensor_msgs::msg::Range>("obstacle_vis", 10);
        global_plan_vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "local_planner_global_plan_vis", 10);
        traj_pub = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(
            "trajectory_segment_to_add", 10);
        traj_track_pub = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(
            "trajectory_override", 10);

        map_debug_pub =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("disparity_map_debug", 1);

        // init parameters
        this->declare_parameter("execute_rate", 5.);
        this->get_parameter("execute_rate", this->execute_rate);
        this->declare_parameter("safety_cost_weight", 1.);
        this->get_parameter("safety_cost_weight", safety_cost_weight);
        this->declare_parameter("obstacle_check_radius", 1.);
        this->get_parameter("obstacle_check_radius", obstacle_check_radius);
        this->declare_parameter("forward_progress_forgiveness_weight", 0.5);
        this->get_parameter("forward_progress_forgiveness_weight",
                            forward_progress_forgiveness_weight);
        this->declare_parameter("yaw_mode", "SMOOTH_YAW");
        auto yaw_mode_str = this->get_parameter("yaw_mode").as_string();
        if (yaw_mode_str == "TRAJECTORY_YAW") {
            this->yaw_mode = TRAJECTORY_YAW;
        } else if (yaw_mode_str == "SMOOTH_YAW") {
            this->yaw_mode = SMOOTH_YAW;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid yaw_mode parameter");
        }
        this->declare_parameter("cost_map", std::string("PointCloudMapRepresentation"));
        this->get_parameter("cost_map", map_representation_class_string);
        this->declare_parameter("auto_waypoint_buffer_duration", 30.);
        this->get_parameter("auto_waypoint_buffer_duration", auto_waypoint_buffer_duration);
        this->declare_parameter("auto_waypoint_spacing_threshold", 0.5);
        this->get_parameter("auto_waypoint_spacing_threshold", auto_waypoint_spacing_threshold);
        this->declare_parameter("auto_waypoint_angle_threshold", 30. * M_PI / 180.);
        this->get_parameter("auto_waypoint_angle_threshold", auto_waypoint_angle_threshold);

        this->declare_parameter("custom_waypoint_timeout_factor", 0.3);
        this->get_parameter("custom_waypoint_timeout_factor", custom_waypoint_timeout_factor);
        this->declare_parameter("custom_waypoint_distance_threshold", 0.5);
        this->get_parameter("custom_waypoint_distance_threshold",
                            custom_waypoint_distance_threshold);

        RCLCPP_INFO_STREAM(this->get_logger(), "DROAN node name is: " << this->get_name());
        this->declare_parameter("trajectory_library_config", std::string(""));
    }

    void initialize() {
        pluginlib::ClassLoader<cost_map_interface::CostMapInterface> map_representation_loader(
            "cost_map_interface", "cost_map_interface::CostMapInterface");
        auto node_ptr = this->shared_from_this();
        this->cost_map =
            map_representation_loader.createSharedInstance(map_representation_class_string);
        this->cost_map->initialize(node_ptr, tf_buffer_ptr);
        this->traj_lib = std::make_unique<TrajectoryLibrary>(
            this->get_parameter("trajectory_library_config").as_string(), node_ptr);
        double interval = 1. / this->execute_rate;
        this->execute_timer =
            rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(interval),
                                 std::bind(&DroanLocalPlanner::execute, this));
    }

    virtual ~DroanLocalPlanner() {}

    virtual bool execute() {
        this->update_waypoint_mode();

        if (!this->is_global_plan_received) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for global plan");
            return true;
        }
        if (!this->is_look_ahead_received) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for look ahead point");
            return true;
        }

        Trajectory global_plan(this, global_plan_msg);

        // transform the look ahead point to the global plan frame
        tf2::Vector3 look_ahead_position = tflib::to_tf(look_ahead_odom.pose.position);
        bool success = tflib::to_frame(tf_buffer_ptr.get(), look_ahead_position,
                                       look_ahead_odom.header.frame_id, global_plan.get_frame_id(),
                                       look_ahead_odom.header.stamp, &look_ahead_position);
        if (!success) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Couldn't transform from lookahead frame to global plan frame");
            return true;
        }

        // increment how far along the global plan we are
        double trajectory_distance;
        bool is_valid = global_plan.get_trajectory_distance_at_closest_point(look_ahead_position,
                                                                             &trajectory_distance);
        // trim the global plan to only the next 10 meters
        if (is_valid) {
            this->global_plan_trajectory_distance += trajectory_distance;
            global_plan = global_plan.trim_trajectory_between_distances(
                this->global_plan_trajectory_distance,
                this->global_plan_trajectory_distance + 10.0);
        } else {
            RCLCPP_INFO(this->get_logger(), "invalid");
        }

        // publish the trimmed segment of the global plan currently being used, for visualization
        visualization_msgs::msg::MarkerArray global_markers =
            global_plan.get_markers(this->now(), "global_plan", 0, 0, 1);
        global_plan_vis_pub->publish(global_markers);

        // get the dynamic trajectories
        std::vector<Trajectory> dynamic_trajectories =
            traj_lib->get_dynamic_trajectories(look_ahead_odom);
        
        // // debug to just one trajectory
        // dynamic_trajectories.resize(1);

        // pick the best trajectory
        auto [is_success, best_traj] = this->get_best_trajectory(dynamic_trajectories, global_plan);

        // publish the trajectory
        if (is_success) {
            airstack_msgs::msg::TrajectoryXYZVYaw best_traj_msg =
                best_traj.get_TrajectoryXYZVYaw_msg();

            // set yaw
            if (yaw_mode == SMOOTH_YAW && best_traj.get_num_waypoints() > 0) {
                apply_smooth_yaw(best_traj_msg);
            }
            best_traj_msg.header.stamp = this->now();
            traj_pub->publish(best_traj_msg);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Published local trajectory");
        } else {
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "No valid trajectories, all trajectories either collide or are unseen");
        }
        return true;
    }

    /**
     * @brief Get the best trajectory based on the cost function. Minimize the cost
     *
     * @param trajectory_candidates
     * @param global_plan
     * @return std::tuple<bool, Trajectory> is_success: whether there exists a valid collision-free
     * trajectory, best_traj: the best trajectory
     */
    std::tuple<bool, Trajectory> get_best_trajectory(
        const std::vector<Trajectory>& trajectory_candidates, Trajectory global_plan) {
        auto now = this->now();

        bool is_at_least_one_trajectory_valid = false;
        Trajectory best_traj_ret;
        double min_cost = std::numeric_limits<double>::max();

        visualization_msgs::msg::MarkerArray traj_lib_marker_arr;

        auto trajectory_safety_costs_per_waypoint =
            this->cost_map->get_trajectory_costs_per_waypoint(trajectory_candidates);

        // for each trajectory
        for (size_t i = 0; i < trajectory_candidates.size(); ++i) {
            Trajectory traj = trajectory_candidates[i];

            Trajectory global_plan_in_traj_frame;
            try {
                global_plan_in_traj_frame = global_plan.to_frame(traj.get_frame_id(), now);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to transform global plan to trajectory frame: %s", ex.what());
                return {false, best_traj_ret};
            }

            bool is_traj_unsafe_because_occupied = false;
            bool is_traj_unsafe_because_unobserved = false;

            double min_safety_cost = std::numeric_limits<double>::infinity();
            double total_deviation_from_global_plan = std::numeric_limits<double>::infinity();
            // std::cout << "Trajectory " << i << " safety costs: ";
            // for each waypoint in the trajectory, fetch its
            // (1) safety cost and (2) deviation from global plan cost
            for (size_t j = 0; j < traj.get_num_waypoints(); j++) {
                Waypoint wp = traj.get_waypoint(j);

                double safety_cost = trajectory_safety_costs_per_waypoint.at(i).at(j);

                // std::cout << safety_cost << " ";

                if (std::isinf(safety_cost)) {
                    is_traj_unsafe_because_occupied = true;
                    // don't consider this trajectory as a best option, just add the debug marker
                    goto add_marker;
                } else if (std::isnan(safety_cost)) {
                    is_traj_unsafe_because_unobserved = true;
                    goto add_marker;
                } else {
                    min_safety_cost = std::min(min_safety_cost, safety_cost);
                }

                // get the closest global plan point to the current trajectory waypoint
                auto [is_valid, closest_waypoint_to_global_plan, wp_index, path_distance] =
                    global_plan_in_traj_frame.get_closest_point(wp.position());

                // reward making progress along the global plan
                double forward_progress_forgiveness =
                    forward_progress_forgiveness_weight * path_distance;

                if (is_valid) {
                    if (std::isinf(total_deviation_from_global_plan)) {
                        total_deviation_from_global_plan = 0;
                    }
                    double deviation =
                        closest_waypoint_to_global_plan.position().distance(wp.position());
                    total_deviation_from_global_plan += deviation - forward_progress_forgiveness;
                }
            }

            // bigger distance from obstacles makes the cost smaller (more negative). cap by the
            // obstacle check radius
            if (!is_traj_unsafe_because_occupied && !is_traj_unsafe_because_unobserved) {
                is_at_least_one_trajectory_valid = true;

                double avg_deviation_from_global_plan =
                    total_deviation_from_global_plan / traj.get_num_waypoints();

                double cost = avg_deviation_from_global_plan + safety_cost_weight * min_safety_cost;
                if (cost < min_cost) {
                    min_cost = cost;
                    best_traj_ret = traj;
                }
            }

        add_marker:
            // std::cout << std::endl;

            // TODO: factor out this marker stuff
            std::string marker_ns = "trajectory_" + std::to_string(i);

            visualization_msgs::msg::MarkerArray traj_markers;
            if (is_traj_unsafe_because_unobserved) {
                // gray for unobserved
                traj_markers = traj.get_markers(this->now(), marker_ns, .7, .7, .7, .3);
            } else if (is_traj_unsafe_because_occupied) {
                // red for collision
                traj_markers = traj.get_markers(this->now(), marker_ns, 1, 0, 0, .3);
            } else {
                // green for no collision
                traj_markers = traj.get_markers(this->now(), marker_ns, 0, 1, 0, .5);
            }
            traj_lib_marker_arr.markers.insert(traj_lib_marker_arr.markers.end(),
                                               traj_markers.markers.begin(),
                                               traj_markers.markers.end());
        }

        traj_lib_vis_pub->publish(traj_lib_marker_arr);
        map_debug_pub->publish(this->cost_map->get_debug_markerarray());

        return {is_at_least_one_trajectory_valid, best_traj_ret};
    }

    /**
     * @brief Applies a smoothing filter to the yaw of the trajectory
     */
    void apply_smooth_yaw(airstack_msgs::msg::TrajectoryXYZVYaw& best_traj_msg) {
        bool found_initial_heading = false;
        double initial_heading = 0;
        try {
            tf2::Stamped<tf2::Transform> transform;
            tf_buffer_ptr->canTransform(
                best_traj_msg.header.frame_id, look_ahead_odom.header.frame_id,
                look_ahead_odom.header.stamp, rclcpp::Duration::from_seconds(0.1));
            auto transform_msg = tf_buffer_ptr->lookupTransform(best_traj_msg.header.frame_id,
                                                                look_ahead_odom.header.frame_id,
                                                                look_ahead_odom.header.stamp);
            tf2::fromMsg(transform_msg, transform);

            transform.setOrigin(tf2::Vector3(0, 0, 0));  // only care about rotation
            initial_heading =
                tf2::getYaw(transform * tflib::to_tf(look_ahead_odom.pose.orientation));

            found_initial_heading = true;

        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
        }

        if (found_initial_heading) {
            best_traj_msg.waypoints[0].yaw = initial_heading;
            double alpha = 0.1;
            double sin_yaw_prev = sin(best_traj_msg.waypoints[0].yaw);
            double cos_yaw_prev = cos(best_traj_msg.waypoints[0].yaw);

            for (size_t i = 1; i < best_traj_msg.waypoints.size(); i++) {
                airstack_msgs::msg::WaypointXYZVYaw wp_prev = best_traj_msg.waypoints[i - 1];
                airstack_msgs::msg::WaypointXYZVYaw& wp_curr = best_traj_msg.waypoints[i];

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

    // subscriber callbacks

    /**
     * @brief Saves the global plan if the goal mode is GLOBAL_PLAN
     *
     * @param global_plan_msg
     */
    void global_plan_callback(const nav_msgs::msg::Path::SharedPtr global_plan_msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "GOT GLOBAL PLAN, goal_mode: " << this->goal_mode);
        if (this->goal_mode != GLOBAL_PLAN) return;

        this->global_plan_msg = *global_plan_msg;  // copies
        this->is_global_plan_received = true;
        this->global_plan_trajectory_distance = 0;
    }

    /**
     * @brief If mode == AUTO_WAYPOINT, then the local planner will automatically interpolate
     * waypoints to reach the desired waypoint. This callback happens when mode == AUTO_WAYPOINT and
     * it receives a waypoint message.
     *
     * @param wp the desired waypoint
     */
    void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr wp) {
        if (this->goal_mode != AUTO_WAYPOINT) {
            this->waypoint_buffer.clear();
            this->waypoint_buffer.push_back(*wp);
            return;
        }

        // remove old waypoints if necessary
        this->waypoint_buffer.push_back(*wp);
        if ((rclcpp::Time(wp->header.stamp) - rclcpp::Time(waypoint_buffer.front().header.stamp))
                .seconds() > auto_waypoint_buffer_duration) {
            waypoint_buffer.pop_front();
        }

        // stitch together the history of waypoints
        nav_msgs::msg::Path global_plan_msg;
        global_plan_msg.header.frame_id = wp->header.frame_id;
        global_plan_msg.header.stamp = wp->header.stamp;

        std::vector<geometry_msgs::msg::PoseStamped> backwards_global_plan;

        std::vector<geometry_msgs::msg::PointStamped> prev_wps;
        for (auto it = waypoint_buffer.rbegin(); it != waypoint_buffer.rend(); it++) {
            geometry_msgs::msg::PointStamped curr_wp = *it;
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.pose.position.x = curr_wp.point.x;
            waypoint.pose.position.y = curr_wp.point.y;
            waypoint.pose.position.z = curr_wp.point.z;

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

                if (distance >= auto_waypoint_spacing_threshold &&
                    angle_diff < auto_waypoint_angle_threshold) {
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
                    if (distance >= auto_waypoint_spacing_threshold) {
                        prev_wps.push_back(curr_wp);
                        backwards_global_plan.push_back(waypoint);
                    }
                }
            }
        }
        for (size_t i = 0; i < backwards_global_plan.size(); i++) {
            global_plan_msg.poses.push_back(
                backwards_global_plan[backwards_global_plan.size() - 1 - i]);
        }
        if (is_tracking_point_received) {
            try {
                tf2::Stamped<tf2::Transform> transform;
                tf_buffer_ptr->canTransform(tracking_point_odom.header.frame_id,
                                            wp->header.frame_id, wp->header.stamp,
                                            rclcpp::Duration::from_seconds(0.1));
                auto transform_msg = tf_buffer_ptr->lookupTransform(
                    tracking_point_odom.header.frame_id, wp->header.frame_id, wp->header.stamp);
                tf2::fromMsg(transform_msg, transform);

                tf2::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.position);
                tf2::Vector3 wp_position = transform * tflib::to_tf(wp->point);

                tf2::Vector3 direction = (wp_position - tp_position).normalized() * 3;
                tf2::Vector3 wp2_position = wp_position + direction;

                geometry_msgs::msg::PoseStamped wp1, wp2;
                wp1.pose.position.x = wp_position.x();
                wp1.pose.position.y = wp_position.y();
                wp1.pose.position.z = wp_position.z();
                wp2.pose.position.x = wp2_position.x();
                wp2.pose.position.y = wp2_position.y();
                wp2.pose.position.z = wp2_position.z();
                global_plan_msg.poses.push_back(wp1);
                global_plan_msg.poses.push_back(wp2);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            }
        }

        global_plan_trajectory_distance = 0;
        this->global_plan_msg = global_plan_msg;
        this->is_global_plan_received = true;
    }

    /**
     * @brief Clears the current global plan reference and go immediately to a custom waypoint
     * (typically set by the user). Sets the global plan to be the line between the current look
     * ahead point and the custom waypoint.
     *
     * @param wp
     */
    void custom_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr wp) {
        if (!is_look_ahead_received) return;

        try {
            tf2::Stamped<tf2::Transform> transform;
            tf_buffer_ptr->canTransform(look_ahead_odom.header.frame_id, wp->header.frame_id,
                                        wp->header.stamp, rclcpp::Duration::from_seconds(0.1));
            auto transform_msg = tf_buffer_ptr->lookupTransform(
                look_ahead_odom.header.frame_id, wp->header.frame_id, wp->header.stamp);
            tf2::fromMsg(transform_msg, transform);

            tf2::Vector3 la_position = tflib::to_tf(look_ahead_odom.pose.position);
            tf2::Vector3 wp_position = transform * tflib::to_tf(wp->pose.position);

            nav_msgs::msg::Path global_plan_msg;
            global_plan_msg.header.frame_id = look_ahead_odom.header.frame_id;
            global_plan_msg.header.stamp = this->now();

            geometry_msgs::msg::PoseStamped wp1, wp2;
            wp1.pose.position.x = la_position.x();
            wp1.pose.position.y = la_position.y();
            wp1.pose.position.z = la_position.z();
            wp2.pose.position.x = wp_position.x();
            wp2.pose.position.y = wp_position.y();
            wp2.pose.position.z = wp_position.z();
            global_plan_msg.poses.push_back(wp1);
            global_plan_msg.poses.push_back(wp2);

            global_plan_trajectory_distance = 0;
            this->global_plan_msg = global_plan_msg;
            this->is_global_plan_received = true;

            goal_mode = CUSTOM_WAYPOINT;
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
        }
    }

    /**
     * Check if we should switch from CUSTOM_WAYPOINT to AUTO_WAYPOINT
     */
    void update_waypoint_mode() {
        if (goal_mode == CUSTOM_WAYPOINT) {
            if (global_plan_msg.poses.size() < 2) goal_mode = AUTO_WAYPOINT;

            // check if the time limit for reaching the waypoint has elapsed
            double elapsed_time = (this->now() - global_plan_msg.header.stamp).seconds();
            double distance = 0;
            for (size_t i = 1; i < global_plan_msg.poses.size(); i++) {
                geometry_msgs::msg::Pose prev_wp, curr_wp;
                prev_wp = global_plan_msg.poses[i - 1].pose;
                curr_wp = global_plan_msg.poses[i].pose;

                distance += sqrt(pow(prev_wp.position.x - curr_wp.position.x, 2) +
                                 pow(prev_wp.position.y - curr_wp.position.y, 2));
            }

            // ROS_INFO_STREAM("elapsed: " << elapsed_time << " / " <<
            // distance/custom_waypoint_timeout_factor << " distance: " << distance);
            if (elapsed_time >= distance / this->custom_waypoint_timeout_factor) {
                // ROS_INFO_STREAM("CUSTOM WAYPOINT TIMEOUT REACHED");
                goal_mode = AUTO_WAYPOINT;
            }

            // check if we are close enough to the waypoint
            if (is_tracking_point_received) {
                try {
                    tf2::Stamped<tf2::Transform> transform;
                    tf_buffer_ptr->canTransform(
                        tracking_point_odom.header.frame_id, global_plan_msg.header.frame_id,
                        global_plan_msg.header.stamp, rclcpp::Duration::from_seconds(0.1));
                    auto transform_msg = tf_buffer_ptr->lookupTransform(
                        tracking_point_odom.header.frame_id, global_plan_msg.header.frame_id,
                        global_plan_msg.header.stamp);
                    tf2::fromMsg(transform_msg, transform);

                    tf2::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.position);
                    tp_position.setZ(0);
                    tf2::Vector3 wp_position =
                        transform * tflib::to_tf(global_plan_msg.poses.back().pose.position);
                    wp_position.setZ(0);

                    if (tp_position.distance(wp_position) < custom_waypoint_distance_threshold) {
                        // ROS_INFO_STREAM("CUSTOM WAYPOINT DISTANCE THRESHOLD MET");
                        goal_mode = AUTO_WAYPOINT;
                    }
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR_STREAM(
                        this->get_logger(),
                        "TransformException in update_waypoint_mode: " << ex.what());
                }
            }
        }
    }
    /**
     * @brief Receive the look ahead point to plan the next local trajectory from
     *
     * @param odom
     */
    void look_ahead_callback(const airstack_msgs::msg::Odometry::SharedPtr odom) {
        is_look_ahead_received = true;
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                "look ahead received with frame id: " << odom->header.frame_id);
        look_ahead_odom = *odom;
    }
    void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr odom) {
        is_tracking_point_received = true;
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                "tracking point received with frame id: " << odom->header.frame_id);
        tracking_point_odom = *odom;
    }
};
