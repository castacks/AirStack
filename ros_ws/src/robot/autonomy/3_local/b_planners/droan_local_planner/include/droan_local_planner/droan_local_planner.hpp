#pragma once

#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_representation_interface/map_representation.hpp>
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

using std::placeholders::_1;

class DroanLocalPlanner : public rclcpp::Node {
   private:
    std::unique_ptr<TrajectoryLibrary> traj_lib;

    std::string map_representation_class_string;
    bool is_global_plan_received;
    airstack_msgs::msg::TrajectoryXYZVYaw global_plan_msg;
    double global_plan_trajectory_distance;
    bool is_look_ahead_received, is_tracking_point_received;
    airstack_msgs::msg::Odometry look_ahead_odom, tracking_point_odom;

    std::vector<Trajectory> static_trajectories;

    double waypoint_spacing, obstacle_check_radius, obstacle_distance_reward,
        forward_progress_reward_weight;
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

    std::shared_ptr<map_representation_interface::MapRepresentation> map_representation;

    rclcpp::Subscription<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr global_plan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
    rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
    rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_track_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr obst_vis_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_plan_vis_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr look_past_vis_pub;

    // services
    // ros::ServiceClient traj_mode_client;
    rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_client;

   public:
    DroanLocalPlanner()
        : Node("droan_local_planner"),
          goal_mode(TRAJECTORY),
          tf_buffer(this->get_clock()),
          tf_listener(tf_buffer) {
        // subscribers
        global_plan_sub = this->create_subscription<airstack_msgs::msg::TrajectoryXYZVYaw>(
            "global_plan", 10, std::bind(&DroanLocalPlanner::global_plan_callback, this, _1));
        waypoint_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "way_point", 10, std::bind(&DroanLocalPlanner::waypoint_callback, this, _1));
        look_ahead_sub = this->create_subscription<airstack_msgs::msg::Odometry>(
            "look_ahead", 10, std::bind(&DroanLocalPlanner::look_ahead_callback, this, _1));
        tracking_point_sub = this->create_subscription<airstack_msgs::msg::Odometry>(
            "tracking_point", 10, std::bind(&DroanLocalPlanner::tracking_point_callback, this, _1));
        range_up_sub = this->create_subscription<sensor_msgs::msg::Range>(
            "range_up", 1, std::bind(&DroanLocalPlanner::range_up_callback, this, _1));
        range_down_sub = this->create_subscription<sensor_msgs::msg::Range>(
            "range_down", 1, std::bind(&DroanLocalPlanner::range_down_callback, this, _1));
        custom_waypoint_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "custom_waypoint", 1,
            std::bind(&DroanLocalPlanner::custom_waypoint_callback, this, _1));

        // publishers

        vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "trajectory_library_vis", 10);
        obst_vis_pub = this->create_publisher<sensor_msgs::msg::Range>("obstacle_vis", 10);
        global_plan_vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "local_planner_global_plan_vis", 10);
        look_past_vis_pub =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("look_past", 10);
        traj_pub = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory", 10);
        traj_track_pub =
            this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_track", 10);

        // services
        traj_mode_client =
            this->create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode");

        // init parameters
        this->declare_parameter("waypoint_spacing", 0.5);
        waypoint_spacing = this->get_parameter("waypoint_spacing").as_double();
        this->declare_parameter("obstacle_distance_reward", 1.);
        obstacle_distance_reward = this->get_parameter("obstacle_distance_reward").as_double();
        this->declare_parameter("forward_progress_reward_weight", 0.5);
        forward_progress_reward_weight =
            this->get_parameter("forward_progress_reward_weight").as_double();
        this->declare_parameter("robot_radius", 0.75);
        robot_radius = this->get_parameter("robot_radius").as_double();
        this->declare_parameter("look_past_distance", 0.0);
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
        map_representation_class_string = this->get_parameter("map_representation").as_string();
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
            this->get_parameter("trajectory_library_config").as_string(), this);

        pluginlib::ClassLoader<map_representation_interface::MapRepresentation>
            map_representation_loader("map_representation_interface",
                                      "map_representation_interface::MapRepresentation");
        try {
            map_representation =
                map_representation_loader.createSharedInstance(map_representation_class_string);
        } catch (pluginlib::PluginlibException& ex) {
            RCLCPP_INFO(this->get_logger(),
                        "The MapRepresentation plugin failed to load. Error: %s", ex.what());
        }
    }
    virtual ~DroanLocalPlanner() {}

    /**
     * Sets the global plan height  based on the height mode
     */
    bool set_global_plan_height_inplace(Trajectory& global_plan) {
        // fixed height mode
        if (this->height_mode == FIXED_HEIGHT) {
            global_plan.set_fixed_height(fixed_height);
        }
        // set between the range sensors
        else if (this->height_mode == RANGE_SENSOR_HEIGHT) {
            if (!got_range_up || !got_range_down) return true;

            try {
                tf2::Stamped<tf2::Transform> transform_up, transform_down;

                geometry_msgs::msg::TransformStamped tf_up_msg, tf_down_msg;
                tf_buffer.canTransform(global_plan.get_frame_id(), range_up.header.frame_id,
                                       range_up.header.stamp, rclcpp::Duration::from_seconds(0.1));
                tf_up_msg = tf_buffer.lookupTransform(
                    global_plan.get_frame_id(), range_up.header.frame_id, range_up.header.stamp);
                tf2::fromMsg(tf_up_msg, transform_up);
                tf_buffer.canTransform(global_plan.get_frame_id(), range_down.header.frame_id,
                                       range_down.header.stamp,
                                       rclcpp::Duration::from_seconds(0.1));
                tf_down_msg =
                    tf_buffer.lookupTransform(global_plan.get_frame_id(),
                                              range_down.header.frame_id, range_down.header.stamp);
                tf2::fromMsg(tf_down_msg, transform_down);

                tf2::Vector3 range_up_gp_frame = transform_up * tf2::Vector3(range_up.range, 0, 0);
                tf2::Vector3 range_down_gp_frame =
                    transform_down * tf2::Vector3(range_down.range, 0, 0);

                double tunnel_height = range_up_gp_frame.z() - range_down_gp_frame.z();
                double z_setpoint = (range_up_gp_frame.z() + range_down_gp_frame.z()) / 2.0;
                if (tunnel_height / 2.0 >= height_above_ground) {
                    z_setpoint = range_down_gp_frame.z() + height_above_ground;
                }

                global_plan.set_fixed_height(z_setpoint);

            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
                return true;
            }
        }
    }

    virtual bool execute() {
        update_waypoint_mode();

        if (!is_global_plan_received) return true;

        Trajectory global_plan(this, global_plan_msg);
        set_global_plan_height_inplace(global_plan);  // set the height of the global plan

        // transform the look ahead point to the global plan frame
        tf2::Vector3 look_ahead_position = tflib::to_tf(look_ahead_odom.pose.position);
        bool success = tflib::to_frame(&tf_buffer, look_ahead_position,
                                       look_ahead_odom.header.frame_id, global_plan.get_frame_id(),
                                       look_ahead_odom.header.stamp, &look_ahead_position);
        if (!success) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Couldn't transform from lookahead frame to global plan frame");
            return true;
        }

        // increment how far along the global plan we are
        double trajectory_distance;
        bool valid = global_plan.get_trajectory_distance_at_closest_point(look_ahead_position,
                                                                          &trajectory_distance);
        if (valid) {
            global_plan_trajectory_distance += trajectory_distance;
            global_plan = global_plan.get_subtrajectory_distance(
                global_plan_trajectory_distance, global_plan_trajectory_distance + 10.0);
        } else {
            RCLCPP_INFO(this->get_logger(), "invalid");
        }

        // publish the segment of the global plan currently being used, for visualization
        visualization_msgs::msg::MarkerArray global_markers =
            global_plan.get_markers(this->now(), 0, 0, 1);
        global_plan_vis_pub->publish(global_markers);

        // get the dynamic trajectories
        std::vector<Trajectory> dynamic_trajectories =
            traj_lib->get_dynamic_trajectories(look_ahead_odom);

        // pick the best trajectory
        Trajectory best_traj;
        bool all_in_collision =
            this->get_best_trajectory(dynamic_trajectories, global_plan, best_traj);

        // publish the trajectory
        if (!all_in_collision) {
            airstack_msgs::msg::TrajectoryXYZVYaw best_traj_msg = best_traj.get_TrajectoryXYZVYaw();

            // set yaw
            if (yaw_mode == SMOOTH_YAW && best_traj.waypoint_count() > 0) {
                bool found_initial_heading = false;
                double initial_heading = 0;
                try {
                    tf2::Stamped<tf2::Transform> transform;
                    tf_buffer.canTransform(
                        best_traj.get_frame_id(), look_ahead_odom.header.frame_id,
                        look_ahead_odom.header.stamp, rclcpp::Duration::from_seconds(0.1));
                    auto transform_msg = tf_buffer.lookupTransform(best_traj.get_frame_id(),
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
                        airstack_msgs::msg::WaypointXYZVYaw wp_prev =
                            best_traj_msg.waypoints[i - 1];
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
            best_traj_msg.header.stamp = this->now();
            traj_pub->publish(best_traj_msg);
        }
        return true;
    }

    std::vector<std::vector<double>> get_trajectory_distances_from_map(
        std::vector<Trajectory> trajectory_candidates) {
        // vector of trajectory candidates in PointStamped-vector form
        std::vector<std::vector<geometry_msgs::msg::PointStamped>> traj_cands_as_point_stamped;
        for (size_t i = 0; i < trajectory_candidates.size(); i++) {
            traj_cands_as_point_stamped.push_back(
                trajectory_candidates[i].get_vector_PointStamped());
        }
        // TODO: clearly we should refactor map_representation to accept Trajectory objects
        std::vector<std::vector<double>> trajectory_distances_to_closest_obstacle =
            this->map_representation->get_values(traj_cands_as_point_stamped);
        return trajectory_distances_to_closest_obstacle;
    }

    /**
     * Choose the best trajectory based on the cost function. Minimize the cost
     */
    bool get_best_trajectory(std::vector<Trajectory> trajectory_candidates, Trajectory global_plan,
                             Trajectory& best_traj_ret) {
        double min_cost = std::numeric_limits<double>::max();
        size_t best_traj_index = 0;
        bool all_in_collision = true;

        auto now = this->now();

        auto trajectory_distances_to_closest_obstacle =
            get_trajectory_distances_from_map(trajectory_candidates);

        visualization_msgs::msg::MarkerArray traj_lib_markers, look_past_markers;

        for (size_t i = 0; i < trajectory_candidates.size(); ++i) {
            Trajectory traj = trajectory_candidates[i];
            double avg_distance_from_global_plan = std::numeric_limits<double>::infinity();
            double closest_obstacle_distance = std::numeric_limits<double>::infinity();

            Trajectory global_plan_in_traj_frame;
            try {
                global_plan_in_traj_frame = global_plan.to_frame(traj.get_frame_id(), now);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to transform global plan to trajectory frame: %s", ex.what());
                return true;
            }

            // for each waypoint in the trajectory, calculate the distance to the closest obstacle
            for (size_t j = 0; j < traj.waypoint_count(); j++) {
                Waypoint wp = traj.get_waypoint(j);

                airstack_msgs::msg::Odometry odom = wp.odometry(now, traj.get_frame_id());
                geometry_msgs::msg::PoseStamped pose;
                pose.header = odom.header;
                pose.pose = odom.pose;
                closest_obstacle_distance =
                    std::min(closest_obstacle_distance,
                             trajectory_distances_to_closest_obstacle.at(i).at(j));

                // get the closest global plan point to the current trajectory waypoint
                Waypoint closest_point_from_global_plan(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                int wp_index;
                double path_distance;
                bool valid = global_plan_in_traj_frame.get_closest_point(
                    wp.position(), &closest_point_from_global_plan, &wp_index, &path_distance);

                // reward making progress along the global plan
                double forward_progress_reward = -forward_progress_reward_weight * path_distance;

                if (valid) {
                    if (!std::isfinite(avg_distance_from_global_plan)) {
                        avg_distance_from_global_plan = 0;
                    }
                    avg_distance_from_global_plan +=
                        closest_point_from_global_plan.position().distance(wp.position()) +
                        forward_progress_reward;
                }
            }
            avg_distance_from_global_plan /= traj.waypoint_count();

            bool collision = closest_obstacle_distance <= robot_radius;
            if (!collision) {
                all_in_collision = false;
            }

            visualization_msgs::msg::MarkerArray traj_markers;
            if (collision) {
                // red for collision
                traj_markers = traj.get_markers(this->now(), 1, 0, 0, .5);
            } else {
                // green for no collision
                traj_markers = traj.get_markers(this->now(), 0, 1, 0, .5);
            }

            // if look_past_distance is set, then use that to calculate the cost instead
            // wtf this doesn't even make sense
            if (look_past_distance > 0) {
                if (traj.waypoint_count() >= 2) {
                    Waypoint curr_wp = traj.get_waypoint(traj.waypoint_count() - 1);
                    Waypoint prev_wp = traj.get_waypoint(traj.waypoint_count() - 2);

                    tf2::Vector3 direction = curr_wp.position() - prev_wp.position();
                    direction.normalize();

                    tf2::Vector3 look_past_position =
                        curr_wp.position() + look_past_distance * direction;

                    Waypoint closest_point_from_global_plan(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                    int wp_index;
                    bool valid = global_plan_in_traj_frame.get_closest_point(
                        look_past_position, &closest_point_from_global_plan, &wp_index);

                    if (!valid) {
                        collision = true;
                    } else {
                        avg_distance_from_global_plan =
                            look_past_position.distance(closest_point_from_global_plan.position());
                    }

                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = traj.get_frame_id();
                    marker.header.stamp = now;
                    marker.ns = "look_past";
                    marker.id = i;
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.action = visualization_msgs::msg::Marker::ADD;

                    marker.pose.position.x = look_past_position.x();
                    marker.pose.position.y = look_past_position.y();
                    marker.pose.position.z = look_past_position.z();
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
            // bigger distance from obstacles makes the cost smaller (more negative). cap by the
            // obstacle check radius
            double cost = avg_distance_from_global_plan -
                          obstacle_distance_reward *
                              std::min(closest_obstacle_distance, obstacle_check_radius);
            if (!collision && cost < min_cost) {
                min_cost = cost;
                best_traj_index = i;
                best_traj_ret = traj;
            }
        }
        if (best_traj_index < look_past_markers.markers.size()) {
            look_past_markers.markers[best_traj_index].color.r = 0;
            look_past_markers.markers[best_traj_index].color.g = 1;
            look_past_markers.markers[best_traj_index].color.b = 0;
            look_past_markers.markers[best_traj_index].color.a = 1;
        }

        vis_pub->publish(traj_lib_markers);
        map_representation->publish_debug();
        look_past_vis_pub->publish(look_past_markers);

        return all_in_collision;
    }

    // subscriber callbacks
    void global_plan_callback(
        const airstack_msgs::msg::TrajectoryXYZVYaw::SharedPtr global_plan_msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "GOT GLOBAL PLAN, goal_mode: " << goal_mode);
        if (goal_mode != TRAJECTORY) return;

        this->global_plan_msg = *global_plan_msg;  // copies
        is_global_plan_received = true;
        global_plan_trajectory_distance = 0;
    }

    void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr wp) {
        if (goal_mode != AUTO_WAYPOINT) {
            waypoint_buffer.clear();
            waypoint_buffer.push_back(*wp);
            return;
        }

        // remove old waypoints if necessary
        waypoint_buffer.push_back(*wp);
        if ((rclcpp::Time(wp->header.stamp) - rclcpp::Time(waypoint_buffer.front().header.stamp))
                .seconds() > waypoint_buffer_duration) {
            waypoint_buffer.pop_front();
        }

        // stitch together the history of waypoints
        airstack_msgs::msg::TrajectoryXYZVYaw global_plan_msg;
        global_plan_msg.header.frame_id = wp->header.frame_id;
        global_plan_msg.header.stamp = wp->header.stamp;

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
        for (size_t i = 0; i < backwards_global_plan.size(); i++) {
            global_plan_msg.waypoints.push_back(
                backwards_global_plan[backwards_global_plan.size() - 1 - i]);
        }
        if (is_tracking_point_received) {
            try {
                tf2::Stamped<tf2::Transform> transform;
                tf_buffer.canTransform(tracking_point_odom.header.frame_id, wp->header.frame_id,
                                       wp->header.stamp, rclcpp::Duration::from_seconds(0.1));
                auto transform_msg = tf_buffer.lookupTransform(
                    tracking_point_odom.header.frame_id, wp->header.frame_id, wp->header.stamp);
                tf2::fromMsg(transform_msg, transform);

                tf2::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.position);
                tf2::Vector3 wp_position = transform * tflib::to_tf(wp->point);

                tf2::Vector3 direction = (wp_position - tp_position).normalized() * 3;
                tf2::Vector3 wp2_position = wp_position + direction;

                airstack_msgs::msg::WaypointXYZVYaw wp1, wp2;
                wp1.position.x = wp_position.x();
                wp1.position.y = wp_position.y();
                wp1.position.z = wp_position.z();
                wp2.position.x = wp2_position.x();
                wp2.position.y = wp2_position.y();
                wp2.position.z = wp2_position.z();
                global_plan_msg.waypoints.push_back(wp1);
                global_plan_msg.waypoints.push_back(wp2);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            }
        }

        global_plan_trajectory_distance = 0;
        this->global_plan_msg = global_plan_msg;
        this->is_global_plan_received = true;
    }

    void custom_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr wp) {
        if (!is_look_ahead_received) return;

        try {
            tf2::Stamped<tf2::Transform> transform;
            tf_buffer.canTransform(look_ahead_odom.header.frame_id, wp->header.frame_id,
                                   wp->header.stamp, rclcpp::Duration::from_seconds(0.1));
            auto transform_msg = tf_buffer.lookupTransform(look_ahead_odom.header.frame_id,
                                                           wp->header.frame_id, wp->header.stamp);
            tf2::fromMsg(transform_msg, transform);

            tf2::Vector3 la_position = tflib::to_tf(look_ahead_odom.pose.position);
            tf2::Vector3 wp_position = transform * tflib::to_tf(wp->pose.position);

            airstack_msgs::msg::TrajectoryXYZVYaw global_plan_msg;
            global_plan_msg.header.frame_id = look_ahead_odom.header.frame_id;
            global_plan_msg.header.stamp = this->now();

            airstack_msgs::msg::WaypointXYZVYaw wp1, wp2;
            wp1.position.x = la_position.x();
            wp1.position.y = la_position.y();
            wp1.position.z = la_position.z();
            wp2.position.x = wp_position.x();
            wp2.position.y = wp_position.y();
            wp2.position.z = wp_position.z();
            global_plan_msg.waypoints.push_back(wp1);
            global_plan_msg.waypoints.push_back(wp2);

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
            if (global_plan_msg.waypoints.size() < 2) goal_mode = AUTO_WAYPOINT;

            // check if the time limit for reaching the waypoint has elapsed
            double elapsed_time = (this->now() - global_plan_msg.header.stamp).seconds();
            double distance = 0;
            for (size_t i = 1; i < global_plan_msg.waypoints.size(); i++) {
                airstack_msgs::msg::WaypointXYZVYaw prev_wp, curr_wp;
                prev_wp = global_plan_msg.waypoints[i - 1];
                curr_wp = global_plan_msg.waypoints[i];

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
            if (is_tracking_point_received) {
                try {
                    tf2::Stamped<tf2::Transform> transform;
                    tf_buffer.canTransform(
                        tracking_point_odom.header.frame_id, global_plan_msg.header.frame_id,
                        global_plan_msg.header.stamp, rclcpp::Duration::from_seconds(0.1));
                    auto transform_msg = tf_buffer.lookupTransform(
                        tracking_point_odom.header.frame_id, global_plan_msg.header.frame_id,
                        global_plan_msg.header.stamp);
                    tf2::fromMsg(transform_msg, transform);

                    tf2::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.position);
                    tp_position.setZ(0);
                    tf2::Vector3 wp_position =
                        transform * tflib::to_tf(global_plan_msg.waypoints.back().position);
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
    void look_ahead_callback(const airstack_msgs::msg::Odometry::SharedPtr odom) {
        is_look_ahead_received = true;
        look_ahead_odom = *odom;
    }
    void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr odom) {
        is_tracking_point_received = true;
        tracking_point_odom = *odom;
    }
    void range_up_callback(const sensor_msgs::msg::Range::SharedPtr range_msg) {
        got_range_up = true;
        range_up = *range_msg;
    }
    void range_down_callback(const sensor_msgs::msg::Range::SharedPtr range_msg) {
        got_range_down = true;
        range_down = *range_msg;
    }
};
