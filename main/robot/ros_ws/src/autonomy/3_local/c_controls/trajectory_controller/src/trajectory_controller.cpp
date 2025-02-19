#include <airstack_common/ros2_helper.hpp>
#include <airstack_common/tflib.hpp>
#include <airstack_common/vislib.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_library/trajectory_library.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//===================================================================================
//----------------------------- Trajectory Control Node -----------------------------
//===================================================================================

class TrajectoryControlNode : public rclcpp::Node {
   private:
    rclcpp::Subscription<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_seg_to_add_sub;
    rclcpp::Subscription<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_override_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_vis_pub;
    rclcpp::Publisher<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_pub;
    rclcpp::Publisher<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_pub;
    rclcpp::Publisher<airstack_msgs::msg::Odometry>::SharedPtr drone_point_pub;
    rclcpp::Publisher<airstack_msgs::msg::Odometry>::SharedPtr virtual_tracking_point_pub;
    rclcpp::Publisher<airstack_msgs::msg::Odometry>::SharedPtr closest_point_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trajectory_completion_percentage_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trajectory_time_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tracking_error_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub;

    tf2_ros::TransformBroadcaster* tf_broadcaster;
    tf2_ros::TransformListener* tf_listener;
    tf2_ros::Buffer* tf_buffer;

    rclcpp::Service<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_srv;

    rclcpp::TimerBase::SharedPtr timer;

    nav_msgs::msg::Odometry odom;
    bool got_odom;

    double target_dt;
    std::string tf_prefix;
    std::string target_frame;
    int trajectory_mode;
    Trajectory* trajectory;
    double prev_time;
    double virtual_time;
    double actual_time;
    airstack_msgs::msg::Odometry look_ahead_point, drone_point;
    airstack_msgs::msg::Odometry virtual_tracking_point_odom;
    airstack_msgs::msg::Odometry closest_point_odom;
    double tracking_point_distance_limit;
    double velocity_look_ahead_time;
    double look_ahead_time;
    double virtual_tracking_ahead_time;
    double min_virtual_tracking_velocity;
    double sphere_radius;
    double ff_min_velocity;
    double search_ahead_factor;
    double prev_vtp_time;
    float traj_vis_thickness;
    float current_velocity;
    float time_multiplier;
    float transition_velocity_scale;
    float transition_dt;
    bool new_rewind;
    float rewind_skip_max_velocity;
    float rewind_skip_max_distance;

    vis::MarkerArray markers;

   public:
    TrajectoryControlNode();
    void timer_callback();

    void traj_seg_to_add_callback(const airstack_msgs::msg::TrajectoryXYZVYaw::SharedPtr traj);
    void traj_override_callback(const airstack_msgs::msg::TrajectoryXYZVYaw::SharedPtr traj);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void set_trajectory_mode(const std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Request> req,
                             std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Response> res);

    std::string mode;
    float velocity_target;
};

TrajectoryControlNode::TrajectoryControlNode() : rclcpp::Node("trajectory_control_node") {
    // init params
    target_dt = 1. / airstack::get_param(this, "execute_target", 50.);
    tf_prefix = airstack::get_param(this, "tf_prefix", std::string(""));
    target_frame = airstack::get_param(this, "target_frame", std::string("world"));
    tracking_point_distance_limit = airstack::get_param(this, "tracking_point_distance_limit", 0.5);
    velocity_look_ahead_time = airstack::get_param(this, "velocity_look_ahead_time", 0.0);
    look_ahead_time = airstack::get_param(this, "look_ahead_time", 1.0);
    virtual_tracking_ahead_time = airstack::get_param(this, "virtual_tracking_ahead_time", 0.5);
    min_virtual_tracking_velocity = airstack::get_param(this, "min_virtual_tracking_velocity", 0.3);
    sphere_radius = airstack::get_param(this, "sphere_radius", 0.3);
    ff_min_velocity = airstack::get_param(this, "ff_min_velocity", 0.4);
    search_ahead_factor = airstack::get_param(this, "search_ahead_factor", 1.5);
    transition_velocity_scale = airstack::get_param(this, "transition_velocity_scale", 1.0);
    traj_vis_thickness = airstack::get_param(this, "traj_vis_thickness", 0.03);
    rewind_skip_max_velocity = airstack::get_param(this, "rewind_skip_max_velocity", 0.1);
    rewind_skip_max_distance = airstack::get_param(this, "rewind_skip_max_distance", 0.1);
    got_odom = false;

    trajectory_mode = airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE;
    trajectory = new Trajectory(this, target_frame);

    // init subscribers
    traj_seg_to_add_sub = this->create_subscription<airstack_msgs::msg::TrajectoryXYZVYaw>(
        "trajectory_segment_to_add", 1,
        std::bind(&TrajectoryControlNode::traj_seg_to_add_callback, this, std::placeholders::_1));
    traj_override_sub = this->create_subscription<airstack_msgs::msg::TrajectoryXYZVYaw>(
        "trajectory_override", 1,
        std::bind(&TrajectoryControlNode::traj_override_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 1,
        std::bind(&TrajectoryControlNode::odom_callback, this, std::placeholders::_1));

    tf_broadcaster = new tf2_ros::TransformBroadcaster(*this);
    tf_buffer = new tf2_ros::Buffer(this->get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    // init publishers
    trajectory_vis_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_vis", 1);
    tracking_point_pub = this->create_publisher<airstack_msgs::msg::Odometry>("tracking_point", 1);
    look_ahead_pub = this->create_publisher<airstack_msgs::msg::Odometry>("look_ahead", 1);
    drone_point_pub = this->create_publisher<airstack_msgs::msg::Odometry>("traj_drone_point", 1);
    virtual_tracking_point_pub =
        this->create_publisher<airstack_msgs::msg::Odometry>("virtual_tracking_point", 1);
    closest_point_pub = this->create_publisher<airstack_msgs::msg::Odometry>("closest_point", 1);
    trajectory_completion_percentage_pub =
        this->create_publisher<std_msgs::msg::Float32>("trajectory_completion_percentage", 1);
    trajectory_time_pub = this->create_publisher<std_msgs::msg::Float32>("trajectory_time", 1);
    tracking_error_pub = this->create_publisher<std_msgs::msg::Float32>("tracking_error", 1);
    velocity_pub =
        this->create_publisher<std_msgs::msg::Float32>("tracking_point_velocity_magnitude", 1);
    debug_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "trajectory_controller_debug_markers", 1);
    // broadcaster = new tf::TransformBroadcaster();
    // listener = new tf::TransformListener();

    // init services
    traj_mode_srv = this->create_service<airstack_msgs::srv::TrajectoryMode>(
        "set_trajectory_mode", std::bind(&TrajectoryControlNode::set_trajectory_mode, this,
                                         std::placeholders::_1, std::placeholders::_2));

    // timers
    timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1. / 20.),
                                 std::bind(&TrajectoryControlNode::timer_callback, this));

    virtual_time = 0;
    actual_time = 0;
    prev_time = 0;
    virtual_tracking_point_odom.header.frame_id = target_frame;
    virtual_tracking_point_odom.child_frame_id = target_frame;
    look_ahead_point.header.frame_id = target_frame;
    look_ahead_point.child_frame_id = target_frame;
    drone_point.header.frame_id = target_frame;
    drone_point.child_frame_id = target_frame;
    current_velocity = 0.f;
    time_multiplier = 1.f;  // for forwards or backwards. Forward uses 1.0, Rewind uses -1.0
    transition_dt = target_dt;
    new_rewind = false;

    // init variables
    prev_vtp_time = 0;
}

void TrajectoryControlNode::timer_callback() {
    static rclcpp::Time prev_execute_time = this->get_clock()->now();
    static rclcpp::Time curr_execute_time = this->get_clock()->now();
    curr_execute_time = this->get_clock()->now();
    double execute_elapsed = (curr_execute_time - prev_execute_time).seconds();
    prev_execute_time = curr_execute_time;

    if (!got_odom) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Waiting to receive odometry.");
        return;
    }

    if (time_multiplier > 1.f) time_multiplier = 1.f;
    if (std::isnan(time_multiplier) || !std::isfinite(time_multiplier)) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "time_multiplier was an invalid value: " << time_multiplier);
        time_multiplier = 1.f;
    }

    // figure out what duration into the trajectory we are
    rclcpp::Time now = this->get_clock()->now();
    tf2::Vector3 robot_point = tflib::to_tf(odom.pose.pose.position);
    double tracking_error =
        tflib::to_tf(virtual_tracking_point_odom.pose.position).distance(robot_point);
    std_msgs::msg::Float32 tracking_error_msg;
    tracking_error_msg.data = tracking_error;
    tracking_error_pub->publish(tracking_error_msg);

    // visualization
    markers.overwrite();
    markers
        .add_sphere(target_frame, now - rclcpp::Duration::from_seconds(0.3), robot_point.x(),
                    robot_point.y(), robot_point.z(), sphere_radius)
        .set_color(0.f, 0.f, 1.f, 0.7f);
    trajectory_vis_pub->publish(trajectory->get_markers(now - rclcpp::Duration::from_seconds(0.3),
                                                        "traj_controller", 1, 1, 0, 1, false, false,
                                                        traj_vis_thickness));

    // find closest point on entire trajectory
    Waypoint closest_wp(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    nav_msgs::msg::Odometry closest_odom;
    bool closest_valid = false;  // trajectory->get_closest_waypoint(robot_point, 0,
                                 // trajectory->get_duration(), &closest_wp);
    if (closest_valid)
        closest_valid = trajectory->get_odom(closest_wp.get_time(), &closest_point_odom, now);
    // else
    //   ROS_INFO("CLOSEST NOT VALID");
    if (closest_valid) closest_point_pub->publish(closest_point_odom);

    double current_virtual_ahead_time = 0.;

    if (trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::PAUSE) {
        if (time_multiplier >= 1.f &&
            tflib::to_tf(virtual_tracking_point_odom.twist.linear).length() >
                min_virtual_tracking_velocity) {
            if (new_rewind) RCLCPP_INFO(this->get_logger(), "YO");
            // find closest point within time interval
            Waypoint closest_ahead_wp(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            bool closest_ahead_valid = trajectory->get_closest_waypoint(
                robot_point, virtual_time, prev_vtp_time + look_ahead_time, &closest_ahead_wp);
            if (closest_ahead_valid) {
                virtual_time = closest_ahead_wp.get_time();

                // visualization
                markers
                    .add_sphere(target_frame, now - rclcpp::Duration::from_seconds(0.3),
                                closest_ahead_wp.get_x(), closest_ahead_wp.get_y(),
                                closest_ahead_wp.get_z(), 0.025f)
                    .set_color(0.f, 1.f, 0.f);

                Waypoint vtp_wp(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                Waypoint end_wp(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                bool vtp_valid = trajectory->get_waypoint_sphere_intersection(
                    virtual_time, search_ahead_factor * sphere_radius,
                    prev_vtp_time + look_ahead_time, robot_point, sphere_radius,
                    min_virtual_tracking_velocity, &vtp_wp, &end_wp);
                if (vtp_valid) current_virtual_ahead_time = vtp_wp.get_time() - virtual_time;

                // visualization
                if (vtp_valid)
                    markers
                        .add_sphere(target_frame, now - rclcpp::Duration::from_seconds(0.3),
                                    vtp_wp.get_x(), vtp_wp.get_y(), vtp_wp.get_z(), 0.025f)
                        .set_color(0.f, 1.f, 0.f);
                else
                    markers
                        .add_sphere(target_frame, now - rclcpp::Duration::from_seconds(0.3),
                                    closest_ahead_wp.get_x(), closest_ahead_wp.get_y(),
                                    closest_ahead_wp.get_z() + sphere_radius, 0.025f)
                        .set_color(1.f, 0.f, 0.f);
                markers
                    .add_sphere(target_frame, now - rclcpp::Duration::from_seconds(0.3),
                                end_wp.get_x(), end_wp.get_y(), end_wp.get_z(), 0.025f)
                    .set_color(0.f, 0.f, 1.f);
            } else
                RCLCPP_INFO(this->get_logger(), "AHEAD NOT VALID");
        } else {
            if (new_rewind) {
                float before = virtual_time;
                virtual_time = trajectory->get_skip_ahead_time(
                    virtual_time, rewind_skip_max_velocity, rewind_skip_max_distance);
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "SKIPPED from " << before << " to " << virtual_time << " ("
                                                   << (virtual_time - before) << ")");
                new_rewind = false;
            }
            virtual_time = std::min(trajectory->get_duration(),
                                    virtual_time + time_multiplier * execute_elapsed);
            if (time_multiplier < 1.f) {
                time_multiplier += transition_dt;
            }
        }
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::PAUSE) {
        // don't update virtual_time
    }
    //*/

    // get virtual waypoint
    Waypoint virtual_wp(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    bool virtual_valid =
        trajectory->get_waypoint(virtual_time + current_virtual_ahead_time, &virtual_wp);
    if (virtual_valid) {
        virtual_valid =
            trajectory->get_odom(virtual_wp.get_time(), &virtual_tracking_point_odom, now);

        // prevent oscillating between slow and normal mode
        if (tflib::to_tf(virtual_tracking_point_odom.twist.linear).length() <=
            min_virtual_tracking_velocity)
            virtual_time += current_virtual_ahead_time;

        float look_ahead_multiplier = 1.f;
        if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND)
            look_ahead_multiplier = -1.f;
        trajectory->get_odom(virtual_wp.get_time() + look_ahead_multiplier * look_ahead_time,
                             &look_ahead_point, now);
        trajectory->get_odom(virtual_wp.get_time(), &drone_point, now);
        actual_time = virtual_wp.get_time();
        prev_vtp_time = virtual_wp.get_time();
    } else {
        RCLCPP_INFO_STREAM_ONCE(
            this->get_logger(),
            "Virtual waypoint does not correspond to a waypoint along the "
            "reference trajectory. If no trajectory is available yet, this is okay.");
    }

    if (trajectory->get_num_waypoints() <= 3) {
        virtual_tracking_point_odom.twist.linear.x = 0;
        virtual_tracking_point_odom.twist.linear.y = 0;
        virtual_tracking_point_odom.twist.linear.z = 0;
    }
    if (tflib::to_tf(virtual_tracking_point_odom.twist.linear).length() <=
        min_virtual_tracking_velocity) {
        virtual_tracking_point_odom.twist.linear.x = 0;
        virtual_tracking_point_odom.twist.linear.y = 0;
        virtual_tracking_point_odom.twist.linear.z = 0;
        virtual_tracking_point_odom.acceleration.x = 0;
        virtual_tracking_point_odom.acceleration.y = 0;
        virtual_tracking_point_odom.acceleration.z = 0;
        virtual_tracking_point_odom.jerk.x = 0;
        virtual_tracking_point_odom.jerk.y = 0;
        virtual_tracking_point_odom.jerk.z = 0;
    }

    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE) {
        virtual_tracking_point_odom.pose = odom.pose.pose;
        virtual_tracking_point_odom.twist.linear.x = 0;
        virtual_tracking_point_odom.twist.linear.y = 0;
        virtual_tracking_point_odom.twist.linear.z = 0;
        virtual_tracking_point_odom.twist.angular.x = 0;
        virtual_tracking_point_odom.twist.angular.y = 0;
        virtual_tracking_point_odom.twist.angular.z = 0;
        virtual_tracking_point_odom.acceleration.x = 0;
        virtual_tracking_point_odom.acceleration.y = 0;
        virtual_tracking_point_odom.acceleration.z = 0;
        virtual_tracking_point_odom.jerk.x = 0;
        virtual_tracking_point_odom.jerk.y = 0;
        virtual_tracking_point_odom.jerk.z = 0;
        look_ahead_point = virtual_tracking_point_odom;
        drone_point = look_ahead_point;
    }

    virtual_tracking_point_odom.header.stamp = now;
    look_ahead_point.header.stamp = now;
    drone_point.header.stamp = now;

    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
        look_ahead_point.twist.linear.x *= -1.f;
        look_ahead_point.twist.linear.y *= -1.f;
        look_ahead_point.twist.linear.z *= -1.f;
    }

    if (time_multiplier >= -1.f && time_multiplier <= 1.f) {
        virtual_tracking_point_odom.twist.linear.x *= time_multiplier;
        virtual_tracking_point_odom.twist.linear.y *= time_multiplier;
        virtual_tracking_point_odom.twist.linear.z *= time_multiplier;
    }

    // When the tracking point reaches the end of the trajectory, the velocity gets set to zero
    if (virtual_wp.get_time() >= trajectory->get_duration() ||
        trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::PAUSE) {
        virtual_tracking_point_odom.twist.linear.x = 0;
        virtual_tracking_point_odom.twist.linear.y = 0;
        virtual_tracking_point_odom.twist.linear.z = 0;
        virtual_tracking_point_odom.twist.angular.x = 0;
        virtual_tracking_point_odom.twist.angular.y = 0;
        virtual_tracking_point_odom.twist.angular.z = 0;
        virtual_tracking_point_odom.acceleration.x = 0;
        virtual_tracking_point_odom.acceleration.y = 0;
        virtual_tracking_point_odom.acceleration.z = 0;
        virtual_tracking_point_odom.jerk.x = 0;
        virtual_tracking_point_odom.jerk.y = 0;
        virtual_tracking_point_odom.jerk.z = 0;

        look_ahead_point.twist.linear.x = 0;
        look_ahead_point.twist.linear.y = 0;
        look_ahead_point.twist.linear.z = 0;
        look_ahead_point.twist.angular.x = 0;
        look_ahead_point.twist.angular.y = 0;
        look_ahead_point.twist.angular.z = 0;
        look_ahead_point.acceleration.x = 0;
        look_ahead_point.acceleration.y = 0;
        look_ahead_point.acceleration.z = 0;
        look_ahead_point.jerk.x = 0;
        look_ahead_point.jerk.y = 0;
        look_ahead_point.jerk.z = 0;
        drone_point = look_ahead_point;
    }

    std_msgs::msg::Float32 velocity_msg;
    velocity_msg.data = sqrt(
        virtual_tracking_point_odom.twist.linear.x * virtual_tracking_point_odom.twist.linear.x +
        virtual_tracking_point_odom.twist.linear.y * virtual_tracking_point_odom.twist.linear.y +
        virtual_tracking_point_odom.twist.linear.z * virtual_tracking_point_odom.twist.linear.z);
    if (time_multiplier >= 1.f) current_velocity = velocity_msg.data;
    velocity_pub->publish(velocity_msg);
    tracking_point_pub->publish(virtual_tracking_point_odom);
    look_ahead_pub->publish(look_ahead_point);
    drone_point_pub->publish(drone_point);

    // create a tf for the tracking point odom
    tf2::Stamped<tf2::Transform> vtp_tf = tflib::to_tf(virtual_tracking_point_odom);
    geometry_msgs::msg::TransformStamped transform = tf2::toMsg(vtp_tf);
    transform.child_frame_id = tf_prefix + "tracking_point";
    geometry_msgs::msg::TransformStamped transform_stabilized =
        tf2::toMsg(tflib::get_stabilized(vtp_tf));
    transform_stabilized.child_frame_id = tf_prefix + "tracking_point_stabilized";
    tf_broadcaster->sendTransform(transform);
    tf_broadcaster->sendTransform(transform_stabilized);

    // create a tf for the look ahead odom
    tf2::Stamped<tf2::Transform> la_tf = tflib::to_tf(look_ahead_point);
    geometry_msgs::msg::TransformStamped look_ahead_transform = tf2::toMsg(la_tf);
    look_ahead_transform.child_frame_id = tf_prefix + "look_ahead_point";
    geometry_msgs::msg::TransformStamped look_ahead_transform_stabilized =
        tf2::toMsg(tflib::get_stabilized(la_tf));
    look_ahead_transform_stabilized.child_frame_id = tf_prefix + "look_ahead_point_stabilized";

    tf_broadcaster->sendTransform(look_ahead_transform);
    tf_broadcaster->sendTransform(look_ahead_transform_stabilized);

    // publish completion percentage
    std_msgs::msg::Float32 trajectory_completion_percentage;
    trajectory_completion_percentage.data = virtual_time / trajectory->get_duration() * 100.f;
    trajectory_completion_percentage_pub->publish(trajectory_completion_percentage);

    // publish current trajectory time
    std_msgs::msg::Float32 trajectory_time;
    trajectory_time.data = (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND)
                               ? trajectory->get_duration() - virtual_time
                               : virtual_time;
    trajectory_time_pub->publish(trajectory_time);

    // publish visualization
    debug_markers_pub->publish(markers.get_marker_array());

    new_rewind = false;
}

void TrajectoryControlNode::set_trajectory_mode(
    const std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Request> req,
    std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Response> res) {
    int prev_trajectory_mode = trajectory_mode;
    trajectory_mode = req->mode;

    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::PAUSE) {
        time_multiplier = 0.f;
    }
    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE) {
        trajectory->clear();
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::TRACK) {
        trajectory->clear();
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT) {
        if (prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::PAUSE &&
            prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::REWIND &&
            prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT) {
            trajectory->clear();
            virtual_time = 0;
            actual_time = 0;
        }
        if (prev_trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
            virtual_time = trajectory->get_duration() - actual_time;
            actual_time = virtual_time;
            prev_vtp_time = virtual_time;
            *trajectory = trajectory->get_reversed_trajectory();

            time_multiplier *= -1.f;
            if (current_velocity != 0.f)
                transition_dt = transition_velocity_scale / current_velocity * target_dt;
            else
                transition_dt = transition_velocity_scale / 0.1f * target_dt;
        }
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
        if (prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
            virtual_time = trajectory->get_duration() - actual_time;
            actual_time = virtual_time;
            prev_vtp_time = virtual_time;
            *trajectory = trajectory->get_reversed_trajectory();
            new_rewind = true;

            time_multiplier *= -1.f;
            if (current_velocity != 0.f)
                transition_dt = transition_velocity_scale / current_velocity * target_dt;
            else
                transition_dt = transition_velocity_scale / 0.1f * target_dt;
        }
    }

    res->success = true;
}

void TrajectoryControlNode::traj_seg_to_add_callback(
    const airstack_msgs::msg::TrajectoryXYZVYaw::SharedPtr traj) {
    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT)
        trajectory->merge(Trajectory(this, *traj), virtual_time);
}

void TrajectoryControlNode::traj_override_callback(
    const airstack_msgs::msg::TrajectoryXYZVYaw::SharedPtr traj) {
    virtual_time = 0;
    trajectory->clear();
    trajectory->merge(Trajectory(this, *traj));
}

void TrajectoryControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    // this->odom = odom;
    // An odometry's child frame is the frame of the robot, and the Twist (velocities) are put in
    // that frame. This line converts the odometry's position AND velocities to the target frame,
    // typically "world" or "map".
    if (tflib::transform_odometry(tf_buffer, *odom, target_frame, target_frame, &(this->odom))) {
        got_odom = true;
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed to transform odometry in frame "
                                                   << odom->header.frame_id << " to target frame "
                                                   << target_frame);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryControlNode>());
    rclcpp::shutdown();
    return 0;
}
