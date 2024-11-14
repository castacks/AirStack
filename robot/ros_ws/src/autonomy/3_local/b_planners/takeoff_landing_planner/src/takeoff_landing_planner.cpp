#include <takeoff_landing_planner/takeoff_landing_planner.hpp>

TakeoffLandingPlanner::TakeoffLandingPlanner() : rclcpp::Node("takeoff_landing_planner") {
    // init parameters
    takeoff_height = airstack::get_param(this, "takeoff_height", 0.5);
    high_takeoff_height = airstack::get_param(this, "high_takeoff_height", 1.2);
    takeoff_landing_velocity = airstack::get_param(this, "takeoff_landing_velocity", 0.3);
    takeoff_acceptance_distance = airstack::get_param(this, "takeoff_acceptance_distance", 0.1);
    takeoff_acceptance_time = airstack::get_param(this, "takeoff_acceptance_time", 2.0);
    landing_stationary_distance = airstack::get_param(this, "landing_stationary_distance", 0.02);
    landing_acceptance_time = airstack::get_param(this, "landing_acceptance_time", 5.0);
    landing_tracking_point_ahead_time =
        airstack::get_param(this, "landing_tracking_point_ahead_time", 5.0);
    takeoff_path_roll = airstack::get_param(this, "takeoff_path_roll", 0.) * M_PI / 180.;
    takeoff_path_pitch = airstack::get_param(this, "takeoff_path_pitch", 0.) * M_PI / 180.;
    takeoff_path_relative_to_orientation =
        airstack::get_param(this, "takeoff_path_relative_to_orientation", false);

    // init subscribers
    tf_buffer = new tf2_ros::Buffer(this->get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);
    completion_percentage_sub = this->create_subscription<std_msgs::msg::Float32>(
        "trajectory_completion_percentage", 1,
        std::bind(&TakeoffLandingPlanner::completion_percentage_callback, this,
                  std::placeholders::_1));
    tracking_point_sub = this->create_subscription<airstack_msgs::msg::Odometry>(
        "tracking_point", 1,
        std::bind(&TakeoffLandingPlanner::tracking_point_callback, this, std::placeholders::_1));
    robot_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 1,
        std::bind(&TakeoffLandingPlanner::robot_odom_callback, this, std::placeholders::_1));
    ekf_active_sub = this->create_subscription<std_msgs::msg::Bool>(
        "ekf_active", 1,
        std::bind(&TakeoffLandingPlanner::ekf_active_callback, this, std::placeholders::_1));
    high_takeoff_sub = this->create_subscription<std_msgs::msg::Bool>(
        "high_takeoff", 1,
        std::bind(&TakeoffLandingPlanner::high_takeoff_callback, this, std::placeholders::_1));

    // init publishers
    traj_override_pub =
        this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_override", 1);
    takeoff_state_pub = this->create_publisher<std_msgs::msg::String>("takeoff_state", 1);
    landing_state_pub = this->create_publisher<std_msgs::msg::String>("landing_state", 1);

    // init services
    command_server = this->create_service<airstack_msgs::srv::TakeoffLandingCommand>(
        "set_takeoff_landing_command",
        std::bind(&TakeoffLandingPlanner::set_takeoff_landing_command, this, std::placeholders::_1,
                  std::placeholders::_2));

    // init variables
    got_completion_percentage = false;
    is_tracking_point_received = false;

    // TODO set this back to false and remove robot_odom initialization
    got_robot_odom = false;

    // track_mode_srv.request.mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK;
    high_takeoff = false;
    takeoff_traj_gen =
        new TakeoffTrajectory(takeoff_height, takeoff_landing_velocity, takeoff_path_roll,
                              takeoff_path_pitch, takeoff_path_relative_to_orientation);
    high_takeoff_traj_gen =
        new TakeoffTrajectory(high_takeoff_height, takeoff_landing_velocity, takeoff_path_roll,
                              takeoff_path_pitch, takeoff_path_relative_to_orientation);
    // TODO: this landing point is hardcoded. it should be parameterized
    landing_traj_gen = new TakeoffTrajectory(-10000., takeoff_landing_velocity);
    current_command = airstack_msgs::srv::TakeoffLandingCommand::Request::NONE;

    completion_percentage = 0.f;
    takeoff_is_newly_active = true;
    land_is_newly_active = true;
    takeoff_distance_check = false;
    ekf_active = false;

    
    // timers
    timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1. / 20.),
                                 std::bind(&TakeoffLandingPlanner::timer_callback, this));
}

void TakeoffLandingPlanner::timer_callback() {
  RCLCPP_INFO_STREAM(this->get_logger(),
		     "conditions: " << got_completion_percentage << " " << is_tracking_point_received << " " << got_robot_odom);
    if (!got_completion_percentage || !is_tracking_point_received || !got_robot_odom) return;

    std_msgs::msg::String takeoff_state, landing_state;
    takeoff_state.data = landing_state.data = "NONE";

    // takeoff
    if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::TAKEOFF) {
        takeoff_state.data = "TAKING_OFF";

        if (completion_percentage >= 100.f) {
            // get distance between tracking point and robot odom
            float distance = std::numeric_limits<float>::max();
            try {
                tf2::Stamped<tf2::Transform> transform;
                geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                    tracking_point_odom.header.frame_id, robot_odom.header.frame_id,
                    robot_odom.header.stamp, rclcpp::Duration::from_seconds(0.1));
                tf2::fromMsg(t, transform);

                tf2::Vector3 tracking_point_position =
                    tflib::to_tf(tracking_point_odom.pose.position);
                tf2::Vector3 robot_odom_position =
                    transform * tflib::to_tf(robot_odom.pose.pose.position);

                distance = tracking_point_position.distance(robot_odom_position);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "TransformException in TakeoffMonitor: " << ex.what());
            }

            // ROS_INFO_STREAM("distance: " << distance);

            // check if distance meets the takeoff acceptance threshold
            if (distance <= takeoff_acceptance_distance) {
                if (!takeoff_distance_check) {
                    takeoff_distance_check = true;
                    takeoff_acceptance_start = robot_odom.header.stamp;
                }

                // check if the distance threshold has been met for the required amount of time
                // ROS_INFO_STREAM("duration: " << (robot_odom.header.stamp -
                // takeoff_acceptance_start).toSec() << " / " << takeoff_acceptance_time);
                if ((rclcpp::Time(robot_odom.header.stamp) - takeoff_acceptance_start).seconds() >=
                        takeoff_acceptance_time ||
                    ekf_active) {
                    takeoff_state.data = "COMPLETE";
                }
            } else
                takeoff_distance_check = false;
        }
    }

    // land
    if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::LAND) {
        landing_state.data = "LANDING";

        // check if the robot has been moving
        if (robot_odoms.size() > 2) {
            nav_msgs::msg::Odometry initial_odom = robot_odoms.front();
            float time_diff =
                (rclcpp::Time(robot_odom.header.stamp) - rclcpp::Time(initial_odom.header.stamp))
                    .seconds();

            // landing_detected = time_diff > landing_acceptance_time;
            bool time_check = time_diff > landing_acceptance_time;
            while (!robot_odoms.empty() && (rclcpp::Time(robot_odom.header.stamp) -
                                            rclcpp::Time(robot_odoms.front().header.stamp))
                                                   .seconds() > landing_acceptance_time) {
                initial_odom = robot_odoms.front();
                robot_odoms.pop_front();
            }
            robot_odoms.push_front(initial_odom);

            bool distance_check = false;
            for (auto it = robot_odoms.begin(); time_check && it != robot_odoms.end(); it++) {
                float distance = tflib::to_tf(robot_odoms.begin()->pose.pose.position)
                                     .distance(tflib::to_tf(it->pose.pose.position));
                distance_check = distance <= landing_stationary_distance;
                if (!distance_check) {
                    // ROS_INFO_STREAM("distance: " << distance);
                    break;
                }
            }

            float z_distance = 100000.f;
            try {
                tf2::Stamped<tf2::Transform> transform;
                geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                    tracking_point_odom.header.frame_id, robot_odom.header.frame_id,
                    robot_odom.header.stamp, rclcpp::Duration::from_seconds(0.1));
                tf2::fromMsg(t, transform);
                tf2::Vector3 tracking_point_position =
                    tflib::to_tf(tracking_point_odom.pose.position);
                tf2::Vector3 robot_odom_position =
                    transform * tflib::to_tf(robot_odom.pose.pose.position);

                z_distance = robot_odom_position.z() - tracking_point_position.z();
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "TransformException in TakeoffMonitor landing tf lookup: " << ex.what());
            }
            bool tracking_point_check =
                (z_distance / takeoff_landing_velocity) > landing_tracking_point_ahead_time;
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "landing: " << z_distance << " " << (z_distance / takeoff_landing_velocity) << " "
                            << landing_tracking_point_ahead_time << " " << tracking_point_check);

            // ROS_INFO_STREAM("LANDING CHECK: " << time_diff << " " << time_check << " " <<
            // distance_check);

            if (time_check && (distance_check || tracking_point_check))
                landing_state.data = "COMPLETE";
        }
    }

    takeoff_state_pub->publish(takeoff_state);
    landing_state_pub->publish(landing_state);
}

// callbacks
void TakeoffLandingPlanner::completion_percentage_callback(std_msgs::msg::Float32::SharedPtr msg) {
    got_completion_percentage = true;
    completion_percentage = msg->data;
}

void TakeoffLandingPlanner::tracking_point_callback(airstack_msgs::msg::Odometry::SharedPtr msg) {
    is_tracking_point_received = true;
    tracking_point_odom = *msg;
}

void TakeoffLandingPlanner::robot_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    got_robot_odom = true;
    robot_odom = *msg;

    if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::LAND)
        robot_odoms.push_back(*msg);
}

void TakeoffLandingPlanner::ekf_active_callback(std_msgs::msg::Bool::SharedPtr msg) {
    ekf_active = msg->data;
}

void TakeoffLandingPlanner::high_takeoff_callback(std_msgs::msg::Bool::SharedPtr msg) {
    high_takeoff = msg->data;
}

void TakeoffLandingPlanner::set_takeoff_landing_command(
    const airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr request,
    airstack_msgs::srv::TakeoffLandingCommand::Response::SharedPtr response) {
    current_command = request->command;

    if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::NONE) {
    } else if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::TAKEOFF) {
        // put the trajectory controller into track mode
        // traj_mode_client.call(track_mode_srv);
        // publish a takeoff trajectory
        RCLCPP_INFO_STREAM(get_logger(), "takeofflanding 1");

        airstack_msgs::msg::Odometry takeoff_starting_point = tracking_point_odom;
        if (got_robot_odom && takeoff_path_relative_to_orientation)
            takeoff_starting_point.pose.orientation = robot_odom.pose.pose.orientation;

        if (high_takeoff) {
            RCLCPP_INFO_STREAM(get_logger(), "takeofflanding hightakeoff");
            traj_override_pub->publish(
                high_takeoff_traj_gen->get_trajectory(takeoff_starting_point));
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "takeofflanding lowtakeoff");
            traj_override_pub->publish(takeoff_traj_gen->get_trajectory(takeoff_starting_point));
        }
    } else if (current_command == airstack_msgs::srv::TakeoffLandingCommand::Request::LAND) {
        robot_odoms.clear();
        // put the trajectory controller into track mode
        // traj_mode_client.call(track_mode_srv);
        // publish a landing trajectory
        traj_override_pub->publish(landing_traj_gen->get_trajectory(tracking_point_odom));
    }
    RCLCPP_INFO_STREAM(get_logger(), "takeofflanding end");

    response->accepted = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TakeoffLandingPlanner>());
    rclcpp::shutdown();
    return 0;
}
