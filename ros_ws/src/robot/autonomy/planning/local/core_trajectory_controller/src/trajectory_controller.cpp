/*#include <base/BaseNode.h>
#include <trajectory_controller/Trajectory.h>
// #include <trajectory_controller/TrajectoryMode.h>
#include <geometry_msgs/msg/TwistStamped.h>
#include <nav_msgs/msg/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tflib/tflib.h>
#include <trajectory_library/trajectory_library.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
*/
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tflib/tflib.h>
#include <trajectory_library/trajectory_library.h>

#include <airstack_msgs/msg/trajectory.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
//===================================================================================
//----------------------------- Trajectory Control Node -----------------------------
//===================================================================================

class TrajectoryControlNode : public rclcpp::Node {
   private:
    // ros::Subscriber traj_sub, traj_track_sub, odom_sub;

    // ros::Publisher marker_vis_pub, segment_marker_vis_pub, tracking_point_pub, look_ahead_pub,
    //     trajectory_completion_percentage_pub, trajectory_time_pub, segment_pub,
    //     tracking_error_pub, velocity_pub;

    // SUBSCRIBERS
    rclcpp::Subscription<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_sub;
    rclcpp::Subscription<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_track_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    // PUBLISHERS
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_vis_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr segment_marker_vis_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tracking_point_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr look_ahead_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trajectory_completion_percentage_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trajectory_time_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tracking_error_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr segment_pub;

    // tf::TransformBroadcaster* broadcaster;
    // tf::TransformListener* listener;

    // LISTENERS AND BROADCATERS
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // ros::ServiceServer traj_style_srv, traj_mode_srv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr traj_style_srv;
    rclcpp::Service<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_srv;

    nav_msgs::msg::Odometry odom;
    bool got_odom;

    std::string tf_prefix;
    std::string target_frame;
    int trajectory_mode;
    Trajectory* trajectory;
    rclcpp::Time start_time;
    double prev_time;
    nav_msgs::msg::Odometry tracking_point, look_ahead_point;
    double tracking_point_distance_limit;
    double velocity_look_ahead_time;

   public:
    // TrajectoryControlNode();

    explicit TrajectoryControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    // virtual bool initialize();
    virtual bool execute();
    virtual ~TrajectoryControlNode();

    void traj_callback(const airstack_msgs::msg::TrajectoryXYZVYaw::ConstSharedPtr msg);
    void traj_track_callback(const airstack_msgs::msg::TrajectoryXYZVYaw::ConstSharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    // bool set_trajectory_style_service(std_srvs::SetBool::Request& req,
    //                                   std_srvs::SetBool::Response& res);
    // bool set_trajectory_mode(trajectory_controller::TrajectoryMode::Request& req,
    //                          trajectory_controller::TrajectoryMode::Response& res);

    bool set_trajectory_mode(
        const std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Request> request,
        std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Response> response);
    void set_trajectory_style_service(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    std::string mode;
    float velocity_target;
};

TrajectoryControlNode::TrajectoryControlNode(const rclcpp::NodeOptions& options)
    : Node("trajectory_control_node", options), trajectory(new Trajectory(target_frame)) {
    // ros::NodeHandle* nh = get_node_handle();
    // ros::NodeHandle* pnh = get_private_node_handle();

    // init params
    // tf_prefix = pnh->param("tf_prefix", std::string(""));
    // target_frame = pnh->param("target_frame", std::string("world"));
    // tracking_point_distance_limit = pnh->param("tracking_point_distance_limit", 0.5);
    // velocity_look_ahead_time = pnh->param("velocity_look_ahead_time", 0.0);
    // got_odom = false;
    tf_prefix = this->declare_parameter("tf_prefix", std::string(""));
    target_frame = this->declare_parameter("target_frame", std::string("world"));
    tracking_point_distance_limit = this->declare_parameter("tracking_point_distance_limit", 0.5);
    velocity_look_ahead_time = this->declare_parameter("velocity_look_ahead_time", 0.0);
    got_odom = false;

    trajectory_mode = airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE;  // TRACK;
    trajectory = new Trajectory(target_frame);
    // tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize tf_broadcaster
    // tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // init subscribers
    /*
    traj_sub = nh->subscribe("trajectory", 10, &TrajectoryControlNode::traj_callback, this);
    traj_track_sub =
        nh->subscribe("trajectory_track", 10, &TrajectoryControlNode::traj_track_callback, this);
    odom_sub = nh->subscribe("odometry", 10, &TrajectoryControlNode::odom_callback, this);
    */
    traj_sub = this->create_subscription<airstack_msgs::msg::TrajectoryXYZVYaw>(
        "trajectory", 10,
        std::bind(&TrajectoryControlNode::traj_callback, this, std::placeholders::_1));
    traj_track_sub = this->create_subscription<airstack_msgs::msg::TrajectoryXYZVYaw>(
        "trajectory_track", 10,
        std::bind(&TrajectoryControlNode::traj_track_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 10,
        std::bind(&TrajectoryControlNode::odom_callback, this, std::placeholders::_1));

    // init publishers
    /*
    segment_pub = nh->advertise<airstack_msgs::TrajectoryXYZVYaw>("trajectory_segment", 10);
    marker_vis_pub = nh->advertise<visualization_msgs::MarkerArray>("trajectory_vis", 10);
    segment_marker_vis_pub =
        nh->advertise<visualization_msgs::MarkerArray>("trajectory_segment_vis", 10);
    tracking_point_pub = nh->advertise<nav_msgs::Odometry>("tracking_point", 10);
    look_ahead_pub = nh->advertise<nav_msgs::Odometry>("look_ahead", 10);
    trajectory_completion_percentage_pub =
        nh->advertise<std_msgs::Float32>("trajectory_completion_percentage", 10);
    trajectory_time_pub = nh->advertise<std_msgs::Float32>("trajectory_time", 10);
    tracking_error_pub = nh->advertise<std_msgs::Float32>("tracking_error", 10);
    velocity_pub = nh->advertise<std_msgs::Float32>("tracking_point_velocity_magnitude", 1);
    broadcaster = new tf::TransformBroadcaster();
    listener = new tf::TransformListener();
    */
    segment_pub =
        this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_segment", 10);
    marker_vis_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_vis", 10);
    segment_marker_vis_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_segment_vis", 10);
    tracking_point_pub = this->create_publisher<nav_msgs::msg::Odometry>("tracking_point", 10);
    look_ahead_pub = this->create_publisher<nav_msgs::msg::Odometry>("look_ahead", 10);
    trajectory_completion_percentage_pub =
        this->create_publisher<std_msgs::msg::Float32>("trajectory_completion_percentage", 10);
    trajectory_time_pub = this->create_publisher<std_msgs::msg::Float32>("trajectory_time", 10);
    tracking_error_pub = this->create_publisher<std_msgs::msg::Float32>("tracking_error", 10);
    velocity_pub =
        this->create_publisher<std_msgs::msg::Float32>("tracking_point_velocity_magnitude", 1);

    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto node = std::make_shared<rclcpp::Node>("node_name");

    // Create tf2_ros::Buffer with required arguments
    tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    // tf_buffer->setCacheTime(std::chrono::seconds(10));

    // Create tf2_ros::TransformListener with the tf2_ros::Buffer
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Create tf2_ros::TransformBroadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    // Init services
    traj_mode_srv = this->create_service<airstack_msgs::srv::TrajectoryMode>(
        "set_trajectory_mode", std::bind(&TrajectoryControlNode::set_trajectory_mode, this,
                                         std::placeholders::_1, std::placeholders::_2));
    traj_style_srv = this->create_service<std_srvs::srv::SetBool>(
        "set_trajectory_style", std::bind(&TrajectoryControlNode::set_trajectory_style_service,
                                          this, std::placeholders::_1, std::placeholders::_2));

    // init services
    // traj_mode_srv = nh->advertiseService("set_trajectory_mode",
    //                                     &TrajectoryControlNode::set_trajectory_mode, this);

    prev_time = 0;

    // return true;
}

bool TrajectoryControlNode::execute() {
    if (got_odom) {
        // publish trajectory visualization
        marker_vis_pub->publish(trajectory->get_markers(1, 1, 0));

        // figure out what duration into the trajectory we are
        // ros::Time now = ros::Time::now();
        auto now = this->now();
        double current_time = (now - start_time).seconds();
        double time_past_end = current_time - trajectory->get_duration();

        double tracking_error = tflib::to_tf(tracking_point.pose.pose.position)
                                    .distance(tflib::to_tf(odom.pose.pose.position));
        std_msgs::msg::Float32 tracking_error_msg;
        tracking_error_msg.data = tracking_error;
        tracking_error_pub->publish(tracking_error_msg);

        if (time_past_end >= 0 &&
            trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
            std::chrono::duration<double> duration_past_end(time_past_end);
            start_time += rclcpp::Duration(duration_past_end);
            current_time = trajectory->get_duration();
        } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::PAUSE ||
                   tracking_error >= tracking_point_distance_limit) {
            double elapsed_time = current_time - prev_time;
            std::chrono::duration<double> duration_elapsed(elapsed_time);
            start_time += rclcpp::Duration(duration_elapsed);
            current_time = (now - start_time).seconds();
        }
        // TODO: zero tracking point velocity in rewind and pause modes
        else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND &&
                 current_time > 0) {
            // start_time += rclcpp::Duration(2.0 * (current_time - prev_time));
            // current_time = (now - start_time).seconds();
            double duration_seconds = 2.0 * (current_time - prev_time);
            rclcpp::Duration duration{std::chrono::duration<double>(duration_seconds)};

            start_time = start_time + duration;

            current_time = (now - start_time).seconds();
        }
        prev_time = current_time;

        // get tracking point and look ahead point. If they aren't valid use the drone's current
        // pose
        // monitor.tic("get_odom");
        bool valid = trajectory->get_odom(current_time, &tracking_point);
        // set the velocity of the tracking point based on the velocity from a different time
        if (valid && velocity_look_ahead_time != 0) {
            nav_msgs::msg::Odometry velocity_look_ahead_point;
            bool vel_valid = trajectory->get_odom(current_time + velocity_look_ahead_time,
                                                  &velocity_look_ahead_point);
            if (vel_valid) {
                tracking_point.twist = velocity_look_ahead_point.twist;
            }
        }

        // double elapsed = monitor.toc("get_odom");
        // ROS_INFO_STREAM("get_odom elapsed: " << elapsed);
        if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE) {
            // if(!valid){
            tracking_point = odom;
            tracking_point.twist.twist.linear.x = 0;
            tracking_point.twist.twist.linear.y = 0;
            tracking_point.twist.twist.linear.z = 0;
            tracking_point.twist.twist.angular.x = 0;
            tracking_point.twist.twist.angular.y = 0;
            tracking_point.twist.twist.angular.z = 0;
            look_ahead_point = tracking_point;
        }
        if (valid) {
            trajectory->get_odom(current_time + 1.0, &look_ahead_point);

            tf2::Vector3 tp = tflib::to_tf(tracking_point.pose.pose.position);
            double start;
            bool success = trajectory->get_trajectory_distance_at_closest_point(tp, &start);
            if (success) {
                Trajectory sub_trajectory =
                    trajectory->get_subtrajectory_distance(start, start + 1000.);
                segment_marker_vis_pub->publish(sub_trajectory.get_markers(0, 1, 1));
                airstack_msgs::msg::TrajectoryXYZVYaw segment_msg =
                    sub_trajectory.get_TrajectoryXYZVYaw();
                segment_msg.header.stamp = tracking_point.header.stamp;
                segment_pub->publish(segment_msg);
            }
        }

        tracking_point.header.stamp = now;
        look_ahead_point.header.stamp = now;

        // TODO: decide whether or not this is a good idea
        // When the tracking point reaches the end of the trajectory, the velocity gets set to zero
        if (time_past_end >= 0 ||
            trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::PAUSE ||
            trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
            tracking_point.twist.twist.linear.x = 0;
            tracking_point.twist.twist.linear.y = 0;
            tracking_point.twist.twist.linear.z = 0;
            tracking_point.twist.twist.angular.x = 0;
            tracking_point.twist.twist.angular.y = 0;
            tracking_point.twist.twist.angular.z = 0;

            look_ahead_point.twist.twist.linear.x = 0;
            look_ahead_point.twist.twist.linear.y = 0;
            look_ahead_point.twist.twist.linear.z = 0;
            look_ahead_point.twist.twist.angular.x = 0;
            look_ahead_point.twist.twist.angular.y = 0;
            look_ahead_point.twist.twist.angular.z = 0;
        }

        std_msgs::msg::Float32 velocity_msg;
        velocity_msg.data =
            sqrt(tracking_point.twist.twist.linear.x * tracking_point.twist.twist.linear.x +
                 tracking_point.twist.twist.linear.y * tracking_point.twist.twist.linear.y +
                 tracking_point.twist.twist.linear.z * tracking_point.twist.twist.linear.z);
        velocity_pub->publish(velocity_msg);
        tracking_point_pub->publish(tracking_point);
        look_ahead_pub->publish(look_ahead_point);

        // create a tf for the tracking point odom
        /*
        tf::StampedTransform transform =
            tflib::to_tf(tracking_point, tf_prefix + "/tracking_point");
        tf::StampedTransform transform_stabilized = tflib::get_stabilized(transform);
        transform_stabilized.child_frame_id_ = tf_prefix + "/tracking_point_stabilized";
        tf_broadcaster->sendTransform(transform);
        tf_broadcaster->sendTransform(transform_stabilized);

        // create a tf for the look ahead odom
        tf::StampedTransform look_ahead_transform =
            tflib::to_tf(look_ahead_point, tf_prefix + "/look_ahead_point");
        tf::StampedTransform look_ahead_transform_stabilized =
            tflib::get_stabilized(look_ahead_transform);
        look_ahead_transform_stabilized.child_frame_id_ =
            tf_prefix + "/look_ahead_point_stabilized";

        tf_broadcaster->sendTransform(look_ahead_transform);
        tf_broadcaster->sendTransform(look_ahead_transform_stabilized);
        */
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = tf_prefix + "/base_link";  // Base frame_id
        transform.child_frame_id = tf_prefix + "/tracking_point";
        transform.transform.translation.x = tracking_point.pose.pose.position.x;
        transform.transform.translation.y = tracking_point.pose.pose.position.y;
        transform.transform.translation.z = tracking_point.pose.pose.position.z;
        transform.transform.rotation = tracking_point.pose.pose.orientation;

        auto transform_stabilized = transform;  // Apply any stabilization logic here
        transform_stabilized.child_frame_id = tf_prefix + "/tracking_point_stabilized";

        tf_broadcaster->sendTransform(transform);
        tf_broadcaster->sendTransform(transform_stabilized);

        // Create a transform for the look ahead odom
        geometry_msgs::msg::TransformStamped look_ahead_transform;
        look_ahead_transform.header.stamp = now;
        look_ahead_transform.header.frame_id = tf_prefix + "/base_link";  // Base frame_id
        look_ahead_transform.child_frame_id = tf_prefix + "/look_ahead_point";
        look_ahead_transform.transform.translation.x = look_ahead_point.pose.pose.position.x;
        look_ahead_transform.transform.translation.y = look_ahead_point.pose.pose.position.y;
        look_ahead_transform.transform.translation.z = look_ahead_point.pose.pose.position.z;
        look_ahead_transform.transform.rotation = look_ahead_point.pose.pose.orientation;

        auto look_ahead_transform_stabilized =
            look_ahead_transform;  // Apply any stabilization logic here
        look_ahead_transform_stabilized.child_frame_id = tf_prefix + "/look_ahead_point_stabilized";
        tf_broadcaster->sendTransform(look_ahead_transform);
        tf_broadcaster->sendTransform(look_ahead_transform_stabilized);

        // publish completion percentage
        std_msgs::msg::Float32 trajectory_completion_percentage;
        trajectory_completion_percentage.data = current_time / trajectory->get_duration() * 100.f;
        trajectory_completion_percentage_pub->publish(trajectory_completion_percentage);

        // publish current trajectory time
        std_msgs::msg::Float32 trajectory_time;
        trajectory_time.data = current_time;
        trajectory_time_pub->publish(trajectory_time);
    }

    return true;
}

bool TrajectoryControlNode::set_trajectory_mode(
    std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Request> request,
    std::shared_ptr<airstack_msgs::srv::TrajectoryMode::Response> response) {
    int prev_trajectory_mode = trajectory_mode;
    trajectory_mode = request->mode;

    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::PAUSE) {
    }
    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE) {
        trajectory->clear();
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::TRACK) {
        trajectory->clear();
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::SEGMENT) {
        if (prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::PAUSE &&
            prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::REWIND &&
            prev_trajectory_mode != airstack_msgs::srv::TrajectoryMode::Request::SEGMENT) {
            trajectory->clear();
            start_time = this->now();
        }
    } else if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::REWIND) {
    }

    response->success = true;
    return true;
}

void TrajectoryControlNode::traj_callback(
    airstack_msgs::msg::TrajectoryXYZVYaw::ConstSharedPtr traj) {
    if (trajectory_mode == airstack_msgs::srv::TrajectoryMode::Request::SEGMENT)
        trajectory->merge(Trajectory(*traj));
}

void TrajectoryControlNode::traj_track_callback(
    airstack_msgs::msg::TrajectoryXYZVYaw::ConstSharedPtr traj) {
    start_time = this->now();
    trajectory->clear();
    trajectory->merge(Trajectory(*traj));
}

void TrajectoryControlNode::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    // this->odom = odom;
    tf2_ros::Buffer tf_buffer(this->get_clock());  // Initialize with a clock
    tf2_ros::TransformListener tf_listener(tf_buffer);

    if (tflib::transform_odometry(&tf_buffer, *odom, target_frame, target_frame, &(this->odom)))
        got_odom = true;
}

TrajectoryControlNode::~TrajectoryControlNode() {}

// BaseNode* BaseNode::get() {
//     TrajectoryControlNode* trajectory_control_node = new TrajectoryControlNode();
//     return trajectory_control_node;
// }
int main(int argc, char* argv[]) {
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create an instance of the TrajectoryControlNode
    auto node = std::make_shared<TrajectoryControlNode>(rclcpp::NodeOptions());

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Clean up and shut down ROS 2
    rclcpp::shutdown();
    return 0;
}