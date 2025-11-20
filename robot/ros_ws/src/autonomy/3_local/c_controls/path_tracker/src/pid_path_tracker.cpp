#include "rclcpp/rclcpp.hpp"

#include <deque>

#include <airstack_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <airstack_common/tflib.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pid_controller_msgs/msg/pid_info.hpp>
#include <std_msgs/msg/empty.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

class PID
{
public:
    rclcpp::Node *node;
    rclcpp::Time time_prev;

    pid_controller_msgs::msg::PIDInfo info;

    rclcpp::Publisher<pid_controller_msgs::msg::PIDInfo>::SharedPtr info_pub;

public:
    PID(rclcpp::Node *node, std::string name);
    void set_target(double target);
    double get_control(double measured, double ff_value = 0.);
    void reset_integrator();
};

PID::PID(rclcpp::Node *node, std::string name)
    : node(node), time_prev(0)
{
    airstack::dynamic_param(node, name + "_p", 1., &info.p);
    airstack::dynamic_param(node, name + "_i", 0., &info.i);
    airstack::dynamic_param(node, name + "_d", 0., &info.d);
    airstack::dynamic_param(node, name + "_ff", 0., &info.ff);

    airstack::dynamic_param(node, name + "_d_alpha", 0., &info.d_alpha);

    airstack::dynamic_param(node, name + "_min", -100000., &info.min);
    airstack::dynamic_param(node, name + "_max", 100000., &info.max);
    airstack::dynamic_param(node, name + "_constant", 0., &info.constant);

    info_pub = node->create_publisher<pid_controller_msgs::msg::PIDInfo>(name + "_pid_info", 1);
}

void PID::set_target(double target)
{
    info.target = target;
}

double PID::get_control(double measured, double ff_value)
{
    info.measured = measured;
    info.ff_value = ff_value;

    rclcpp::Time time_now = node->now();
    info.header.stamp = time_now;
    if (time_prev.seconds() == 0)
    {
        time_prev = time_now;
        info_pub->publish(info);
        return 0.;
    }
    info.dt = (time_now - time_prev).seconds();
    time_prev = time_now;
    if (info.dt <= 0.)
    {
        info_pub->publish(info);
        return info.control;
    }

    double error_prev = info.error;
    info.error = info.target - info.measured;
    info.p_component = info.p * info.error;

    info.integral += info.error * info.dt;
    if (info.i == 0)
        info.integral = 0;
    info.i_component = info.i * info.integral;
    if (info.i_component > info.max)
    {
        info.integral = info.max / info.i;
        info.i_component = info.i * info.integral;
    }
    else if (info.i_component < info.min)
    {
        info.integral = info.min / info.i;
        info.i_component = info.i * info.integral;
    }

    info.derivative = info.d_alpha * info.derivative + (1. - info.d_alpha) * (info.error - error_prev) / info.dt;
    info.d_component = info.d * info.derivative;

    info.ff_component = info.ff * info.ff_value;

    info.control = info.p_component + info.i_component + info.d_component + info.ff_component + info.constant;
    info.control = std::max(info.min, std::min(info.max, info.control));

    info_pub->publish(info);
    return info.control;
}

void PID::reset_integrator()
{
    info.integral = 0.;
}

class PIDPathTrackerNode : public rclcpp::Node
{
private:
    // params
    std::string target_frame;
    double max_roll_pitch; // rad

    PID x_pid, y_pid, z_pid, vx_pid, vy_pid, vz_pid;

    // state
    bool got_odometry;
    nav_msgs::msg::Odometry odometry;

    // trajectory-related
    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_sub_;
    std_msgs::msg::Header traj_header_;

    std::deque<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> traj_points_;
    std::deque<rclcpp::Duration> traj_waiting_times_;
    rclcpp::TimerBase::SharedPtr traj_timer_;

    // other subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_integrators_sub;
    tf2_ros::Buffer *tf_buffer;
    tf2_ros::TransformListener *tf_listener;

    // publishers
    rclcpp::Publisher<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr command_pub;

public:
    PIDPathTrackerNode()
        : Node("pid_traj_tracker"),
          x_pid(this, "x"),
          y_pid(this, "y"),
          z_pid(this, "z"),
          vx_pid(this, "vx"),
          vy_pid(this, "vy"),
          vz_pid(this, "vz"),
          got_odometry(false)
    {
        target_frame = airstack::get_param(this, "target_frame", std::string("base_link"));
        max_roll_pitch = airstack::get_param(this, "max_roll_pitch", 10.) * M_PI / 180.;

        // odom sub
        odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 1, std::bind(&PIDPathTrackerNode::odometry_callback, this, std::placeholders::_1));

        // trajectory subscriber
        traj_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "trajectory", 1, std::bind(&PIDPathTrackerNode::trajectory_callback, this, std::placeholders::_1));

        reset_integrators_sub = this->create_subscription<std_msgs::msg::Empty>(
            "reset_integrators", 1, std::bind(&PIDPathTrackerNode::reset_integrators_callback, this, std::placeholders::_1));

        tf_buffer = new tf2_ros::Buffer(this->get_clock());
        tf_listener = new tf2_ros::TransformListener(*tf_buffer);

        command_pub = this->create_publisher<mav_msgs::msg::RollPitchYawrateThrust>("command", 1);
    }

    void trajectory_callback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty())
        {
            RCLCPP_WARN(get_logger(), "Got trajectory with 0 points.");
            return;
        }

        // stop the old timer
        if (traj_timer_)
        {
            traj_timer_->cancel();
            traj_timer_.reset();
        }

        traj_points_.clear();
        traj_waiting_times_.clear();
        traj_header_ = msg->header;

        const auto &pts = msg->points;
        const std::size_t n = pts.size();

        // Save points and dt
        traj_points_.push_back(pts.front());
        for (std::size_t i = 1; i < n; ++i)
        {
            traj_points_.push_back(pts[i]);

            rclcpp::Duration dt_i(pts[i].time_from_start);
            rclcpp::Duration dt_prev(pts[i - 1].time_from_start);
            traj_waiting_times_.push_back(dt_i - dt_prev);
        }

        RCLCPP_INFO(get_logger(), "Received trajectory with %zu points.", n);

        // output first point control immediately
        if (!traj_points_.empty())
        {
            auto first_pt = traj_points_.front();
            traj_points_.pop_front();

            airstack_msgs::msg::Odometry tp_msg;
            if (convertTrajPointToTrackingPoint(first_pt, traj_header_, tp_msg))
            {
                processTrackingPoint(tp_msg);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Failed to convert first traj point to tracking point.");
            }
        }

        schedule_next_timer();
    }

    void schedule_next_timer()
    {
        if (traj_waiting_times_.empty() || traj_points_.empty())
        {
            traj_timer_.reset();
            return;
        }

        rclcpp::Duration wait = traj_waiting_times_.front();
        traj_waiting_times_.pop_front();

        if (wait <= rclcpp::Duration(0, 0))
        {
            wait = rclcpp::Duration(0, 1); // 1ns
        }

        auto period = wait.to_chrono<std::chrono::nanoseconds>();

        traj_timer_ = this->create_wall_timer(period, std::bind(&PIDPathTrackerNode::traj_timer_callback, this));
    }

    void traj_timer_callback()
    {
        if (traj_timer_)
        {
            traj_timer_->cancel();
            traj_timer_.reset();
        }

        if (traj_points_.empty())
        {
            return;
        }

        auto pt = traj_points_.front();
        traj_points_.pop_front();

        airstack_msgs::msg::Odometry tp_msg;
        if (convertTrajPointToTrackingPoint(pt, traj_header_, tp_msg))
        {
            processTrackingPoint(tp_msg);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to convert traj point to tracking point in timer.");
        }

        schedule_next_timer();
    }

    bool convertTrajPointToTrackingPoint(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &pt,
                                         const std_msgs::msg::Header &traj_header,
                                         airstack_msgs::msg::Odometry &tp_out)
    {
        if (pt.transforms.empty())
        {
            return false;
        }

        const auto &tf = pt.transforms[0];

        tp_out.header.frame_id = traj_header.frame_id;
        tp_out.header.stamp = this->now();

        tp_out.child_frame_id = traj_header.frame_id;

        tp_out.pose.position.x = tf.translation.x;
        tp_out.pose.position.y = tf.translation.y;
        tp_out.pose.position.z = tf.translation.z;
        tp_out.pose.orientation = tf.rotation;

        if (!pt.velocities.empty())
        {
            const auto &vel = pt.velocities[0];
            tp_out.twist.linear = vel.linear;
            tp_out.twist.angular = vel.angular;
        }
        else
        {
            tp_out.twist.linear.x = tp_out.twist.linear.y = tp_out.twist.linear.z = 0.0;
            tp_out.twist.angular.x = tp_out.twist.angular.y = tp_out.twist.angular.z = 0.0;
        }

        tp_out.acceleration.x = tp_out.acceleration.y = tp_out.acceleration.z = 0.0;
        tp_out.jerk.x = tp_out.jerk.y = tp_out.jerk.z = 0.0;

        return true;
    }

    void processTrackingPoint(const airstack_msgs::msg::Odometry &msg)
    {
        if (!got_odometry)
            return;

        airstack_msgs::msg::Odometry tp;
        nav_msgs::msg::Odometry odom;

        airstack_msgs::msg::Odometry temp = msg;
        bool s1 = tflib::transform_odometry(tf_buffer, temp, target_frame, target_frame, &tp,
                                            rclcpp::Duration::from_seconds(0.1));

        if (!s1)
        {
            RCLCPP_ERROR(get_logger(), "Transform tracking point failed.");
            return;
        }

        bool s2 = tflib::transform_odometry(tf_buffer, odometry, target_frame, target_frame, &odom,
                                            rclcpp::Duration::from_seconds(0.1));

        if (!s2)
        {
            RCLCPP_ERROR(get_logger(), "Transform odometry failed.");
            return;
        }

        tf2::Vector3 tp_pos = tflib::to_tf(tp.pose.position);
        tf2::Vector3 tp_vel = tflib::to_tf(tp.twist.linear);
        tf2::Vector3 odom_pos = tflib::to_tf(odom.pose.pose.position);
        tf2::Vector3 odom_vel = tflib::to_tf(odom.twist.twist.linear);

        // ----- outer loop: pos -> vel -----
        x_pid.set_target(tp_pos.x());
        y_pid.set_target(tp_pos.y());
        z_pid.set_target(tp_pos.z());

        double vx = x_pid.get_control(odom_pos.x());
        double vy = y_pid.get_control(odom_pos.y());
        double vz = z_pid.get_control(odom_pos.z());

        // ----- inner loop: vel -> attitude/thrust -----
        vx_pid.set_target(vx);
        vy_pid.set_target(vy);
        vz_pid.set_target(vz);

        double roll = -vy_pid.get_control(odom_vel.y());
        double pitch = vx_pid.get_control(odom_vel.x());
        double thrust = vz_pid.get_control(odom_vel.z());

        mav_msgs::msg::RollPitchYawrateThrust command;
        command.header.frame_id = target_frame;
        command.header.stamp = tp.header.stamp;

        command.roll = roll;
        command.pitch = pitch;

        double _, yaw;
        tf2::Matrix3x3(tflib::to_tf(msg.pose.orientation)).getRPY(_, _, yaw);
        command.yaw_rate = yaw;

        command.thrust.z = thrust;

        command_pub->publish(command);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        got_odometry = true;
        odometry = *msg;
    }

    void reset_integrators_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "RESET INTEGRATORS");

        x_pid.reset_integrator();
        y_pid.reset_integrator();
        z_pid.reset_integrator();
        vx_pid.reset_integrator();
        vy_pid.reset_integrator();
        vz_pid.reset_integrator();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPathTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
