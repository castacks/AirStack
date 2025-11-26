#include "rclcpp/rclcpp.hpp"

#include <deque>
#include <algorithm>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pid_controller_msgs/msg/pid_info.hpp>
#include <std_msgs/msg/empty.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

constexpr double PI = 3.14159265358979323846;

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

private:
    template <typename T>
    void declare_and_get(const std::string &param_name,
                         const T &default_value,
                         T &out);
};

PID::PID(rclcpp::Node *node, std::string name)
    : node(node),
      time_prev(0)
{
    declare_and_get(name + "_p", 1.0, info.p);
    declare_and_get(name + "_i", 0.0, info.i);
    declare_and_get(name + "_d", 0.0, info.d);
    declare_and_get(name + "_ff", 0.0, info.ff);
    declare_and_get(name + "_d_alpha", 0.0, info.d_alpha);
    declare_and_get(name + "_min", -1000.0, info.min);
    declare_and_get(name + "_max", 1000.0, info.max);
    declare_and_get(name + "_constant", 0.0, info.constant);

    // initialize
    info.error = 0;
    info.integral = 0;
    info.derivative = 0;
    info.control = 0;
    info.p_component = 0;
    info.i_component = 0;
    info.d_component = 0;
    info.ff_component = 0;
    info.dt = 0;

    info_pub = node->create_publisher<pid_controller_msgs::msg::PIDInfo>(name + "_pid_info", 1);
}

template <typename T>
void PID::declare_and_get(const std::string &param_name,
                          const T &default_value,
                          T &out)
{
    node->declare_parameter<T>(param_name, default_value);
    node->get_parameter(param_name, out);
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
    if (info.i == 0.0)
    {
        // reset integral when k_i = 0
        info.integral = 0;
    }
    info.i_component = info.i * info.integral;
    if (info.i != 0.0)
    {
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
    }

    info.derivative = info.d_alpha * info.derivative + (1. - info.d_alpha) * (info.error - error_prev) / info.dt;
    info.d_component = info.d * info.derivative;

    info.ff_component = info.ff * info.ff_value;

    info.control = info.p_component + info.i_component + info.d_component + info.ff_component + info.constant;
    info.control = std::clamp(info.control, info.min, info.max);

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

    PID x_pid, y_pid, z_pid;

    double yaw_p_;
    double yaw_rate_max_;

    // state
    bool got_odometry;
    nav_msgs::msg::Odometry odometry;

    bool have_ref_;
    nav_msgs::msg::Odometry current_ref_;

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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tracking_point_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub;

public:
    PIDPathTrackerNode()
        : Node("pid_traj_tracker"),
          x_pid(this, "x"),
          y_pid(this, "y"),
          z_pid(this, "z"),
          got_odometry(false),
          have_ref_(false)
    {
        this->declare_parameter("target_frame", "base_link");
        this->get_parameter("target_frame", target_frame);

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

        command_pub = this->create_publisher<geometry_msgs::msg::Twist>("command", 1);
        tracking_point_pub = this->create_publisher<nav_msgs::msg::Odometry>("/tracking_point_vis", 1);

        this->declare_parameter("yaw_p", 1.0);
        this->declare_parameter("yaw_rate_max", 1.0);

        this->get_parameter("yaw_p", yaw_p_);
        this->get_parameter("yaw_rate_max", yaw_rate_max_);
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

        // RCLCPP_INFO(get_logger(), "Received trajectory with %zu points.", n);

        // output first point control immediately
        if (!traj_points_.empty())
        {
            auto first_pt = traj_points_.front();
            traj_points_.pop_front();

            nav_msgs::msg::Odometry tp_msg;
            if (convertTrajPointToTrackingPoint(first_pt, traj_header_, tp_msg))
            {
                setTrackingPoint(tp_msg);
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

        rclcpp::Duration wait_time = traj_waiting_times_.front();
        traj_waiting_times_.pop_front();

        if (wait_time <= rclcpp::Duration(0, 0))
        {
            wait_time = rclcpp::Duration(0, 1); // 1ns
        }

        traj_timer_ = rclcpp::create_timer(this->get_node_base_interface(),
                                           this->get_node_timers_interface(),
                                           this->get_clock(),
                                           wait_time,
                                           std::bind(&PIDPathTrackerNode::traj_timer_callback, this));
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

        nav_msgs::msg::Odometry tp_msg;
        if (convertTrajPointToTrackingPoint(pt, traj_header_, tp_msg))
        {
            setTrackingPoint(tp_msg);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to convert traj point to tracking point in timer.");
        }

        schedule_next_timer();
    }

    bool convertTrajPointToTrackingPoint(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &pt,
                                         const std_msgs::msg::Header &traj_header,
                                         nav_msgs::msg::Odometry &tp_out)
    {
        if (pt.transforms.empty())
        {
            return false;
        }

        const auto &tf = pt.transforms[0];

        tp_out.header.frame_id = traj_header.frame_id;
        tp_out.header.stamp = this->now();

        tp_out.child_frame_id = traj_header.frame_id;

        tp_out.pose.pose.position.x = tf.translation.x;
        tp_out.pose.pose.position.y = tf.translation.y;
        tp_out.pose.pose.position.z = tf.translation.z;
        tp_out.pose.pose.orientation = tf.rotation;

        if (!pt.velocities.empty())
        {
            const auto &vel = pt.velocities[0];
            tp_out.twist.twist.linear = vel.linear;
            tp_out.twist.twist.angular = vel.angular;
        }
        else
        {
            tp_out.twist.twist.linear.x = tp_out.twist.twist.linear.y = tp_out.twist.twist.linear.z = 0.0;
            tp_out.twist.twist.angular.x = tp_out.twist.twist.angular.y = tp_out.twist.twist.angular.z = 0.0;
        }

        return true;
    }

    void setTrackingPoint(const nav_msgs::msg::Odometry &msg)
    {
        current_ref_ = msg;

        have_ref_ = true;

        tracking_point_pub->publish(msg);

        x_pid.set_target(msg.pose.pose.position.x);
        y_pid.set_target(msg.pose.pose.position.y);
        z_pid.set_target(msg.pose.pose.position.z);
    }

    double getYawFromQuat(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double angleDiff(double target, double current)
    {
        double e = target - current;
        while (e > PI)
        {
            e -= 2.0 * PI;
        }
        while (e < -PI)
        {
            e += 2.0 * PI;
        }
        return e;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        got_odometry = true;
        odometry = *msg;

        if (!have_ref_)
        {
            return;
        }

        const double tp_x = current_ref_.pose.pose.position.x;
        const double tp_y = current_ref_.pose.pose.position.y;
        const double tp_z = current_ref_.pose.pose.position.z;

        const double odom_x = odometry.pose.pose.position.x;
        const double odom_y = odometry.pose.pose.position.y;
        const double odom_z = odometry.pose.pose.position.z;

        x_pid.set_target(tp_x);
        y_pid.set_target(tp_y);
        z_pid.set_target(tp_z);

        const double vx_des_W = x_pid.get_control(odom_x);
        const double vy_des_W = y_pid.get_control(odom_y);
        const double vz_des_W = z_pid.get_control(odom_z);

        // command vel in body frame

        tf2::Quaternion q(odometry.pose.pose.orientation.x,
                          odometry.pose.pose.orientation.y,
                          odometry.pose.pose.orientation.z,
                          odometry.pose.pose.orientation.w);
        tf2::Matrix3x3 R_WB(q);

        tf2::Vector3 v_des_W(vx_des_W, vy_des_W, vz_des_W);
        tf2::Vector3 v_des_B = R_WB.transpose() * v_des_W;

        // yaw command same in world/body frame
        const double yaw_ref = getYawFromQuat(current_ref_.pose.pose.orientation);
        const double yaw_cur = getYawFromQuat(odometry.pose.pose.orientation);
        const double yaw_err = angleDiff(yaw_ref, yaw_cur);
        double yaw_rate = yaw_p_ * yaw_err;

        double yaw_rate_cmd = std::clamp(yaw_rate, -yaw_rate_max_, yaw_rate_max_);

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = v_des_B.x();
        cmd.linear.y = v_des_B.y();
        cmd.linear.z = v_des_B.z();

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = yaw_rate_cmd;

        // cmd now in world frame

        command_pub->publish(cmd);
    }

    void reset_integrators_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "RESET INTEGRATORS");

        x_pid.reset_integrator();
        y_pid.reset_integrator();
        z_pid.reset_integrator();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPathTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
