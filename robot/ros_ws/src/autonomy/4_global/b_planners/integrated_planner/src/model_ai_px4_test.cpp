#include <cmath>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class SquareTrajUnitTest : public rclcpp::Node
{
public:
    SquareTrajUnitTest() : Node("square_traj_unit_test_cpp")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lk(mtx_);
                latest_odom_ = *msg;
            });

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
            "/cmd_trajectory", rclcpp::QoS(10));

        srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/make_square",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
            {
                this->handle_make_square(resp);
            });

        turn_square_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/turn_square",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
            {
                this->handle_turn_square(resp);
            });

        hover_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/offboard_hover",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
            {
                this->handle_offboard_hover(resp);
            });

        RCLCPP_INFO(this->get_logger(), "Ready: sub /odom, pub /square_traj, srv /make_square");
    }

private:
    void handle_make_square(std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
        nav_msgs::msg::Odometry odom;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!latest_odom_.has_value())
            {
                resp->success = false;
                resp->message = "No /odom yet";
                return;
            }
            odom = *latest_odom_;
        }

        double x = odom.pose.pose.position.x;
        double y = odom.pose.pose.position.y;
        double z = odom.pose.pose.position.z;

        double x0 = x;
        double y0 = y;
        double z0 = z;

        const double step = 0.1;                                      // 10cm
        const double side = 1.0;                                      // 1m
        const double speed = 0.2;                                     // 0.2 m/s
        const int n_step = static_cast<int>(std::round(side / step)); // 10
        const double dt = step / speed;                               // 1s per point

        trajectory_msgs::msg::MultiDOFJointTrajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        // yaw = 0 => quat (0,0,0,1)
        const double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

        auto add_point = [&](double px, double py, double pz,
                             double vx, double vy, double vz,
                             double t_sec)
        {
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt;

            geometry_msgs::msg::Transform tf;
            tf.translation.x = px;
            tf.translation.y = py;
            tf.translation.z = pz;
            tf.rotation.x = qx;
            tf.rotation.y = qy;
            tf.rotation.z = qz;
            tf.rotation.w = qw;
            pt.transforms.push_back(tf);

            geometry_msgs::msg::Twist tw;
            tw.linear.x = vx;
            tw.linear.y = vy;
            tw.linear.z = vz;
            tw.angular.x = 0.0;
            tw.angular.y = 0.0;
            tw.angular.z = 0.0;
            pt.velocities.push_back(tw);

            pt.time_from_start = rclcpp::Duration::from_seconds(t_sec);

            traj.points.push_back(std::move(pt));
        };

        double t = 0.0;
        add_point(x, y, z,
                  0.0, 0.0, 0.0,
                  t); // start

        // Edges: +x, +y, -x, -y
        struct Edge
        {
            double dx, dy, vx, vy;
        };
        const Edge edges[4] = {
            {+step, 0.0, +speed, 0.0}, // forward
            {0.0, +step, 0.0, +speed}, // right
            {-step, 0.0, -speed, 0.0}, // back
            {0.0, -step, 0.0, -speed}, // left
        };

        for (const auto &e : edges)
        {
            for (int i = 0; i < n_step; ++i)
            {
                x += e.dx;
                y += e.dy;
                t += dt;
                add_point(x, y, z, e.vx, e.vy, 0.0, t);
            }
        }

        traj.points.pop_back();
        add_point(x0, y0, z0, 0.0, 0.0, 0.0, t);

        traj_pub_->publish(traj);

        resp->success = true;
        resp->message = "Published square trajectory: " + std::to_string(traj.points.size()) + " points";
    }

    void handle_turn_square(std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
        nav_msgs::msg::Odometry odom;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!latest_odom_.has_value())
            {
                resp->success = false;
                resp->message = "No /odom yet";
                return;
            }
            odom = *latest_odom_;
        }

        double x = odom.pose.pose.position.x;
        double y = odom.pose.pose.position.y;
        double z = odom.pose.pose.position.z;

        double x0 = x;
        double y0 = y;
        double z0 = z;

        const double step = 0.1;                                      // 10cm
        const double side = 1.0;                                      // 1m
        const double speed = 0.2;                                     // 0.2 m/s
        const int n_step = static_cast<int>(std::round(side / step)); // 10
        const double dt = step / speed;                               // 1s per point

        trajectory_msgs::msg::MultiDOFJointTrajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        // yaw = 0 => quat (0,0,0,1)

        auto add_point = [&](double px, double py, double pz,
                             double vx, double vy, double vz,
                             double qx, double qy, double qz, double qw,
                             double t_sec)
        {
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt;

            geometry_msgs::msg::Transform tf;
            tf.translation.x = px;
            tf.translation.y = py;
            tf.translation.z = pz;
            tf.rotation.x = qx;
            tf.rotation.y = qy;
            tf.rotation.z = qz;
            tf.rotation.w = qw;
            pt.transforms.push_back(tf);

            geometry_msgs::msg::Twist tw;
            tw.linear.x = vx;
            tw.linear.y = vy;
            tw.linear.z = vz;
            tw.angular.x = 0.0;
            tw.angular.y = 0.0;
            tw.angular.z = 0.0;
            pt.velocities.push_back(tw);

            pt.time_from_start = rclcpp::Duration::from_seconds(t_sec);

            traj.points.push_back(std::move(pt));
        };

        double t = 0.0;
        add_point(x, y, z,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0,
                  t); // start

        // Edges: +x, +y, -x, -y
        struct Edge
        {
            double dx, dy, vx, vy, qx, qy, qz, qw;
        };
        const Edge edges[4] = {
            {+step, 0.0, +speed, 0.0, 0.0, 0.0, 0.0, 1.0},          // forward
            {0.0, +step, 0.0, +speed, 0, 0, 0.7071068, 0.7071068},   // right, +90 degree in ned
            {-step, 0.0, -speed, 0.0, 0, 0, 1, 0}, // back, 180 degree in ned
            {0.0, -step, 0.0, -speed, 0, 0, -0.7071068, 0.7071068},   // left, -90 degree in ned
        };

        for (const auto &e : edges)
        {
            for (int i = 0; i < n_step; ++i)
            {
                x += e.dx;
                y += e.dy;
                t += dt;
                add_point(x, y, z,
                          e.vx, e.vy, 0.0,
                          e.qx, e.qy, e.qz, e.qw,
                          t);
            }
        }

        traj.points.pop_back();
        add_point(x0, y0, z0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0,
                  t);

        traj_pub_->publish(traj);

        resp->success = true;
        resp->message = "Published square trajectory: " + std::to_string(traj.points.size()) + " points";
    }

    void handle_offboard_hover(std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
        nav_msgs::msg::Odometry odom;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!latest_odom_.has_value())
            {
                resp->success = false;
                resp->message = "No /odom yet";
                return;
            }
            odom = *latest_odom_;
        }

        double x = odom.pose.pose.position.x;
        double y = odom.pose.pose.position.y;
        double z = odom.pose.pose.position.z;

        double x0 = x;
        double y0 = y;
        double z0 = z;

        trajectory_msgs::msg::MultiDOFJointTrajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        // yaw = 0 => quat (0,0,0,1)
        const double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

        auto add_point = [&](double px, double py, double pz,
                             double vx, double vy, double vz,
                             double t_sec)
        {
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt;

            geometry_msgs::msg::Transform tf;
            tf.translation.x = px;
            tf.translation.y = py;
            tf.translation.z = pz;
            tf.rotation.x = qx;
            tf.rotation.y = qy;
            tf.rotation.z = qz;
            tf.rotation.w = qw;
            pt.transforms.push_back(tf);

            geometry_msgs::msg::Twist tw;
            tw.linear.x = vx;
            tw.linear.y = vy;
            tw.linear.z = vz;
            tw.angular.x = 0.0;
            tw.angular.y = 0.0;
            tw.angular.z = 0.0;
            pt.velocities.push_back(tw);

            pt.time_from_start = rclcpp::Duration::from_seconds(t_sec);

            traj.points.push_back(std::move(pt));
        };

        double t = 0.0;

        add_point(x, y, z,
                  0.0, 0.0, 0.0,
                  t); // start

        add_point(x, y, z,
                  0.0, 0.0, 0.0,
                  t + 1.0); // end

        traj_pub_->publish(traj);

        resp->success = true;
        resp->message = "offboard hovering";
    }

    std::mutex mtx_;
    std::optional<nav_msgs::msg::Odometry> latest_odom_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn_square_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hover_srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareTrajUnitTest>());
    rclcpp::shutdown();
    return 0;
}
