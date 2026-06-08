// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file modalai_interface.cpp
 * @brief RobotInterface plugin for ModalAI VOXL2.
 *
 * The VOXL2 runs PX4 and exposes it via the MicroDDS bridge, so all command
 * topics (/fmu/in/*) are identical to px4_interface.  The only difference is
 * the odometry source: instead of consuming PX4's out/vehicle_odometry this
 * node subscribes to VOXL's qvio topic (geometry_msgs/PoseStamped, local NED)
 * and converts it to nav_msgs/Odometry in ENU/FLU for the rest of AirStack.
 *
 * ## Frame conventions
 *
 * AirStack / ROS:  ENU world, FLU body
 * PX4 / VOXL qvio: NED world, FRD body
 *
 * Position  (ENU ↔ NED):  N=ENU_y, E=ENU_x, D=-ENU_z
 * Body-rate (FLU ↔ FRD):  roll=same, pitch=-pitch, yaw=-yaw
 * Quaternion: q_px4 = q_NED_ENU ⊗ q_ros ⊗ q_FLU_FRD
 *
 * ## Topic mapping (relative — push /fmu namespace from the launch file)
 *
 * Commands → PX4 (via MicroDDS):
 *   in/offboard_control_mode
 *   in/trajectory_setpoint
 *   in/vehicle_attitude_setpoint
 *   in/vehicle_rates_setpoint
 *   in/vehicle_command
 *   in/vehicle_visual_odometry
 *
 * Feedback ← PX4:
 *   out/vehicle_status       → arm state, nav state
 *
 * Odometry ← VOXL:
 *   qvio                     → geometry_msgs/PoseStamped (local NED)
 *                              remapped in launch to the actual VOXL topic
 */

#include <chrono>
#include <cmath>
#include <array>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/msg/rate_thrust.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <robot_interface/robot_interface.hpp>

namespace modalai_interface
{

// ---------------------------------------------------------------------------
// Quaternion helpers (identical to px4_interface)
// ---------------------------------------------------------------------------

static inline std::array<double, 4> qmul(double aw, double ax, double ay, double az,
                                          double bw, double bx, double by, double bz)
{
    return {aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw};
}

static constexpr double kSqrt2Inv = 0.70710678118654752;

static inline std::array<double, 4> enu_flu_to_ned_frd(double qw, double qx,
                                                         double qy, double qz)
{
    auto tmp = qmul(0.0, kSqrt2Inv, kSqrt2Inv, 0.0, qw, qx, qy, qz);
    return qmul(tmp[0], tmp[1], tmp[2], tmp[3], 0.0, 1.0, 0.0, 0.0);
}

static inline std::array<double, 4> ned_frd_to_enu_flu(double qw, double qx,
                                                         double qy, double qz)
{
    auto tmp = qmul(0.0, -kSqrt2Inv, -kSqrt2Inv, 0.0, qw, qx, qy, qz);
    return qmul(tmp[0], tmp[1], tmp[2], tmp[3], 0.0, 1.0, 0.0, 0.0);
}

// ---------------------------------------------------------------------------
// ModalAIInterface
// ---------------------------------------------------------------------------

class ModalAIInterface : public robot_interface::RobotInterface
{
public:
    ModalAIInterface() : RobotInterface("modalai_interface")
    {
        // VOXL2 MicroDDS uses the same QoS as PX4 standalone.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();

        // ---- Publishers → PX4 via MicroDDS ----
        offboard_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "in/offboard_control_mode", qos);

        trajectory_sp_pub_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "in/trajectory_setpoint", qos);

        attitude_sp_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
                "in/vehicle_attitude_setpoint", qos);

        rates_sp_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
                "in/vehicle_rates_setpoint", qos);

        vehicle_cmd_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "in/vehicle_command", qos);

        visual_odom_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleOdometry>(
                "in/vehicle_visual_odometry", qos);

        // ---- Subscriber ← PX4: arm/mode state ----
        vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "out/vehicle_status", qos,
                std::bind(&ModalAIInterface::on_vehicle_status, this,
                          std::placeholders::_1));

        // ---- Subscriber ← VOXL: qvio odometry ----
        // Remap "qvio" in the launch file to the actual VOXL topic, e.g.
        // /voxl/qvio or /qvio depending on your voxl-mpa-to-ros2 config.
        qvio_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "qvio", 10,
                std::bind(&ModalAIInterface::on_qvio, this,
                          std::placeholders::_1));

        // ---- AirStack odometry output ----
        odometry_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

        // ---- Optional: AirStack visual odometry → PX4 fusion ----
        visual_odom_in_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "visual_odometry_in", 10,
                std::bind(&ModalAIInterface::on_visual_odometry_in, this,
                          std::placeholders::_1));

        // Offboard heartbeat at 10 Hz (PX4 requires ≥ 2 Hz).
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ModalAIInterface::publish_offboard_heartbeat, this));

        RCLCPP_INFO(this->get_logger(),
                    "ModalAIInterface initialised (VOXL2 + PX4 MicroDDS)");
    }

    virtual ~ModalAIInterface() = default;

    // -----------------------------------------------------------------------
    // RobotInterface: control-command callbacks
    // (Frame conversions identical to px4_interface — VOXL2 still runs PX4)
    // -----------------------------------------------------------------------

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::POSITION);
        publish_offboard_heartbeat();

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp    = now_us();
        sp.position[0]  = static_cast<float>(cmd->pose.position.y);
        sp.position[1]  = static_cast<float>(cmd->pose.position.x);
        sp.position[2]  = static_cast<float>(-cmd->pose.position.z);
        sp.velocity[0]  = sp.velocity[1] = sp.velocity[2] = NAN;
        sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;

        tf2::Quaternion q_enu(cmd->pose.orientation.x, cmd->pose.orientation.y,
                               cmd->pose.orientation.z, cmd->pose.orientation.w);
        double roll{}, pitch{}, yaw_enu{};
        tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw_enu);
        sp.yaw      = static_cast<float>(M_PI_2 - yaw_enu);
        sp.yawspeed = NAN;

        trajectory_sp_pub_->publish(sp);
    }

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::VELOCITY);
        publish_offboard_heartbeat();

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp    = now_us();
        sp.position[0]  = sp.position[1] = sp.position[2] = NAN;
        sp.velocity[0]  = static_cast<float>(cmd->twist.linear.y);
        sp.velocity[1]  = static_cast<float>(cmd->twist.linear.x);
        sp.velocity[2]  = static_cast<float>(-cmd->twist.linear.z);
        sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
        sp.yaw      = NAN;
        sp.yawspeed = static_cast<float>(-cmd->twist.angular.z);

        trajectory_sp_pub_->publish(sp);
    }

    void attitude_thrust_callback(
        const mav_msgs::msg::AttitudeThrust::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::ATTITUDE);
        publish_offboard_heartbeat();

        px4_msgs::msg::VehicleAttitudeSetpoint sp{};
        sp.timestamp = now_us();

        auto q = enu_flu_to_ned_frd(cmd->attitude.w, cmd->attitude.x,
                                     cmd->attitude.y, cmd->attitude.z);
        sp.q_d[0] = static_cast<float>(q[0]);
        sp.q_d[1] = static_cast<float>(q[1]);
        sp.q_d[2] = static_cast<float>(q[2]);
        sp.q_d[3] = static_cast<float>(q[3]);

        sp.thrust_body[0] = 0.0f;
        sp.thrust_body[1] = 0.0f;
        sp.thrust_body[2] = static_cast<float>(-cmd->thrust.z);

        attitude_sp_pub_->publish(sp);
    }

    void rate_thrust_callback(
        const mav_msgs::msg::RateThrust::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::BODY_RATE);
        publish_offboard_heartbeat();

        px4_msgs::msg::VehicleRatesSetpoint sp{};
        sp.timestamp      = now_us();
        sp.roll           = static_cast<float>(cmd->angular_rates.x);
        sp.pitch          = static_cast<float>(-cmd->angular_rates.y);
        sp.yaw            = static_cast<float>(-cmd->angular_rates.z);
        sp.thrust_body[0] = 0.0f;
        sp.thrust_body[1] = 0.0f;
        sp.thrust_body[2] = static_cast<float>(-cmd->thrust.z);

        rates_sp_pub_->publish(sp);
    }

    void roll_pitch_yawrate_thrust_callback(
        const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::BODY_RATE);
        publish_offboard_heartbeat();

        px4_msgs::msg::VehicleRatesSetpoint sp{};
        sp.timestamp      = now_us();
        sp.roll           = static_cast<float>(cmd->roll);
        sp.pitch          = static_cast<float>(-cmd->pitch);
        sp.yaw            = static_cast<float>(-cmd->yaw_rate);
        sp.thrust_body[0] = 0.0f;
        sp.thrust_body[1] = 0.0f;
        sp.thrust_body[2] = static_cast<float>(-cmd->thrust.z);

        rates_sp_pub_->publish(sp);
    }

    // -----------------------------------------------------------------------
    // RobotInterface: command functions
    // -----------------------------------------------------------------------

    bool request_control() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f, 6.0f);
        RCLCPP_INFO(this->get_logger(), "Offboard mode requested.");
        return true;
    }

    bool arm() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
        return true;
    }

    bool disarm() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0f);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent.");
        return true;
    }

    bool is_armed() override
    {
        return status_received_ &&
               vehicle_status_.arming_state ==
                   px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    }

    bool has_control() override
    {
        return status_received_ &&
               vehicle_status_.nav_state ==
                   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    bool takeoff() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
        RCLCPP_INFO(this->get_logger(), "Takeoff command sent.");
        return true;
    }

    bool land() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Land command sent.");
        return true;
    }

private:
    // -----------------------------------------------------------------------
    // Internal types
    // -----------------------------------------------------------------------

    enum class ControlMode : uint8_t
    {
        NONE      = 0,
        POSITION  = 1,
        VELOCITY  = 2,
        ATTITUDE  = 3,
        BODY_RATE = 4,
    };

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------

    ControlMode control_mode_{ControlMode::NONE};
    px4_msgs::msg::VehicleStatus vehicle_status_{};
    bool status_received_{false};

    // Publishers → PX4
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;

    // Subscribers ← PX4
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // Subscriber ← VOXL qvio
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qvio_sub_;

    // AirStack I/O
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_in_sub_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    uint64_t now_us()
    {
        return static_cast<uint64_t>(
            this->get_clock()->now().nanoseconds() / 1000ULL);
    }

    void set_control_mode(ControlMode mode) { control_mode_ = mode; }

    void publish_offboard_heartbeat()
    {
        if (control_mode_ == ControlMode::NONE) return;

        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp    = now_us();
        msg.position     = (control_mode_ == ControlMode::POSITION);
        msg.velocity     = (control_mode_ == ControlMode::VELOCITY);
        msg.acceleration = false;
        msg.attitude     = (control_mode_ == ControlMode::ATTITUDE);
        msg.body_rate    = (control_mode_ == ControlMode::BODY_RATE);
        offboard_mode_pub_->publish(msg);

        // PX4 also requires a matching trajectory_setpoint stream.
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = now_us();
        sp.position[0] = sp.position[1] = sp.position[2] = NAN;
        sp.velocity[0] = sp.velocity[1] = sp.velocity[2] = 0.0f;
        sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
        sp.yaw      = NAN;
        sp.yawspeed = 0.0f;
        trajectory_sp_pub_->publish(sp);
    }

    void send_vehicle_command(uint32_t command,
                               float p1 = 0.f, float p2 = 0.f,
                               float p3 = 0.f, float p4 = 0.f,
                               double p5 = 0.0, double p6 = 0.0,
                               float p7 = 0.f)
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp        = now_us();
        cmd.command          = command;
        cmd.param1           = p1;
        cmd.param2           = p2;
        cmd.param3           = p3;
        cmd.param4           = p4;
        cmd.param5           = p5;
        cmd.param6           = p6;
        cmd.param7           = p7;
        cmd.target_system    = 0;  // broadcast: PX4 sets MAV_SYS_ID = vehicle_id+1
        cmd.target_component = 0;
        cmd.source_system    = 1;
        cmd.source_component = 1;
        cmd.from_external    = true;
        vehicle_cmd_pub_->publish(cmd);
    }

    // -----------------------------------------------------------------------
    // Callbacks
    // -----------------------------------------------------------------------

    void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        vehicle_status_  = *msg;
        status_received_ = true;
    }

    /**
     * @brief Convert VOXL qvio PoseStamped (NED/FRD) → nav_msgs/Odometry (ENU/FLU).
     *
     * voxl-mpa-to-ros2 publishes qvio as geometry_msgs/PoseStamped in the
     * local NED frame with FRD body orientation.  Velocity is not available
     * from this topic so the twist fields are left as zero.
     */
    void on_qvio(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = msg->header.stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id  = "base_link";

        // Position: NED → ENU
        odom.pose.pose.position.x = static_cast<double>(msg->pose.position.y);
        odom.pose.pose.position.y = static_cast<double>(msg->pose.position.x);
        odom.pose.pose.position.z = static_cast<double>(-msg->pose.position.z);

        // Orientation: FRD→NED → FLU→ENU
        auto q = ned_frd_to_enu_flu(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        odom.pose.pose.orientation.w = q[0];
        odom.pose.pose.orientation.x = q[1];
        odom.pose.pose.orientation.y = q[2];
        odom.pose.pose.orientation.z = q[3];

        odometry_pub_->publish(odom);
    }

    /**
     * @brief Forward AirStack ENU/FLU odometry → PX4 visual-odometry fusion.
     *
     * Enable external vision fusion in PX4 via EKF2_EV_CTRL.
     */
    void on_visual_odometry_in(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        px4_msgs::msg::VehicleOdometry vio{};
        vio.timestamp        = now_us();
        vio.timestamp_sample = now_us();

        // Position: ENU → NED
        vio.position[0] = static_cast<float>(msg->pose.pose.position.y);
        vio.position[1] = static_cast<float>(msg->pose.pose.position.x);
        vio.position[2] = static_cast<float>(-msg->pose.pose.position.z);

        // Orientation: FLU→ENU → FRD→NED
        auto q = enu_flu_to_ned_frd(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        vio.q[0] = static_cast<float>(q[0]);
        vio.q[1] = static_cast<float>(q[1]);
        vio.q[2] = static_cast<float>(q[2]);
        vio.q[3] = static_cast<float>(q[3]);

        // Velocity: ENU → NED
        vio.velocity[0] = static_cast<float>(msg->twist.twist.linear.y);
        vio.velocity[1] = static_cast<float>(msg->twist.twist.linear.x);
        vio.velocity[2] = static_cast<float>(-msg->twist.twist.linear.z);

        vio.pose_frame     = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        vio.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        vio.position_variance[0]    = static_cast<float>(msg->pose.covariance[0]);
        vio.position_variance[1]    = static_cast<float>(msg->pose.covariance[7]);
        vio.position_variance[2]    = static_cast<float>(msg->pose.covariance[14]);
        vio.orientation_variance[0] = static_cast<float>(msg->pose.covariance[21]);
        vio.orientation_variance[1] = static_cast<float>(msg->pose.covariance[28]);
        vio.orientation_variance[2] = static_cast<float>(msg->pose.covariance[35]);

        visual_odom_pub_->publish(vio);
    }
};

}  // namespace modalai_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(modalai_interface::ModalAIInterface, robot_interface::RobotInterface)
