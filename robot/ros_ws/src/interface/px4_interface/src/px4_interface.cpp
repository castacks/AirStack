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
 * @file px4_interface.cpp
 * @author AirLab @ CMU
 * @brief RobotInterface implementation that interfaces directly with PX4 via
 *        uXRCE-DDS ROS topics (no MAVROS).
 *
 * ## Frame conventions
 *
 * AirStack / ROS uses:
 *   - World frame: ENU  (x = East, y = North, z = Up)
 *   - Body frame:  FLU  (x = Forward, y = Left, z = Up)
 *
 * PX4 uXRCE-DDS uses:
 *   - World frame: NED  (x = North, y = East, z = Down)
 *   - Body frame:  FRD  (x = Forward, y = Right, z = Down)
 *
 * Position / velocity conversion (ENU <-> NED):
 *   NED_x = ENU_y,  NED_y = ENU_x,  NED_z = -ENU_z
 *
 * Body-rate conversion (FLU <-> FRD):
 *   FRD_roll = FLU_roll,  FRD_pitch = -FLU_pitch,  FRD_yaw = -FLU_yaw
 *
 * Attitude quaternion conversion (q_FLU_ENU <-> q_FRD_NED):
 *   q_px4 = q_NED_ENU ⊗ q_ros ⊗ q_FLU_FRD
 *   q_ros = q_ENU_NED ⊗ q_px4 ⊗ q_FRD_FLU
 * where
 *   q_NED_ENU  = [w=0,  x=1/√2, y=1/√2, z=0]   (180° around axis (1,1,0)/√2)
 *   q_FLU_FRD  = [w=0,  x=1,    y=0,    z=0]   (180° around body-x)
 *   q_ENU_NED  = conj(q_NED_ENU) — but since w=0 this equals -q_NED_ENU,
 *                which represents the same rotation; implementation uses the
 *                product formula with negated x,y directly.
 *
 * ## Topic mapping (no /fmu/ prefix — push that namespace in the launch file)
 *
 * Commands → PX4:
 *   in/offboard_control_mode      ← heartbeat, mode selection
 *   in/trajectory_setpoint        ← position / velocity setpoint
 *   in/vehicle_attitude_setpoint  ← attitude setpoint
 *   in/vehicle_rates_setpoint     ← body-rate setpoint
 *   in/vehicle_command            ← arm / disarm / set-mode / takeoff / land
 *   in/vehicle_visual_odometry    ← visual / mocap odometry fusion input
 *
 * Feedback ← PX4:
 *   out/vehicle_status    → arm state, nav state
 *   out/vehicle_odometry  → odometry (republished as nav_msgs/Odometry in ENU)
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

namespace px4_interface
{

// ---------------------------------------------------------------------------
// Quaternion helpers (all quaternions as [w, x, y, z])
// ---------------------------------------------------------------------------

/// Multiply two quaternions: result = a ⊗ b
static inline std::array<double, 4> qmul(double aw, double ax, double ay, double az,
                                          double bw, double bx, double by, double bz)
{
    return {aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw};
}

// Fixed frame-conversion quaternions (compile-time constants).
//   q_NED_ENU = [w=0, x=1/√2, y=1/√2, z=0]
//   q_FLU_FRD = [w=0, x=1,    y=0,    z=0]   (same as q_FRD_FLU, self-inverse)
static constexpr double kSqrt2Inv = 0.70710678118654752;  // 1/√2

/// Convert an AirStack quaternion (FLU→ENU) to a PX4 quaternion (FRD→NED).
/// q_px4 = q_NED_ENU ⊗ q_ros ⊗ q_FLU_FRD
static inline std::array<double, 4> enu_flu_to_ned_frd(double qw, double qx,
                                                         double qy, double qz)
{
    // Step 1: q_NED_ENU ⊗ q_ros  (q_NED_ENU = [0, 1/√2, 1/√2, 0])
    auto tmp = qmul(0.0, kSqrt2Inv, kSqrt2Inv, 0.0, qw, qx, qy, qz);
    // Step 2: result ⊗ q_FLU_FRD  (q_FLU_FRD = [0, 1, 0, 0])
    return qmul(tmp[0], tmp[1], tmp[2], tmp[3], 0.0, 1.0, 0.0, 0.0);
}

/// Convert a PX4 quaternion (FRD→NED) to an AirStack quaternion (FLU→ENU).
/// q_ros = q_ENU_NED ⊗ q_px4 ⊗ q_FRD_FLU
/// where q_ENU_NED = conj(q_NED_ENU) = [0, -1/√2, -1/√2, 0]
///   and q_FRD_FLU = q_FLU_FRD = [0, 1, 0, 0]  (self-inverse rotation)
static inline std::array<double, 4> ned_frd_to_enu_flu(double qw, double qx,
                                                         double qy, double qz)
{
    // Step 1: q_ENU_NED ⊗ q_px4
    auto tmp = qmul(0.0, -kSqrt2Inv, -kSqrt2Inv, 0.0, qw, qx, qy, qz);
    // Step 2: result ⊗ q_FRD_FLU  (= q_FLU_FRD = [0, 1, 0, 0])
    return qmul(tmp[0], tmp[1], tmp[2], tmp[3], 0.0, 1.0, 0.0, 0.0);
}

// ---------------------------------------------------------------------------
// PX4Interface class
// ---------------------------------------------------------------------------

class PX4Interface : public robot_interface::RobotInterface
{
public:
    PX4Interface() : RobotInterface("px4_interface")
    {
        // Use BEST_EFFORT + VOLATILE to match PX4's default uXRCE-DDS QoS.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .best_effort()
                       .durability_volatile();

        // ---- Publishers → PX4 ----
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

        // ---- Subscribers ← PX4 ----
        vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "out/vehicle_status", qos,
                std::bind(&PX4Interface::on_vehicle_status, this,
                          std::placeholders::_1));

        vehicle_odom_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "out/vehicle_odometry", qos,
                std::bind(&PX4Interface::on_vehicle_odometry, this,
                          std::placeholders::_1));

        // ---- AirStack odometry output ----
        odometry_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

        // ---- Optional: visual odometry input from AirStack ----
        visual_odom_in_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "visual_odometry_in", 10,
                std::bind(&PX4Interface::on_visual_odometry_in, this,
                          std::placeholders::_1));

        // ---- Heartbeat timer at 10 Hz ----
        // PX4 requires offboard_control_mode to be published at ≥ 2 Hz while
        // in offboard mode.  We publish at 10 Hz to guarantee margin.
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PX4Interface::publish_offboard_heartbeat, this));

        RCLCPP_INFO(this->get_logger(), "PX4Interface initialised (uXRCE-DDS)");
    }

    virtual ~PX4Interface() = default;

    // -----------------------------------------------------------------------
    // RobotInterface: control-command callbacks
    // -----------------------------------------------------------------------

    /**
     * @brief Position setpoint (ENU → NED).
     *
     * Yaw convention:
     *   yaw_ned = π/2 − yaw_enu
     * (ENU yaw 0 = East; NED yaw 0 = North → when ENU yaw = 90° the vehicle
     *  points North, which is NED yaw 0.)
     */
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::POSITION);
        publish_offboard_heartbeat();

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = now_us();

        // Position: ENU → NED
        sp.position[0] = static_cast<float>(cmd->pose.position.y);   // N = ENU-y
        sp.position[1] = static_cast<float>(cmd->pose.position.x);   // E = ENU-x
        sp.position[2] = static_cast<float>(-cmd->pose.position.z);  // D = -ENU-z

        // Velocity and acceleration unused → NaN
        sp.velocity[0] = sp.velocity[1] = sp.velocity[2] = NAN;
        sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;

        // Yaw: extract ENU yaw then convert to NED yaw
        tf2::Quaternion q_enu(cmd->pose.orientation.x, cmd->pose.orientation.y,
                               cmd->pose.orientation.z, cmd->pose.orientation.w);
        double roll{}, pitch{}, yaw_enu{};
        tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw_enu);
        sp.yaw      = static_cast<float>(M_PI_2 - yaw_enu);
        sp.yawspeed = NAN;

        trajectory_sp_pub_->publish(sp);
    }

    /**
     * @brief Velocity setpoint (ENU → NED).
     *
     * Yaw-rate sign: ENU CCW positive → NED CW positive → negate.
     */
    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::VELOCITY);
        publish_offboard_heartbeat();

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = now_us();

        // Position unused → NaN
        sp.position[0] = sp.position[1] = sp.position[2] = NAN;

        // Velocity: ENU → NED
        sp.velocity[0] = static_cast<float>(cmd->twist.linear.y);   // vN = vEast (ENU-y)
        sp.velocity[1] = static_cast<float>(cmd->twist.linear.x);   // vE = vNorth (ENU-x)
        sp.velocity[2] = static_cast<float>(-cmd->twist.linear.z);  // vD = -vUp

        sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
        sp.yaw = NAN;

        // Yaw-rate: ENU CCW+ → NED CW+ → negate
        sp.yawspeed = static_cast<float>(-cmd->twist.angular.z);

        trajectory_sp_pub_->publish(sp);
    }

    /**
     * @brief Attitude + thrust setpoint.
     *
     * Quaternion convention:
     *   q_px4 = q_NED_ENU ⊗ q_ros ⊗ q_FLU_FRD
     *
     * Thrust convention:
     *   mav_msgs thrust.z is normalised [0,1] (positive = up in FLU).
     *   PX4 thrust_body[2] is negative-FRD (negative = up/thrust direction).
     *   So thrust_body[2] = −thrust.z.
     */
    void attitude_thrust_callback(
        const mav_msgs::msg::AttitudeThrust::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::ATTITUDE);
        publish_offboard_heartbeat();

        px4_msgs::msg::VehicleAttitudeSetpoint sp{};
        sp.timestamp = now_us();

        auto q = enu_flu_to_ned_frd(cmd->attitude.w, cmd->attitude.x,
                                     cmd->attitude.y, cmd->attitude.z);
        sp.q_d[0] = static_cast<float>(q[0]);  // w
        sp.q_d[1] = static_cast<float>(q[1]);  // x
        sp.q_d[2] = static_cast<float>(q[2]);  // y
        sp.q_d[3] = static_cast<float>(q[3]);  // z

        sp.thrust_body[0] = 0.0f;
        sp.thrust_body[1] = 0.0f;
        sp.thrust_body[2] = static_cast<float>(-cmd->thrust.z);

        attitude_sp_pub_->publish(sp);
    }

    /**
     * @brief Body-rate + thrust setpoint.
     *
     * PX4 VehicleRatesSetpoint is in body FRD frame.
     * AirStack mav_msgs rates are in body FLU frame.
     *   roll  (around x): same direction
     *   pitch (around y): negated (FLU-y = −FRD-y)
     *   yaw   (around z): negated (FLU-z = −FRD-z)
     */
    void rate_thrust_callback(
        const mav_msgs::msg::RateThrust::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::BODY_RATE);
        publish_offboard_heartbeat();

        px4_msgs::msg::VehicleRatesSetpoint sp{};
        sp.timestamp = now_us();

        sp.roll  = static_cast<float>(cmd->angular_rates.x);
        sp.pitch = static_cast<float>(-cmd->angular_rates.y);
        sp.yaw   = static_cast<float>(-cmd->angular_rates.z);

        sp.thrust_body[0] = 0.0f;
        sp.thrust_body[1] = 0.0f;
        sp.thrust_body[2] = static_cast<float>(-cmd->thrust.z);

        rates_sp_pub_->publish(sp);
    }

    /**
     * @brief Roll / pitch / yaw-rate + thrust setpoint.
     *
     * Same body-rate axis convention as rate_thrust_callback.
     * Note: cmd->roll and cmd->pitch are angles (rad), not rates — pitch is
     * negated for the same reason as pitch rate.
     */
    void roll_pitch_yawrate_thrust_callback(
        const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr cmd) override
    {
        set_control_mode(ControlMode::BODY_RATE);
        publish_offboard_heartbeat();

        px4_msgs::msg::VehicleRatesSetpoint sp{};
        sp.timestamp = now_us();

        sp.roll  = static_cast<float>(cmd->roll);
        sp.pitch = static_cast<float>(-cmd->pitch);
        sp.yaw   = static_cast<float>(-cmd->yaw_rate);

        sp.thrust_body[0] = 0.0f;
        sp.thrust_body[1] = 0.0f;
        sp.thrust_body[2] = static_cast<float>(-cmd->thrust.z);

        rates_sp_pub_->publish(sp);
    }

    // -----------------------------------------------------------------------
    // RobotInterface: command functions
    // -----------------------------------------------------------------------

    /**
     * @brief Request offboard control mode from PX4.
     *
     * Sends VEHICLE_CMD_DO_SET_MODE with:
     *   param1 = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
     *   param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
     *
     * The offboard heartbeat (offboard_control_mode + setpoint) must already
     * be flowing before calling this, otherwise PX4 will reject the switch.
     */
    bool request_control() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,  // param1: enable custom mode
            6.0f   // param2: OFFBOARD
        );
        RCLCPP_INFO(this->get_logger(), "Offboard mode requested.");
        return true;
    }

    /// Arm the vehicle via VEHICLE_CMD_COMPONENT_ARM_DISARM (param1 = 1).
    bool arm() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
        return true;
    }

    /// Disarm the vehicle via VEHICLE_CMD_COMPONENT_ARM_DISARM (param1 = 0).
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

    /// Send NAV_TAKEOFF command (PX4 will use the configured takeoff altitude).
    bool takeoff() override
    {
        send_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
        RCLCPP_INFO(this->get_logger(), "Takeoff command sent.");
        return true;
    }

    /// Send NAV_LAND command.
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
    // Member variables
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
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;

    // AirStack I/O
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_in_sub_;

    // Heartbeat timer
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// @return Microseconds since system start used by PX4.
    uint64_t now_us()
    {
        return static_cast<uint64_t>(
            this->get_clock()->now().nanoseconds() / 1000ULL);
    }

    void set_control_mode(ControlMode mode) { control_mode_ = mode; }

    /// Publish the offboard_control_mode heartbeat with the current mode flags.
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
    }

    /**
     * @brief Send a VehicleCommand to PX4.
     *
     * All unused params default to 0.  target_system = 1 (the autopilot).
     */
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
        cmd.param5           = p5;  // float64
        cmd.param6           = p6;  // float64
        cmd.param7           = p7;
        cmd.target_system    = 1;
        cmd.target_component = 1;
        cmd.source_system    = 1;
        cmd.source_component = 1;   // uint16
        cmd.from_external    = true;
        vehicle_cmd_pub_->publish(cmd);
    }

    // -----------------------------------------------------------------------
    // PX4 subscriber callbacks
    // -----------------------------------------------------------------------

    void on_vehicle_status(
        const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        vehicle_status_  = *msg;
        status_received_ = true;
    }

    /**
     * @brief Convert PX4 VehicleOdometry (NED/FRD) to nav_msgs/Odometry (ENU/FLU).
     *
     * The VehicleOdometry published on out/vehicle_odometry has:
     *   pose_frame     = POSE_FRAME_NED
     *   velocity_frame = VELOCITY_FRAME_NED  (linear vel in NED world frame)
     *   angular_velocity in body FRD frame
     */
    void on_vehicle_odometry(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = this->get_clock()->now();
        odom.header.frame_id = "map";
        odom.child_frame_id  = "base_link";

        // Position: NED → ENU
        odom.pose.pose.position.x = static_cast<double>(msg->position[1]);   // E = NED-y
        odom.pose.pose.position.y = static_cast<double>(msg->position[0]);   // N = NED-x
        odom.pose.pose.position.z = static_cast<double>(-msg->position[2]);  // U = -NED-z

        // Attitude quaternion: FRD→NED → FLU→ENU
        auto q = ned_frd_to_enu_flu(
            static_cast<double>(msg->q[0]),   // w
            static_cast<double>(msg->q[1]),   // x
            static_cast<double>(msg->q[2]),   // y
            static_cast<double>(msg->q[3]));  // z
        odom.pose.pose.orientation.w = q[0];
        odom.pose.pose.orientation.x = q[1];
        odom.pose.pose.orientation.y = q[2];
        odom.pose.pose.orientation.z = q[3];

        // Linear velocity: NED world → ENU world
        odom.twist.twist.linear.x = static_cast<double>(msg->velocity[1]);   // E = NED-vy
        odom.twist.twist.linear.y = static_cast<double>(msg->velocity[0]);   // N = NED-vx
        odom.twist.twist.linear.z = static_cast<double>(-msg->velocity[2]);  // U = -NED-vz

        // Angular velocity: FRD body → FLU body
        odom.twist.twist.angular.x =  static_cast<double>(msg->angular_velocity[0]);  // roll  = same
        odom.twist.twist.angular.y = -static_cast<double>(msg->angular_velocity[1]);  // pitch negated
        odom.twist.twist.angular.z = -static_cast<double>(msg->angular_velocity[2]);  // yaw   negated

        odometry_pub_->publish(odom);
    }

    /**
     * @brief Accept a nav_msgs/Odometry in ENU/FLU and forward it to PX4's
     *        visual-odometry fusion input (in/vehicle_visual_odometry) in NED/FRD.
     *
     * Enable external vision fusion in PX4 with EKF2_EV_CTRL.
     */
    void on_visual_odometry_in(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        px4_msgs::msg::VehicleOdometry vio{};
        vio.timestamp        = now_us();
        vio.timestamp_sample = now_us();

        // Position: ENU → NED
        vio.position[0] = static_cast<float>(msg->pose.pose.position.y);   // N = ENU-y
        vio.position[1] = static_cast<float>(msg->pose.pose.position.x);   // E = ENU-x
        vio.position[2] = static_cast<float>(-msg->pose.pose.position.z);  // D = -ENU-z

        // Attitude quaternion: FLU→ENU → FRD→NED
        auto q = enu_flu_to_ned_frd(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        vio.q[0] = static_cast<float>(q[0]);
        vio.q[1] = static_cast<float>(q[1]);
        vio.q[2] = static_cast<float>(q[2]);
        vio.q[3] = static_cast<float>(q[3]);

        // Velocity: ENU → NED (world frame)
        vio.velocity[0] = static_cast<float>(msg->twist.twist.linear.y);   // vN = vENU-y
        vio.velocity[1] = static_cast<float>(msg->twist.twist.linear.x);   // vE = vENU-x
        vio.velocity[2] = static_cast<float>(-msg->twist.twist.linear.z);  // vD = -vENU-z

        vio.pose_frame     = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        vio.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        // Position variance (diagonal of 3×3 covariance)
        vio.position_variance[0] = static_cast<float>(msg->pose.covariance[0]);   // xx
        vio.position_variance[1] = static_cast<float>(msg->pose.covariance[7]);   // yy
        vio.position_variance[2] = static_cast<float>(msg->pose.covariance[14]);  // zz

        // Orientation variance
        vio.orientation_variance[0] = static_cast<float>(msg->pose.covariance[21]);  // roll
        vio.orientation_variance[1] = static_cast<float>(msg->pose.covariance[28]);  // pitch
        vio.orientation_variance[2] = static_cast<float>(msg->pose.covariance[35]);  // yaw

        visual_odom_pub_->publish(vio);
    }
};

}  // namespace px4_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(px4_interface::PX4Interface, robot_interface::RobotInterface)
