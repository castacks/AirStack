#include "rclcpp/rclcpp.hpp"

#include <airstack_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <airstack_common/tflib.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pid_controller_msgs/msg/pid_info.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <chrono>
#include <pid_controller/control_state.hpp>
#include <pid_controller/tf_diagnostic.hpp>

class PID {
public:

  rclcpp::Node* node;
  pid_controller::SampleClock sample_clock;
  
  pid_controller_msgs::msg::PIDInfo info;

  
  rclcpp::Publisher<pid_controller_msgs::msg::PIDInfo>::SharedPtr info_pub;

public:
  PID(rclcpp::Node* node, std::string name);
  void set_target(double target);
  double get_control(double measured, double ff_value=0.);
  void reset_integrator();
};

PID::PID(rclcpp::Node* node, std::string name)
  : node(node){
  airstack::dynamic_param(node, name + "_p", 1., &info.p);
  airstack::dynamic_param(node, name + "_i", 0., &info.i);
  airstack::dynamic_param(node, name + "_d", 0., &info.d);
  airstack::dynamic_param(node, name + "_ff", 0., &info.ff);

  
  airstack::dynamic_param(node, name + "_d_alpha", 0., &info.d_alpha);

  airstack::dynamic_param(node, name + "_min", -100000., &info.min);
  airstack::dynamic_param(node, name + "_max",  100000., &info.max);
  airstack::dynamic_param(node, name + "_constant", 0., &info.constant);

  info_pub = node->create_publisher<pid_controller_msgs::msg::PIDInfo>(name + "_pid_info", 1);
}

void PID::set_target(double target){
  info.target = target;
}

double PID::get_control(double measured, double ff_value){
  info.measured = measured;
  info.ff_value = ff_value;
  
  rclcpp::Time time_now = node->now();
  info.header.stamp = time_now;
  const double dt = sample_clock.next(time_now.nanoseconds(), info);
  pid_controller::step(info, dt);

  info_pub->publish(info);
  return info.control;
}

void PID::reset_integrator(){
  pid_controller::reset_history(info);
  sample_clock.reset();
}

class PIDControllerNode : public rclcpp::Node {
private:
  // params
  std::string target_frame;
  double max_roll_pitch;

  PID x_pid, y_pid, z_pid, vx_pid, vy_pid, vz_pid;
  pid_controller::ControlAuthority authority;
  bool active_previous = false;
  bool closed_loop_started = false;
  double active_since = 0.0, odometry_at = -INFINITY, tracking_at = -INFINITY;
  double state_timeout_s;
  double clock_order_wait_s;
  airstack_msgs::msg::Odometry::SharedPtr pending_tracking;
  double pending_tracking_received = 0.0;
  double pending_window_started = 0.0;
  rclcpp::TimerBase::SharedPtr clock_order_timer;

  // variables
  bool got_odometry;
  nav_msgs::msg::Odometry odometry;

  // subscribers
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_integrators_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub, control_sub;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  // publishers
  rclcpp::Publisher<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr command_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr admission_pub;
  uint64_t admission_sequence = 0;
  std::string tf_diagnostic = "null";
  double tracking_gap_s = INFINITY;
  uint32_t history_reset_mask = 0;
  
public:
  PIDControllerNode()
    : Node("pid_controller")
    , x_pid(this, "x")
    , y_pid(this, "y")
    , z_pid(this, "z")
    , vx_pid(this, "vx")
    , vy_pid(this, "vy")
    , vz_pid(this, "vz")
    , authority(this->declare_parameter<double>("state_timeout_s", 0.5)){
    if(this->get_parameter("state_timeout_s").as_double() <= 0.0 ||
       !std::isfinite(this->get_parameter("state_timeout_s").as_double()))
      throw std::invalid_argument("state_timeout_s must be finite and positive");
    state_timeout_s = this->get_parameter("state_timeout_s").as_double();
    clock_order_wait_s = this->declare_parameter<double>("clock_order_wait_s", 0.1);
    if(!std::isfinite(clock_order_wait_s) || clock_order_wait_s <= 0.0 ||
       clock_order_wait_s > state_timeout_s)
      throw std::invalid_argument("clock_order_wait_s must be positive and no greater than state_timeout_s");
    // init params
    target_frame = airstack::get_param(this, "target_frame", std::string("base_link"));
    max_roll_pitch = airstack::get_param(this, "max_roll_pitch", 10.)*M_PI/180.;
    armed_sub = this->create_subscription<std_msgs::msg::Bool>(
      this->declare_parameter<std::string>("is_armed_topic", "is_armed"), 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){
        authority.armed(msg->data, steady_seconds());
        if(!authority.active(steady_seconds())) reset_pids();
      });
    control_sub = this->create_subscription<std_msgs::msg::Bool>(
      this->declare_parameter<std::string>("has_control_topic", "has_control"), 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){
        authority.control(msg->data, steady_seconds());
        if(!authority.active(steady_seconds())) reset_pids();
      });

    // init subscribers
    odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>("odometry", 1,
								  std::bind(&PIDControllerNode::odometry_callback,
									    this, std::placeholders::_1));
    tracking_point_sub = this->create_subscription<airstack_msgs::msg::Odometry>("tracking_point", 1,
										 std::bind(&PIDControllerNode::tracking_point_callback,
											   this, std::placeholders::_1));
    reset_integrators_sub = this->create_subscription<std_msgs::msg::Empty>("reset_integrators", 1,
									    std::bind(&PIDControllerNode::reset_integrators_callback,
										      this, std::placeholders::_1));
    
    tf_buffer = new tf2_ros::Buffer(this->get_clock());
    //tf_buffer->setUsingDedicatedThread(true);
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);
    

    // init publishers
    command_pub = this->create_publisher<mav_msgs::msg::RollPitchYawrateThrust>("command", 1);
    admission_pub = this->create_publisher<std_msgs::msg::String>("admission_diagnostic", rclcpp::QoS(10).best_effort());

    // init variables
    got_odometry = false;
    clock_order_timer = this->create_wall_timer(std::chrono::milliseconds(5), [this](){
      if(!pending_tracking) return;
      const double now = steady_seconds();
      const auto stamp = rclcpp::Time(pending_tracking->header.stamp,
                                     this->get_clock()->get_clock_type());
      if(this->now() < stamp && now-pending_window_started < clock_order_wait_s &&
         authority.active(now)) return;
      auto message = pending_tracking;
      const double received = pending_tracking_received;
      pending_tracking.reset();
      process_tracking_point(message, received);
    });
  }

  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg){
    const double received = steady_seconds();
    const double lead = (rclcpp::Time(msg->header.stamp,
                          this->get_clock()->get_clock_type())-this->now()).seconds();
    // /clock and tracking travel on separate DDS subscriptions in Isaac.
    // Retain original receipt age and wait for a small positive clock lead;
    // never admit future evidence, refresh its receipt, or wait indefinitely.
    if(this->get_parameter("use_sim_time").as_bool() && authority.active(received) &&
       lead > 0.0 && lead <= clock_order_wait_s){
      // New samples replace the target, not the bounded wait's deadline.
      if(!pending_tracking) pending_window_started = received;
      pending_tracking = msg;
      pending_tracking_received = received;
      return;
    }
    pending_tracking.reset();
    process_tracking_point(msg, received);
  }

  void process_tracking_point(const airstack_msgs::msg::Odometry::SharedPtr msg, double received){
    tf_diagnostic = "null";
    const double now = steady_seconds();
    const bool active = authority.active(now);
    tracking_gap_s = received - tracking_at;
    history_reset_mask = (!active ? 1u : 0u) | (!active_previous ? 2u : 0u) |
                         (tracking_gap_s > state_timeout_s ? 4u : 0u);
    if(!active || !active_previous || now - tracking_at > state_timeout_s){
      reset_pids();
      active_since = now;
    }
    tracking_at = received;
    active_previous = active;
    const auto pre_ros_now = this->now();
    uint32_t reasons = admission_reasons(now, pre_ros_now, msg->header.stamp);
    if(reasons){
      publish_idle();
      publish_admission(reasons, "pre_tf", now, pre_ros_now, msg->header.stamp);
      return;
    }
    // transform tracking point and odometry
    airstack_msgs::msg::Odometry tp;
    nav_msgs::msg::Odometry odom;
    airstack_msgs::msg::Odometry temp = *msg;
    bool tp_tf_success = transform_with_diagnostic(temp, &tp, "tracking");
    if(!tp_tf_success){
      RCLCPP_ERROR_STREAM(get_logger(), "failed to transform tracking point");
      const double tf_now = steady_seconds();
      const auto tf_ros_now = this->now();
      const uint32_t tf_reasons = admission_reasons(tf_now, tf_ros_now, msg->header.stamp) |
                                  pid_controller::TRACKING_TF_FAILED;
      reset_pids();
      publish_idle();
      publish_admission(tf_reasons, "tf", tf_now, tf_ros_now, msg->header.stamp);
      return;
    }
    bool odom_tf_success = transform_with_diagnostic(odometry, &odom, "odometry");
    if(!odom_tf_success){
      RCLCPP_ERROR_STREAM(get_logger(), "failed to transform odometry");
      const double tf_now = steady_seconds();
      const auto tf_ros_now = this->now();
      const uint32_t tf_reasons = admission_reasons(tf_now, tf_ros_now, msg->header.stamp) |
                                  pid_controller::ODOM_TF_FAILED;
      reset_pids();
      publish_idle();
      publish_admission(tf_reasons, "tf", tf_now, tf_ros_now, msg->header.stamp);
      return;
    }
    // TF lookup may block; recheck authority/freshness before publishing control.
    const double post_now = steady_seconds();
    const auto post_ros_now = this->now();
    reasons = admission_reasons(post_now, post_ros_now, msg->header.stamp);
    if(reasons){
      reset_pids();
      publish_idle();
      publish_admission(reasons, "post_tf", post_now, post_ros_now, msg->header.stamp);
      return;
    }

    tf2::Vector3 tp_pos = tflib::to_tf(tp.pose.position);
    tf2::Vector3 tp_vel = tflib::to_tf(tp.twist.linear);
    if (!pid_controller::horizontal_reference_valid(tp_vel.x(), tp_vel.y())) {
      reset_pids();
      publish_idle();
      publish_admission(pid_controller::TRACKING_VELOCITY_INVALID, "post_tf",
                        post_now, post_ros_now, msg->header.stamp);
      return;
    }
    tf2::Vector3 odom_pos = tflib::to_tf(odom.pose.pose.position);
    tf2::Vector3 odom_vel = tflib::to_tf(odom.twist.twist.linear);

    if(!closed_loop_started){
      for(PID *pid : {&x_pid, &y_pid, &z_pid, &vx_pid, &vy_pid, &vz_pid})
        pid->reset_integrator();
      closed_loop_started = true;
    }
    
    x_pid.set_target(tp_pos.x());
    y_pid.set_target(tp_pos.y());
    z_pid.set_target(tp_pos.z());

    double vx = x_pid.get_control(odom_pos.x());
    double vy = y_pid.get_control(odom_pos.y());
    double vz = z_pid.get_control(odom_pos.z());

    vx_pid.set_target(pid_controller::horizontal_velocity_target(
        vx, tp_vel.x(), x_pid.info.min, x_pid.info.max));
    vy_pid.set_target(pid_controller::horizontal_velocity_target(
        vy, tp_vel.y(), y_pid.info.min, y_pid.info.max));
    vz_pid.set_target(vz);

    double roll = -vy_pid.get_control(odom_vel.y());
    double pitch = vx_pid.get_control(odom_vel.x());
    double thrust = vz_pid.get_control(odom_vel.z());

    // compute control
    mav_msgs::msg::RollPitchYawrateThrust command;
    command.header.frame_id = target_frame;
    command.header.stamp = tp.header.stamp;
    
    command.roll = roll;//-std::max(-max_roll_pitch, std::min(max_roll_pitch, tp.pose.position.y - odom.pose.pose.position.y));
    command.pitch = pitch;//std::max(-max_roll_pitch, std::min(max_roll_pitch, tp.pose.position.x - odom.pose.pose.position.x));

    double _, yaw;
    tf2::Matrix3x3(tflib::to_tf(msg->pose.orientation)).getRPY(_, _, yaw);
    command.yaw_rate = yaw;

    //RCLCPP_INFO_STREAM(get_logger(), "roll pitch: " << (command.roll*180./M_PI) << " " << (command.pitch*180./M_PI));
    //RCLCPP_INFO_STREAM(get_logger(), "min max: " << vx_pid.info.min << " " << vx_pid.info.max << " " << vy_pid.info.min << " " << vy_pid.info.max);
    
    command.thrust.z = thrust;//0.5;

    command_pub->publish(command);
    publish_admission(0u, "active", post_now, post_ros_now, msg->header.stamp);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    got_odometry = true;
    odometry_at = steady_seconds();
    odometry = *msg;
  }

  void reset_integrators_callback(const std_msgs::msg::Empty::SharedPtr msg){
    RCLCPP_INFO_STREAM(get_logger(), "RESET INTEGRATORS");
    reset_pids();
  }

  static double steady_seconds(){
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  uint32_t admission_reasons(double now, const rclcpp::Time &ros_now,
                             const builtin_interfaces::msg::Time &stamp){
    using namespace pid_controller;
    uint32_t result = authority.failure_reasons(now);
    if(!got_odometry) result |= ODOM_MISSING;
    if(odometry_at < active_since) result |= ODOM_BEFORE_ACTIVATION;
    if(now - odometry_at > state_timeout_s) result |= ODOM_RECEIPT_EXPIRED;
    if(now - tracking_at > state_timeout_s) result |= TRACKING_RECEIPT_EXPIRED;
    const auto clock_type = this->get_clock()->get_clock_type();
    result |= stamp_reasons((ros_now-rclcpp::Time(stamp, clock_type)).seconds(), state_timeout_s,
                            TRACKING_FUTURE, TRACKING_STALE);
    if(got_odometry)
      result |= stamp_reasons((ros_now-rclcpp::Time(odometry.header.stamp, clock_type)).seconds(), state_timeout_s,
                              ODOM_FUTURE, ODOM_STALE);
    return result;
  }

  template<typename Odometry>
  bool transform_with_diagnostic(const Odometry &input, Odometry *output, const char *channel){
    const double started = steady_seconds();
    try {
      // Same transform implementation and per-lookup timeout as the bool overload.
      // Catch here so the original exception is retained without a second lookup.
      *output = tflib::transform_odometry(tf_buffer, input, target_frame, target_frame,
                                         rclcpp::Duration::from_seconds(0.1));
      return true;
    } catch (const tf2::TransformException &error) {
      const char *kind = "transform";
      if (dynamic_cast<const tf2::LookupException *>(&error)) kind = "lookup";
      else if (dynamic_cast<const tf2::ConnectivityException *>(&error)) kind = "connectivity";
      else if (dynamic_cast<const tf2::ExtrapolationException *>(&error)) kind = "extrapolation";
      else if (dynamic_cast<const tf2::InvalidArgumentException *>(&error)) kind = "invalid_argument";
      using pid_controller::bounded_json_string;
      const std::string detail = error.what();
      std::ostringstream out;
      out.precision(17);
      out << "{\"channel\":\"" << channel << "\",\"exception_kind\":\"" << kind << "\""
          << ",\"source_frame\":" << bounded_json_string(input.header.frame_id, 192)
          << ",\"source_child_frame\":" << bounded_json_string(input.child_frame_id, 192)
          << ",\"target_frame\":" << bounded_json_string(target_frame, 192)
          << ",\"stamp_ns\":" << rclcpp::Time(input.header.stamp).nanoseconds()
          << ",\"lookup_wall_s\":" << steady_seconds() - started
          << ",\"per_lookup_timeout_s\":0.1"
          << ",\"exception\":" << bounded_json_string(detail, 512)
          << ",\"exception_truncated\":" << (detail.size() > 512 ? "true" : "false")
          << ",\"frames_truncated\":" << (input.header.frame_id.size() > 192 ||
              input.child_frame_id.size() > 192 || target_frame.size() > 192 ? "true" : "false")
          << "}";
      tf_diagnostic = out.str();
      return false;
    }
  }

  void publish_admission(uint32_t reasons, const char *phase, double now,
                         const rclcpp::Time &ros_now, const builtin_interfaces::msg::Time &stamp){
    const auto clock_type = this->get_clock()->get_clock_type();
    std::ostringstream out;
    out.precision(17);
    const auto age = [](double value){
      if(!std::isfinite(value)) return std::string("null");
      std::ostringstream number; number.precision(17); number << value; return number.str();
    };
    out << "{\"schema\":\"pid-admission/v1\",\"sequence\":" << ++admission_sequence
        << ",\"reason_mask\":" << reasons << ",\"phase\":\"" << phase << "\""
        << ",\"active\":" << (reasons == 0u ? "true" : "false")
        << ",\"history_reset_mask\":" << history_reset_mask
        << ",\"ros_now_ns\":" << ros_now.nanoseconds()
        << ",\"tracking_stamp_ns\":" << rclcpp::Time(stamp, clock_type).nanoseconds()
        << ",\"odom_stamp_ns\":" << (got_odometry ? rclcpp::Time(odometry.header.stamp, clock_type).nanoseconds() : -1)
        << ",\"steady_now_s\":" << now
        << ",\"armed_receipt_age_s\":" << age(authority.armed_age(now))
        << ",\"control_receipt_age_s\":" << age(authority.control_age(now))
        << ",\"odom_receipt_age_s\":" << (got_odometry ? age(now-odometry_at) : "null")
        << ",\"tracking_gap_s\":" << age(tracking_gap_s)
        << ",\"tracking_receipt_age_s\":" << age(now-tracking_at)
        << ",\"tf_failure\":" << tf_diagnostic << "}";
    std_msgs::msg::String message; message.data = out.str();
    admission_pub->publish(message);
  }

  void publish_idle(){
    closed_loop_started = false;
    // Preserve prestream without stale target P/D or retained I. The configured
    // thrust constant is a compatibility baseline, NOT qualified hover thrust.
    mav_msgs::msg::RollPitchYawrateThrust command;
    command.header.frame_id = target_frame;
    command.header.stamp = this->now();
    for(PID *pid : {&x_pid, &y_pid, &z_pid, &vx_pid, &vy_pid, &vz_pid}){
      pid->reset_integrator();
      pid->info.target = pid->info.measured = pid->info.ff_value = 0.0;
      pid->get_control(0.0);
    }
    command.thrust.z = vz_pid.info.control;
    command_pub->publish(command);
  }

  void reset_pids(){
    active_previous = false;
    closed_loop_started = false;
    x_pid.reset_integrator();
    y_pid.reset_integrator();
    z_pid.reset_integrator();
    vx_pid.reset_integrator();
    vy_pid.reset_integrator();
    vz_pid.reset_integrator();
  }
  
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
