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

class PID {
public:

  rclcpp::Node* node;
  rclcpp::Time time_prev;
  
  pid_controller_msgs::msg::PIDInfo info;

  
  rclcpp::Publisher<pid_controller_msgs::msg::PIDInfo>::SharedPtr info_pub;

public:
  PID(rclcpp::Node* node, std::string name);
  void set_target(double target);
  double get_control(double measured, double ff_value=0.);
  void reset_integrator();
};

PID::PID(rclcpp::Node* node, std::string name)
  : node(node)
  , time_prev(0){
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
  if(time_prev.seconds() == 0){
    time_prev = time_now;
    info_pub->publish(info);
    return 0.;
  }
  info.dt = (time_now - time_prev).seconds();
  time_prev = time_now;
  if(info.dt <= 0.){
    info_pub->publish(info);
    return info.control;
  }

  double error_prev = info.error;
  info.error = info.target - info.measured;
  info.p_component = info.p * info.error;
  
  info.integral += info.error * info.dt;
  if(info.i == 0)
    info.integral = 0;
  info.i_component = info.i * info.integral;
  if(info.i_component > info.max){
    info.integral = info.max / info.i;
    info.i_component = info.i * info.integral;
  }
  else if(info.i_component < info.min){
    info.integral = info.min / info.i;
    info.i_component = info.i * info.integral;
  }
  
  info.derivative = info.d_alpha*info.derivative + (1. - info.d_alpha)*(info.error - error_prev)/info.dt;
  info.d_component = info.d * info.derivative;
  
  info.ff_component = info.ff * info.ff_value;

  info.control = info.p_component + info.i_component + info.d_component + info.ff_component + info.constant;
  info.control = std::max(info.min, std::min(info.max, info.control));

  info_pub->publish(info);
  return info.control;
}

void PID::reset_integrator(){
  info.integral = 0.;
}

class PIDControllerNode : public rclcpp::Node {
private:
  // params
  std::string target_frame;
  double max_roll_pitch;

  PID x_pid, y_pid, z_pid, vx_pid, vy_pid, vz_pid;

  // NATIVE-SETPOINT MUZZLE (2026-09-02, MIGHTY_NATIVE_SETPOINTS / "Option A").
  //
  // Two command streams to PX4 at once is a fault: PX4 acts on whichever
  // setpoint arrived last and its offboard_control_mode flags follow the
  // setpoint TYPE, so interleaving this node's attitude targets with the
  // native path's mavros/setpoint_raw/local position targets makes the flight
  // task switch at the stream rate. While mighty_bridge's native setpoint
  // streamer owns the vehicle it therefore silences this node, by setting
  // `command_muted` over this node's own set_parameters service.
  //
  // WHY HERE and not somewhere else: this node is the LEAF of the legacy
  // chain — the only publisher of interface/cmd_roll_pitch_yawrate_thrust,
  // publishing from exactly one place (tracking_point_callback). One `if`
  // covers the whole path. A trajectory_controller MODE would not have worked:
  // tracking_point_pub->publish() is unconditional in its timer for PAUSE /
  // ROBOT_POSE / TRACK alike, so this cascade keeps emitting attitude in all
  // of them. Gating inside mavros_interface / robot_interface would have put
  // a new branch in the safety boundary that arming, takeoff and land all
  // route through, and in a class shared with px4_interface.
  //
  // DEFAULT false => every stack that does not opt in is bit-identical.
  //
  // WATCHDOG: `command_muted` is a DEADMAN, not a latch. The streamer must
  // re-assert it at least every `command_mute_timeout` seconds; if it stops
  // (crash, kill, node restart) this node un-mutes itself, because the
  // alternative is a muted cascade with nothing else commanding — PX4 loses
  // its setpoint stream and trips its OFFBOARD failsafe. Set the timeout <= 0
  // to disable the watchdog (a plain latch); do that only on a bench.
  bool command_muted;
  double command_mute_timeout;
  rclcpp::Time command_mute_stamp;
  bool command_mute_recovered;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mute_param_cb;

  // variables
  bool got_odometry;
  nav_msgs::msg::Odometry odometry;

  // subscribers
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_integrators_sub;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  // publishers
  rclcpp::Publisher<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr command_pub;
  
public:
  PIDControllerNode()
    : Node("pid_controller")
    , x_pid(this, "x")
    , y_pid(this, "y")
    , z_pid(this, "z")
    , vx_pid(this, "vx")
    , vy_pid(this, "vy")
    , vz_pid(this, "vz"){
    // init params
    target_frame = airstack::get_param(this, "target_frame", std::string("base_link"));
    max_roll_pitch = airstack::get_param(this, "max_roll_pitch", 10.)*M_PI/180.;

    // Native-setpoint muzzle (see the member declarations for the rationale).
    // Declared with airstack::get_param so the value is settable from the YAML
    // like every other parameter here, but the on-set callback is our OWN:
    // airstack::dynamic_param's shared callback only assigns the variable, and
    // this flag additionally needs its refresh STAMPED for the watchdog.
    // rclcpp supports multiple on-set callbacks, so this coexists with the
    // one the PID gain objects installed above.
    command_muted = airstack::get_param(this, "command_muted", false);
    command_mute_timeout = airstack::get_param(this, "command_mute_timeout", 0.5);
    command_mute_stamp = this->now();
    command_mute_recovered = false;
    mute_param_cb = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for(const auto& param : params){
          if(param.get_name() == "command_muted"){
            const bool was = command_muted;
            command_muted = param.as_bool();
            // Stamp EVERY assertion, not just the transitions: the refresh is
            // the deadman and a repeated `true` is exactly what feeds it.
            command_mute_stamp = this->now();
            if(command_muted)
              command_mute_recovered = false;
            if(was != command_muted){
              // Start the resumed cascade clean: while muted the vehicle was
              // flown by someone else, so any integral wound up before the
              // mute describes a world that no longer exists.
              if(!command_muted)
                reset_all_integrators();
              RCLCPP_WARN_STREAM(get_logger(),
                                 "pid command output " << (command_muted ? "MUTED" : "UNMUTED")
                                 << " (native setpoint path)");
            }
          }
          else if(param.get_name() == "command_mute_timeout"){
            command_mute_timeout = param.as_double();
          }
        }
        return result;
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

    // init variables
    got_odometry = false;
  }

  /**
   * True while the native MIGHTY->PX4 setpoint streamer owns the vehicle.
   * Self-clearing if the streamer stops refreshing the flag — see the
   * command_muted member declaration.
   */
  bool command_is_muted(){
    if(!command_muted)
      return false;
    if(command_mute_timeout <= 0.)
      return true;                       // watchdog disabled on purpose
    const double age = (this->now() - command_mute_stamp).seconds();
    if(age > command_mute_timeout){
      command_muted = false;
      if(!command_mute_recovered){
        command_mute_recovered = true;
        reset_all_integrators();
        RCLCPP_ERROR_STREAM(get_logger(),
                            "command_muted went stale (" << age << " s > "
                            << command_mute_timeout << " s): the native setpoint "
                            "streamer stopped refreshing it. UN-MUTING so the "
                            "vehicle keeps a commander.");
      }
      return false;
    }
    return true;
  }

  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg){
    if(!got_odometry)
      return;

    // Muzzled: the native setpoint path is commanding PX4 directly. Bail out
    // BEFORE the PID cascade runs, not just before the publish — running the
    // loops would keep winding the integrators against a target the vehicle
    // is not being flown to, and the handback would then start with a
    // saturated integral term. (Un-muting also resets them; see the on-set
    // parameter callback and command_is_muted().)
    if(command_is_muted())
      return;

    // transform tracking point and odometry
    airstack_msgs::msg::Odometry tp;
    nav_msgs::msg::Odometry odom;
    airstack_msgs::msg::Odometry temp = *msg;
    bool tp_tf_success = tflib::transform_odometry(tf_buffer, temp, target_frame, target_frame, &tp,
						   rclcpp::Duration::from_seconds(0.1));
    if(!tp_tf_success){
      RCLCPP_ERROR_STREAM(get_logger(), "failed to transform tracking point");
      return;
    }
    bool odom_tf_success = tflib::transform_odometry(tf_buffer, odometry, target_frame, target_frame, &odom,
						     rclcpp::Duration::from_seconds(0.1));
    if(!odom_tf_success){
      RCLCPP_ERROR_STREAM(get_logger(), "failed to transform odometry");
      return;
    }

    tf2::Vector3 tp_pos = tflib::to_tf(tp.pose.position);
    tf2::Vector3 tp_vel = tflib::to_tf(tp.twist.linear);
    tf2::Vector3 tp_acc = tflib::to_tf(tp.acceleration);
    tf2::Vector3 odom_pos = tflib::to_tf(odom.pose.pose.position);
    tf2::Vector3 odom_vel = tflib::to_tf(odom.twist.twist.linear);

    x_pid.set_target(tp_pos.x());
    y_pid.set_target(tp_pos.y());
    z_pid.set_target(tp_pos.z());

    // Trajectory feedforward (2026-09-02): the tracking point carries the
    // planned velocity and acceleration, but this cascade used to be pure
    // feedback — the ff_value stayed at its default 0 at every call site, so
    // planners with real dynamics (MIGHTY's jerk-continuous splines) were
    // tracked with lag and overshot corners into obstacles. Standard cascade
    // wiring: position loop ff = planned velocity, velocity loop ff =
    // planned acceleration. INERT unless the *_ff gains are set — they
    // default to 0.0 in config/pid_controller.yaml, so stacks that do not
    // opt in (droan, takeoff/land legs) behave bit-identically.
    double vx = x_pid.get_control(odom_pos.x(), tp_vel.x());
    double vy = y_pid.get_control(odom_pos.y(), tp_vel.y());
    double vz = z_pid.get_control(odom_pos.z(), tp_vel.z());

    vx_pid.set_target(vx);
    vy_pid.set_target(vy);
    vz_pid.set_target(vz);

    double roll = -vy_pid.get_control(odom_vel.y(), tp_acc.y());
    double pitch = vx_pid.get_control(odom_vel.x(), tp_acc.x());
    double thrust = vz_pid.get_control(odom_vel.z(), tp_acc.z());

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
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    got_odometry = true;
    odometry = *msg;
  }

  void reset_all_integrators(){
    x_pid.reset_integrator();
    y_pid.reset_integrator();
    z_pid.reset_integrator();
    vx_pid.reset_integrator();
    vy_pid.reset_integrator();
    vz_pid.reset_integrator();
  }

  void reset_integrators_callback(const std_msgs::msg::Empty::SharedPtr msg){
    RCLCPP_INFO_STREAM(get_logger(), "RESET INTEGRATORS");
    reset_all_integrators();
  }
  
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
