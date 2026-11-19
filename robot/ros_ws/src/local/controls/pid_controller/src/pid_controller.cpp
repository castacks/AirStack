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
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <pid_controller/attitude_feedforward.hpp>

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
  // attitude feed-forward (6-DOF references; see attitude_feedforward.hpp). Active only
  // while a reference attitude younger than ff_timeout arrives on `attitude_feedforward`
  // (nothing publishes it in the stock stacks, so the controller is then unchanged).
  double ff_timeout;
  double ff_max_tilt;
  double ff_velocity_gain;
  bool ff_thrust_compensation;

  PID x_pid, y_pid, z_pid, vx_pid, vy_pid, vz_pid;

  // variables
  bool got_odometry;
  nav_msgs::msg::Odometry odometry;

  // subscribers
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_integrators_sub;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_ff_sub;
  geometry_msgs::msg::QuaternionStamped attitude_ff;
  rclcpp::Time attitude_ff_time;
  bool got_attitude_ff;
  bool ff_was_active;
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
    ff_timeout = airstack::get_param(this, "attitude_feedforward_timeout_s", 0.25);
    ff_max_tilt = airstack::get_param(this, "attitude_feedforward_max_tilt_deg", 35.)*M_PI/180.;
    ff_velocity_gain = airstack::get_param(this, "attitude_feedforward_velocity_gain", 1.0);
    ff_thrust_compensation = airstack::get_param(this, "attitude_feedforward_thrust_compensation", true);
    got_attitude_ff = false;
    ff_was_active = false;

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
    attitude_ff_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "attitude_feedforward", 1,
        [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr m) {
          attitude_ff = *m;
          attitude_ff_time = this->now();
          got_attitude_ff = true;
        });
    
    tf_buffer = new tf2_ros::Buffer(this->get_clock());
    //tf_buffer->setUsingDedicatedThread(true);
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);
    

    // init publishers
    command_pub = this->create_publisher<mav_msgs::msg::RollPitchYawrateThrust>("command", 1);

    // init variables
    got_odometry = false;
  }

  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg){
    if(!got_odometry)
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
    tf2::Vector3 odom_pos = tflib::to_tf(odom.pose.pose.position);
    tf2::Vector3 odom_vel = tflib::to_tf(odom.twist.twist.linear);

    // attitude feed-forward: on while a fresh reference attitude arrives
    const bool ff_active = got_attitude_ff && (this->now() - attitude_ff_time).seconds() <= ff_timeout;
    if(ff_active != ff_was_active){
      RCLCPP_INFO_STREAM(get_logger(), "attitude feed-forward " << (ff_active ? "ON (6-DOF reference)" : "OFF"));
      ff_was_active = ff_active;
    }
    const double kv = ff_active ? ff_velocity_gain : 0.;
    
    x_pid.set_target(tp_pos.x());
    y_pid.set_target(tp_pos.y());
    z_pid.set_target(tp_pos.z());

    // with the feed-forward on, the reference velocity is added AFTER the position loop's
    // clamp: the clamp bounds the correction, not the planned speed (a quadrotor plan can
    // exceed the x/y_max that sets the gimbal plans' cruise speed)
    double vx = x_pid.get_control(odom_pos.x()) + kv*tp_vel.x();
    double vy = y_pid.get_control(odom_pos.y()) + kv*tp_vel.y();
    double vz = z_pid.get_control(odom_pos.z()) + kv*tp_vel.z();

    vx_pid.set_target(vx);
    vy_pid.set_target(vy);
    vz_pid.set_target(vz);

    double roll = -vy_pid.get_control(odom_vel.y());
    double pitch = vx_pid.get_control(odom_vel.x());
    double thrust = vz_pid.get_control(odom_vel.z());

    if(ff_active){
      double _r, _p, vehicle_yaw;
      tf2::Matrix3x3(tflib::to_tf(odometry.pose.pose.orientation)).getRPY(_r, _p, vehicle_yaw);
      const auto& q = attitude_ff.quaternion;
      const pid_controller::TiltAngles ff = pid_controller::feedforward_tilt(q.x, q.y, q.z, q.w, vehicle_yaw);
      const pid_controller::TiltAngles cmd = pid_controller::combine_tilt({roll, pitch}, ff, ff_max_tilt);
      roll = cmd.roll;
      pitch = cmd.pitch;
      if(ff_thrust_compensation)
        thrust = std::min(1.0, pid_controller::tilt_compensated_thrust(thrust, roll, pitch));
    }

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

  void reset_integrators_callback(const std_msgs::msg::Empty::SharedPtr msg){
    RCLCPP_INFO_STREAM(get_logger(), "RESET INTEGRATORS");
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
