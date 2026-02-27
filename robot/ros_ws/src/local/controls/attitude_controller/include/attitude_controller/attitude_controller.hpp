#ifndef _DRONE_FLIGHT_CONTROL_H_
#define _DRONE_FLIGHT_CONTROL_H_

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hp>

#include <attitude_controller_msgs/msg/kfState.h>
#include <attitude_controller_msgs/msg/attitude_controller_debug.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/console.h>

#include <string>

#include <core_trajectory_library/trajectory_library.h>
#include <tflib/tflib.h>
#include <templib/templib.h>

class DroneFlightControl : public BaseNode {
private:

  // params
  std::string target_frame;
  double p1, p2, p3, p1_volt, p2_volt;
  double kp1, kp2, kp3, kp4, kd1, kd2, kd3, ki1_body, ki2_body, ki3_body, ki1_vel_body, ki2_vel_body,
    ki1_ground, ki2_ground, ki3_ground, ki1_vel_ground, ki2_vel_ground, ki3_vel_ground, kf1, kf2, kf3;

  double command_angle_limit;
  double i_angle_limit;
  double z_i_limit;
  double g, m, hover_throttle, hover_throttle_param;
  double min_dt;
  double low_thrust_duration, ekf_delay;
  bool use_ekf_state, use_ekf_dist, thrust_sim;
  double yawrate_limit;
  double upper_bbox;
  bool compensate_control;

  bool disturb_est_body_frame;
  bool clip_disturbances, vel_only_stuck, zero_rp_stuck;
  double attitude_stuck_limit, control_stuck_threshold, recover_threshold;
  
  // variables
  bool got_odom, /*got_tracking_point,*/ got_closest_point, /*got_vtp,*/ got_accel, got_filtered_odom, got_filtered_state, 
    got_in_air, got_run_local_planner, got_vtp_jerk, got_battery_volt;
  nav_msgs::msg::Odometry odom, prev_odom, /*tracking_point,*/ closest_point, /*vtp, */filtered_odom, prev_filtered_odom, px4_odom;
  subt_control_ekf::kfState filtered_state;
  std_msgs::msg::Bool run_local_planner;
  airstack_msgs::Odometry vtp_jerk;

  double x_i_body, y_i_body, z_i_body, x_i_ground, y_i_ground, z_i_ground, vx_i_ground, vy_i_ground, vz_i_ground;
  double vx_i_body, vy_i_body;
  double x_e_prev, y_e_prev, z_e_prev;
  ros::Time prev_time;
  attitude_controller_msgs::AttitudeControllerDebug debug;
  ros::Time low_thrust_start_time, start_time_ekf, lqr_start_time;
  double boost_factor_angle_p;
  std_msgs::Bool in_air;
  double roll_px4_odom, pitch_px4_odom;
  double battery_volt;

  templib::TimeOutMonitor* control_stuck_monitor;
  templib::TimeOutMonitor* recover_monitor;
  
  // accel variables
  tf2::Vector3 setpoint_vel_target_frame_prev;
  tf2::Vector3 actual_accel_target_frame, longterm_accel_filter, shortterm_accel_filter, command_filtered_accel;
  double accelAlpha;
  double accelLongTermAlpha;
  double targetDT;
  double angle_tilt;
  tf2::Vector3 gainAccel,alphaCommandAccel,betaAccel,maxAccel;
  
  // publishers
  ros::Publisher command_pub, debug_pub, control_stuck_pub;
  
  // subscribers
  tf::TransformListener* listener;
  ros::Subscriber odometry_sub, /*tracking_point_sub,*/ closest_point_sub, /*vtp_sub,*/ imu_sub, in_air_sub, filtered_odometry_sub;
  ros::Subscriber filtered_state_sub, pixhawk_odom_sub, run_local_planner_sub, vtp_jerk_sub, battery_volt_sub;

  // services
  ros::ServiceServer reset_integrator_server;
  
  // callbacks
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  //void tracking_point_callback(nav_msgs::Odometry msg);
  void closest_point_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void vtp_jerk_callback(const airstack_msgs::msg::Odometry::SharedPtr msg);
  //void vtp_callback(nav_msgs::Odometry msg);
  bool reset_integrator_callback(std_srvs::srv::Empty::Request& request, std_srvs::srv::Empty::Response& response);
  void reset_integrators();
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);
  void in_air_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void filtered_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void filtered_state_callback(subt_control_ekf::kfState msg);
  void pixhawk_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void run_local_planner_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void battery_volt_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  double thrustToThrottle(double thrust_normIn);
  
public:
  DroneFlightControl(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~DroneFlightControl();

};


#endif
