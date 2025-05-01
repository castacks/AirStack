#include <attitude_controller/attitude_controller.hpp>

DroneFlightControl::DroneFlightControl(std::string node_name)
  : BaseNode(node_name){
}

bool DroneFlightControl::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init params
  target_frame = airstack::get_param(this, "target_frame", std::string("map"));
  p1 = airstack::get_param(this, "p1", 24.28);
  p2 = airstack::get_param(this, "p2", 6.287);
  p3 = airstack::get_param(this, "p3", -1.844);
  kp1 = airstack::get_param(this, "kp1", 1.0);
  kp2 = airstack::get_param(this, "kp2", 1.0);
  kp3 = airstack::get_param(this, "kp3", 1.0);
  kp4 = airstack::get_param(this, "kp4", 1.0);
  kd1 = airstack::get_param(this, "kd1", 1.0);
  kd2 = airstack::get_param(this, "kd2", 1.0);
  kd3 = airstack::get_param(this, "kd3", 1.0);
  ki1_body = airstack::get_param(this, "ki1_body", 1.0);
  ki2_body = airstack::get_param(this, "ki2_body", 1.0);
  ki1_vel_body = airstack::get_param(this, "ki1_vel_body", 1.0);
  ki2_vel_body = airstack::get_param(this, "ki2_vel_body", 1.0);
  // ki3_body = airstack::get_param(this, "ki3_body", 1.0);
  ki1_ground = airstack::get_param(this, "ki1_ground", 1.0);
  ki2_ground = airstack::get_param(this, "ki2_ground", 1.0);
  ki3_ground = airstack::get_param(this, "ki3_ground", 1.0);
  ki1_vel_ground = airstack::get_param(this, "ki1_vel_ground", 0.0);
  ki2_vel_ground = airstack::get_param(this, "ki2_vel_ground", 0.0);
  ki3_vel_ground = airstack::get_param(this, "ki3_vel_ground", 0.0);
  kf1 = airstack::get_param(this, "kf1", 1.0);
  kf2 = airstack::get_param(this, "kf2", 1.0);
  kf3 = airstack::get_param(this, "kf3", 0.0);
  disturb_est_body_frame = airstack::get_param(this, "disturb_est_body_frame", false);
  command_angle_limit = airstack::get_param(this, "command_angle_limit", 20.0)*M_PI/180.;
  i_angle_limit = airstack::get_param(this, "i_angle_limit", 2.0)*M_PI/180.;
  z_i_limit = airstack::get_param(this, "z_i_limit", 10000.);
  g = airstack::get_param(this, "gravity", 9.81);
  m = airstack::get_param(this, "mass", 5000.0);
  hover_throttle_param = airstack::get_param(this, "hover_throttle", 0.4);
  min_dt = airstack::get_param(this, "min_dt", 0.0001);
  low_thrust_duration = airstack::get_param(this, "low_thrust_duration", 2.0);
  ekf_delay = airstack::get_param(this, "ekf_delay", 2.0);
  use_ekf_state = airstack::get_param(this, "use_ekf_state", true);
  use_ekf_dist = airstack::get_param(this, "use_ekf_dist", true);
  thrust_sim = airstack::get_param(this, "thrust_sim", true);
  boost_factor_angle_p = airstack::get_param(this, "boost_factor_angle_p", 1.0);
  yawrate_limit = fabs(airstack::get_param(this, "yawrate_limit", 10.0)*M_PI/180.);
  upper_bbox = airstack::get_param(this, "upper_bbox", 2.0);
  compensate_control = airstack::get_param(this, "compensate_control", false);
  p1_volt = airstack::get_param(this, "p1_volt", 1.0);
  p2_volt = airstack::get_param(this, "p2_volt", 1.0);

  attitude_stuck_limit = airstack::get_param(this, "attitude_stuck_limit", 15.0)*M_PI/180.;
  control_stuck_threshold = airstack::get_param(this, "control_stuck_threshold", 1.0);
  recover_threshold = airstack::get_param(this, "recover_threshold", 1.0);
  clip_disturbances = airstack::get_param(this, "clip_disturbances", true);
  vel_only_stuck = airstack::get_param(this, "vel_only_stuck", true);
  zero_rp_stuck = airstack::get_param(this, "zero_rp_stuck", true);
  
  // accel params
  accelAlpha         = airstack::get_param(this, "accelAlpha", 0.05);
  accelLongTermAlpha = airstack::get_param(this, "accelLongTermAlpha", 0.003);
  gainAccel = tf2::Vector3(airstack::get_param(this, "gainAccelX", 0.015),
			   airstack::get_param(this, "gainAccelY", 0.015),
			   airstack::get_param(this, "gainAccelZ", 0.008));
  betaAccel = tf2::Vector3(airstack::get_param(this, "betaAccelX", 1.0),
			   airstack::get_param(this, "betaAccelY", 1.0),
			   airstack::get_param(this, "betaAccelZ", 1.0));
  maxAccel  = tf2::Vector3(airstack::get_param(this, "maxAccelX", 0.5),
			   airstack::get_param(this, "maxAccelY", 0.5),
			   airstack::get_param(this, "maxAccelZ", 0.5));
  alphaCommandAccel = tf2::Vector3(airstack::get_param(this, "alphaCommandAccelX", 0.3),
				   airstack::get_param(this, "alphaCommandAccelY", 0.3),
				   airstack::get_param(this, "alphaCommandAccelZ", 0.3));
  targetDT = 1.0/airstack::get_param(this, "execute_target", 50);
  longterm_accel_filter = tf2::Vector3(0,0,9.81);
  shortterm_accel_filter = tf2::Vector3(0,0,9.81);
  command_filtered_accel = tf2::Vector3(0,0,0);
  setpoint_vel_target_frame_prev = tf2::Vector3(0,0,0);
  angle_tilt = 1.0;
  low_thrust_start_time = ros::Time(0, 0); // make sure the time is in the past far enough so that it doesn't start out in low thrust mode
  start_time_ekf = ros::Time(0, 0);

  // init variables
  got_odom = false; /*got_tracking_point = false;*/ got_closest_point = false; got_vtp_jerk = false;
  /*got_vtp = false;*/  got_accel = false; got_filtered_odom = false; got_filtered_state = false;
  in_air.data = false; got_in_air = false, got_battery_volt=false;
  x_i_body = 0; y_i_body = 0; // z_i_body = 0; 
  x_e_prev = 0; y_e_prev = 0; z_e_prev = 0; x_i_ground = 0; y_i_ground = 0; z_i_ground = 0;
  vx_i_ground = 0; vy_i_ground = 0; vz_i_ground = 0;
  hover_throttle = hover_throttle_param;

  prev_time = ros::Time::now();
  control_stuck_monitor = new templib::TimeOutMonitor(control_stuck_threshold);  
  recover_monitor = new templib::TimeOutMonitor(recover_threshold);  
  control_stuck_monitor->add_time();
  recover_monitor->add_time();
  
  // init publishers
  command_pub = nh->advertise<mav_msgs::RollPitchYawrateThrust>("roll_pitch_yawrate_thrust_command", 1);
  debug_pub = nh->advertise<drone_flight_control::DroneFlightControlDebug>("drone_flight_control_debug", 1);
  control_stuck_pub = nh->advertise<std_msgs::Bool>("control_stuck", 1);
  
  // init subscribers
  listener = new tf::TransformListener();
  //tracking_point_sub = nh->subscribe("tracking_point", 1, &DroneFlightControl::tracking_point_callback, this, ros::TransportHints().tcpNoDelay());
  closest_point_sub = nh->subscribe("closest_point", 1, &DroneFlightControl::closest_point_callback, this, ros::TransportHints().tcpNoDelay());
  //vtp_sub = nh->subscribe("virtual_target_point", 1, &DroneFlightControl::vtp_callback, this, ros::TransportHints().tcpNoDelay());
  odometry_sub = nh->subscribe("odometry", 1, &DroneFlightControl::odometry_callback, this, ros::TransportHints().tcpNoDelay());
  imu_sub = nh->subscribe("/imu/data", 2, &DroneFlightControl::imu_callback, this);
  in_air_sub = nh->subscribe("in_air", 1, &DroneFlightControl::in_air_callback, this);
  filtered_odometry_sub = nh->subscribe("filtered_odom_control", 1, &DroneFlightControl::filtered_odometry_callback, this, ros::TransportHints().tcpNoDelay());
  filtered_state_sub = nh->subscribe("subt_control_ekf_state", 1, &DroneFlightControl::filtered_state_callback, this, ros::TransportHints().tcpNoDelay());
  pixhawk_odom_sub = nh->subscribe("mavros/local_position/odom", 1, &DroneFlightControl::pixhawk_odom_callback, this, ros::TransportHints().tcpNoDelay());
  run_local_planner_sub = nh->subscribe("run_local_planner", 1, &DroneFlightControl::run_local_planner_callback, this);
  vtp_jerk_sub = nh->subscribe("tracking_point", 1, &DroneFlightControl::vtp_jerk_callback, this, ros::TransportHints().tcpNoDelay());
  battery_volt_sub = nh->subscribe("mavros/battery", 1, &DroneFlightControl::battery_volt_callback, this);

  // init services
  reset_integrator_server = nh->advertiseService("reset_integrator", &DroneFlightControl::reset_integrator_callback, this);
  
  return true;
}

bool DroneFlightControl::execute(){

  if(got_battery_volt && !thrust_sim)
    hover_throttle = p1_volt * battery_volt + p2_volt;
  else
    hover_throttle = hover_throttle_param;

  if(got_odom && got_vtp_jerk && odom.header.seq != prev_odom.header.seq){// && 
    // filtered_odom.header.seq != prev_filtered_odom.header.seq){

    ros::Time curr_time = odom.header.stamp;//ros::Time::now();
    double dt = (curr_time - prev_time).toSec();
    if(dt < min_dt)
      return true;
    prev_time = curr_time;
    prev_odom = odom;

    if((ros::Time::now() - low_thrust_start_time).toSec() < low_thrust_duration){
      mav_msgs::RollPitchYawrateThrust command;
      command.header.stamp = ros::Time::now();
      command.roll = 0;
      command.pitch = 0;
      command.thrust.z = 0.15;
      command.yaw_rate = 0;
      command_pub.publish(command);
      reset_integrators();

      return true;
    }    

    // actual values
    double x_a, y_a, z_a, vx_a, vy_a, vz_a;//, x_d, y_d, z_d, vx_d, vy_d, vz_d;
    double roll_a, pitch_a, yaw_a, roll_c, pitch_c, yaw_c, roll_d, pitch_d, yaw_d, roll_v, pitch_v, yaw_v;
    tf::Vector3 velocity_target_frame, position_target_frame;    
    //tf::Vector3 tracking_position_target, tracking_velocity_target;
    tf::Vector3 closest_position_target, closest_velocity_target;
    tf::Vector3 vtp_position_target, vtp_velocity_target;
    tf::StampedTransform velocity_transform_closest, velocity_transform_vtp;;
    try{
      
      // transform for filtered odom
      if(use_ekf_state && got_filtered_odom && start_time_ekf.toSec()>0.0 && (ros::Time::now() - start_time_ekf).toSec() > ekf_delay){
      // && filtered_odom.header.seq != prev_filtered_odom.header.seq){

        // tf::Vector3 filtered_velocity_target, filtered_position_target;
        tf::StampedTransform filtered_transform;
        listener->waitForTransform(target_frame, filtered_odom.header.frame_id, filtered_odom.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, filtered_odom.header.frame_id, filtered_odom.header.stamp, filtered_transform);
        tf::StampedTransform filtered_velocity_transform;
        listener->waitForTransform(target_frame, filtered_odom.child_frame_id, filtered_odom.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, filtered_odom.child_frame_id, filtered_odom.header.stamp, filtered_velocity_transform);
        filtered_velocity_transform.setOrigin(tf::Vector3(0, 0, 0));

        tf::Quaternion q_filtered_target = filtered_transform*tflib::to_tf(filtered_odom.pose.pose.orientation);
        tf::Matrix3x3(q_filtered_target).getRPY(roll_a, pitch_a, yaw_a);

        position_target_frame = filtered_transform*tflib::to_tf(filtered_odom.pose.pose.position);
        x_a = position_target_frame.x();
        y_a = position_target_frame.y();
        z_a = position_target_frame.z();
        
        velocity_target_frame = filtered_velocity_transform*tflib::to_tf(filtered_odom.twist.twist.linear);
        vx_a = velocity_target_frame.x();
        vy_a = velocity_target_frame.y();
        vz_a = velocity_target_frame.z();  
      }
      else{ // use absolute odom
        
        // transform for odom
        tf::StampedTransform transform;
        listener->waitForTransform(target_frame, odom.header.frame_id, odom.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, odom.header.frame_id, odom.header.stamp, transform);
        tf::StampedTransform velocity_transform;
        listener->waitForTransform(target_frame, odom.child_frame_id, odom.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, odom.child_frame_id, odom.header.stamp, velocity_transform);
        velocity_transform.setOrigin(tf::Vector3(0, 0, 0));

        tf::Quaternion q_target_frame = transform*tflib::to_tf(odom.pose.pose.orientation);
        tf::Matrix3x3(q_target_frame).getRPY(roll_a, pitch_a, yaw_a);

        position_target_frame = transform*tflib::to_tf(odom.pose.pose.position);      
        x_a = position_target_frame.x();
        y_a = position_target_frame.y();
        z_a = position_target_frame.z();
        
        velocity_target_frame = velocity_transform*tflib::to_tf(odom.twist.twist.linear);
        vx_a = velocity_target_frame.x();
        vy_a = velocity_target_frame.y();
        vz_a = velocity_target_frame.z();        
      }
      
      // transform for tracking point
      /*
      tf::StampedTransform transform_tracking;
      listener->waitForTransform(target_frame, tracking_point.header.frame_id, tracking_point.header.stamp, ros::Duration(0.1));
      listener->lookupTransform(target_frame, tracking_point.header.frame_id, tracking_point.header.stamp, transform_tracking);
      tf::StampedTransform velocity_transform_tracking;
      listener->waitForTransform(target_frame, tracking_point.child_frame_id, tracking_point.header.stamp, ros::Duration(0.1));
      listener->lookupTransform(target_frame, tracking_point.child_frame_id, tracking_point.header.stamp, velocity_transform_tracking);
      velocity_transform_tracking.setOrigin(tf::Vector3(0, 0, 0));

      tracking_position_target = transform_tracking*tflib::to_tf(tracking_point.pose.pose.position);
      tf::Quaternion q_tracking_target = transform_tracking*tflib::to_tf(tracking_point.pose.pose.orientation);
      x_d = tracking_position_target.x();
      y_d = tracking_position_target.y();
      z_d = tracking_position_target.z();
      tf::Matrix3x3(q_tracking_target).getRPY(roll_d, pitch_d, yaw_d);
      
      tracking_velocity_target = velocity_transform_tracking*tflib::to_tf(tracking_point.twist.twist.linear);
      vx_d = tracking_velocity_target.x();
      vy_d = tracking_velocity_target.y();
      vz_d = tracking_velocity_target.z();
      */

      // transform for closest point
      if( got_closest_point ){

        tf::StampedTransform transform_closest;
        listener->waitForTransform(target_frame, closest_point.header.frame_id, closest_point.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, closest_point.header.frame_id, closest_point.header.stamp, transform_closest);
        
        listener->waitForTransform(target_frame, closest_point.child_frame_id, closest_point.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, closest_point.child_frame_id, closest_point.header.stamp, velocity_transform_closest);
        velocity_transform_closest.setOrigin(tf::Vector3(0, 0, 0)); // TODO: what does this line do?

        closest_position_target = transform_closest*tflib::to_tf(closest_point.pose.pose.position);
        tf::Quaternion q_closest_target = transform_closest*tflib::to_tf(closest_point.pose.pose.orientation);
        tf::Matrix3x3(q_closest_target).getRPY(roll_c, pitch_c, yaw_c);

        closest_velocity_target = velocity_transform_closest*tflib::to_tf(closest_point.twist.twist.linear);
      }

      // transform for VTP
      if( got_vtp_jerk ){

        tf::StampedTransform transform_vtp;
        listener->waitForTransform(target_frame, vtp_jerk.header.frame_id, vtp_jerk.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, vtp_jerk.header.frame_id, vtp_jerk.header.stamp, transform_vtp);
        
        listener->waitForTransform(target_frame, vtp_jerk.child_frame_id, vtp_jerk.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, vtp_jerk.child_frame_id, vtp_jerk.header.stamp, velocity_transform_vtp);
        velocity_transform_vtp.setOrigin(tf::Vector3(0, 0, 0)); // TODO: what does this line do?

        vtp_position_target = transform_vtp*tflib::to_tf(vtp_jerk.pose.position);
        tf::Quaternion q_vtp_target = transform_vtp*tflib::to_tf(vtp_jerk.pose.orientation);
        tf::Matrix3x3(q_vtp_target).getRPY(roll_v, pitch_v, yaw_v);

        vtp_velocity_target = velocity_transform_vtp*tflib::to_tf(vtp_jerk.twist.linear);
      }

      // transform for px4
      if(compensate_control){

        double yaw_temp;
        tf::StampedTransform transform_px4;
        listener->waitForTransform(target_frame, px4_odom.header.frame_id, px4_odom.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(target_frame, px4_odom.header.frame_id, px4_odom.header.stamp, transform_px4);

        tf::Quaternion px4_q_target_frame = transform_px4*tflib::to_tf(px4_odom.pose.pose.orientation); 
        tf::Matrix3x3(px4_q_target_frame).getRPY(roll_px4_odom, pitch_px4_odom, yaw_temp);
      }
      
    } 
    catch(tf::TransformException& te){
      ROS_ERROR_STREAM("TransformException while transform odometry: " << te.what());
      return true;
    }
    
    
    double yaw_tf = yaw_a;

    // declare variables
    double x_e, y_e, z_e, yaw_e=0, vx_e, vy_e, vz_e, x_e_d, y_e_d, z_e_d, roll, pitch, yawrate, throttle, thrust_norm;
    double x_e_body, y_e_body, vx_e_body, vy_e_body;
    double x_ff=0, y_ff=0, z_ff=0;

    ///// Control laws ///////// 

    // check stuck condition      
    bool velOnly=false, zero_rp=false;    
    if( sqrt( roll_a*roll_a + pitch_a*pitch_a ) < attitude_stuck_limit &&
      sqrt( roll_px4_odom*roll_px4_odom + pitch_px4_odom*pitch_px4_odom ) < attitude_stuck_limit ){
      control_stuck_monitor->add_time();
    }

    std_msgs::Bool control_stuck_msg;      

    debug.recover_mode.data = false; // default

    if(control_stuck_monitor->is_timed_out()){
      control_stuck_msg.data = true;

      if(!recover_monitor->is_timed_out()){
        // do recovery action
        debug.recover_mode.data=true;    

        // clip disturbances
        if(clip_disturbances){
          filtered_state.disOut.x=0.;
          filtered_state.disOut.y=0.;
        }

        // switch to vel control
        if(vel_only_stuck){
          velOnly=true;
        }

        // switch to zero_rp
        if(zero_rp_stuck){
          zero_rp=true;
        }
      }
    }
    else{
      control_stuck_msg.data = false;
      recover_monitor->add_time();
    }    
    control_stuck_pub.publish(control_stuck_msg);

    debug.stuck_mode.data = control_stuck_msg.data;

    // When tracking VTP //
    if(got_vtp_jerk){// && got_filtered_odom){// && filtered_odom.header.seq != prev_filtered_odom.header.seq){      

      tf::Vector3 diffPos = vtp_position_target - position_target_frame;

      // TODO: better way to get direction of motion
      tf::Vector3 posErr;
      if( fabs(vtp_velocity_target.length2()) < 1e-6 || diffPos.length() > upper_bbox){

        posErr = diffPos;
        // ROS_INFO_STREAM("Setpoint velocity went to zero");
      }
      else
        posErr = diffPos - diffPos.dot(vtp_velocity_target)/( vtp_velocity_target.length2() )*vtp_velocity_target;

      tf::Vector3 velErr = vtp_velocity_target - velocity_target_frame;

      // errors
      x_e = posErr.x();
      y_e = posErr.y();
      z_e = vtp_position_target.z() - z_a;

      if(velOnly){
        x_e=0.; y_e=0.;
      }

      yaw_e = atan2(sin(yaw_v - yaw_a), cos(yaw_v - yaw_a));
      vx_e = velErr.x();
      vy_e = velErr.y();
      vz_e = velErr.z(); 
                    
      if(got_run_local_planner && run_local_planner.data && got_vtp_jerk){

        // tf::Vector3 vtp_acceleration_target = velocity_transform_vtp*tflib::to_tf(vtp_jerk.acceleration);
        // x_ff = vtp_acceleration_target.x(); y_ff = vtp_acceleration_target.y(); z_ff = vtp_acceleration_target.z();
        x_ff = vtp_jerk.acceleration.x; y_ff = vtp_jerk.acceleration.y; z_ff = vtp_jerk.acceleration.z;
      }      
      else if(got_vtp_jerk){
        x_ff = vtp_jerk.acceleration.x; y_ff = vtp_jerk.acceleration.y; z_ff = vtp_jerk.acceleration.z;
      }
    }  

    // for integrators
    x_e_body = cos(yaw_tf)*x_e + sin(yaw_tf)*y_e;
    vx_e_body = cos(yaw_tf)*vx_e + sin(yaw_tf)*vy_e;
    y_e_body =  -sin(yaw_tf)*x_e + cos(yaw_tf)*y_e;
    vy_e_body =  -sin(yaw_tf)*vx_e + cos(yaw_tf)*vy_e;

    double accel_xdes = kp1 * x_e + kd1 * vx_e  + ki1_ground * x_i_ground + ki1_vel_ground*vx_i_ground + kf1 * x_ff;
    double accel_ydes = kp2 * y_e + kd2 * vy_e  + ki2_ground * y_i_ground + ki2_vel_ground*vy_i_ground + kf2 * y_ff;
    double accel_zdes = kp3 * z_e + kd3 * vz_e  + ki3_ground * z_i_ground + ki3_vel_ground*vz_i_ground + kf3 * z_ff;

    // add ground frame dist terms
    if (use_ekf_dist && got_filtered_odom && !disturb_est_body_frame && start_time_ekf.toSec()>0.0 && 
      (ros::Time::now() - start_time_ekf).toSec() > ekf_delay ){
      accel_xdes += filtered_state.disOut.x;
      accel_ydes += filtered_state.disOut.y;
      accel_zdes += filtered_state.disOut.z * g;
    }
    
    // generate control inputs
    // thrust_norm = (tf::Vector3(accel_xdes, accel_ydes, accel_zdes) + tf::Vector3(0,0,g)).length();

    roll = ( accel_xdes * sin(yaw_tf) - accel_ydes * cos(yaw_tf) )/g - ki2_body*y_i_body - ki2_vel_body*vy_i_body;
    pitch = ( accel_xdes * cos(yaw_tf) + accel_ydes * sin(yaw_tf) )/g + ki1_body*x_i_body + ki1_vel_body*vx_i_body;
    // roll = asin( std::min( 1.0, std::max( (accel_xdes*sin(yaw_tf) - accel_ydes*cos(yaw_tf) )/
    //                     thrust_norm, -1.0 )) );
    // pitch = acos( std::min(1.0, std::max(-1.0, (accel_zdes + g)/(thrust_norm*cos(roll_a)) )) );


    if(compensate_control && in_air.data){
      roll += roll_px4_odom - roll_a;
      pitch += pitch_px4_odom - pitch_a;
    }

    // add body frame dist terms
    if(use_ekf_dist && got_filtered_odom && disturb_est_body_frame && start_time_ekf.toSec()>0.0 && 
      (ros::Time::now() - start_time_ekf).toSec() > ekf_delay ){
      roll += filtered_state.disOut.x;
      pitch += filtered_state.disOut.y;
      accel_zdes += filtered_state.disOut.z * g; 
    }

    accel_zdes = std::max(accel_zdes, -g);    
    thrust_norm = (accel_zdes+g);
    // thrust_norm = (accel_zdes + g)/std::max(0.5, std::min(1.0,  cos(roll_a)*cos(pitch_a) ) );
    yawrate = std::max(-yawrate_limit, std::min(yawrate_limit, kp4*yaw_e));

    // get throttle from thrust
    if(isnan(thrust_norm))
      throttle = hover_throttle;
    else
      throttle = std::min( 1.0, thrustToThrottle(thrust_norm) ); 


    // cap control angles
    if(roll > command_angle_limit)
      roll = command_angle_limit;
    else if(roll < -command_angle_limit)
      roll = -command_angle_limit;
    else if(isnan(roll))
      roll = 0;

    if(pitch > command_angle_limit)
      pitch = command_angle_limit;
    else if(pitch < -command_angle_limit)
      pitch = -command_angle_limit;
    else if (isnan(pitch))
      pitch=0;

    if(isnan(yaw_e))
      yawrate=0;     

    // while taking off //
    if(!in_air.data){

      roll = 0; pitch=0; throttle = hover_throttle; yawrate=0;
    }      

    // recovery condition 
    if(zero_rp){roll=0.; pitch=0.;}

    // publish debug info
    debug.header = odom.header;

    if(got_closest_point){
      
      debug.position_closest.x = closest_position_target.x();
      debug.position_closest.y = closest_position_target.y();
      debug.position_closest.z = closest_position_target.z();
      debug.yaw_closest = yaw_c;
      debug.velocity_closest.x = closest_velocity_target.x();
      debug.velocity_closest.y = closest_velocity_target.y();
      debug.velocity_closest.z = closest_velocity_target.z();
    }

    if(got_vtp_jerk){

      debug.position_vtp.x = vtp_position_target.x();
      debug.position_vtp.y = vtp_position_target.y();
      debug.position_vtp.z = vtp_position_target.z();
      debug.yaw_vtp = yaw_c;
      debug.velocity_vtp.x = vtp_velocity_target.x();
      debug.velocity_vtp.y = vtp_velocity_target.y();
      debug.velocity_vtp.z = vtp_velocity_target.z();
      debug.acceleration_vtp.x = vtp_jerk.acceleration.x;
      debug.acceleration_vtp.x = vtp_jerk.acceleration.y;
      debug.acceleration_vtp.x = vtp_jerk.acceleration.z;
      debug.accel_vtp_net = tf::Vector3(vtp_jerk.acceleration.x, vtp_jerk.acceleration.y, vtp_jerk.acceleration.z + g).length();
    }
    
    debug.position_actual.x = x_a;
    debug.position_actual.y = y_a;
    debug.position_actual.z = z_a;
    debug.yaw_actual = yaw_a;
    debug.velocity_actual.x = vx_a;
    debug.velocity_actual.y = vy_a;
    debug.velocity_actual.z = vz_a;

    debug.integrator_ground.x = x_i_ground;
    debug.integrator_ground.y = y_i_ground;
    debug.integrator_ground.z = z_i_ground;

    debug.vel_integrator_ground.x = vx_i_ground;
    debug.vel_integrator_ground.y = vy_i_ground;
    debug.vel_integrator_ground.z = vz_i_ground;

    debug.integrator_body.x = x_i_body;
    debug.integrator_body.y = y_i_body;

    debug.vel_integrator_body.x = vx_i_body;
    debug.vel_integrator_body.y = vy_i_body;    

    debug.position_error.x = x_e;
    debug.position_error.y = y_e;
    debug.position_error.z = z_e;
    debug.yaw_error = yaw_e;
    debug.velocity_error.x = vx_e;
    debug.velocity_error.y = vy_e;
    debug.velocity_error.z = vz_e;

    debug.derivative.x = x_e_d;
    debug.derivative.y = y_e_d;
    debug.derivative.z = z_e_d;

    debug.kp.x = kp1;
    debug.kp.y = kp2;
    debug.kp.z = kp3;
    debug.ki_ground.x = ki1_ground;
    debug.ki_ground.y = ki2_ground;
    debug.ki_ground.z = ki3_ground;
    debug.ki_vel_ground.x = ki1_vel_ground;
    debug.ki_vel_ground.y = ki2_vel_ground;
    debug.ki_vel_ground.z = ki3_vel_ground;
    debug.kd.x = kd1;
    debug.kd.y = kd2;
    debug.kd.z = kd3;
    
    debug.dt = dt;
    
    debug.roll = roll * 180./M_PI;
    debug.pitch = pitch * 180./M_PI;
    debug.yawrate = yawrate;
    debug.throttle = throttle;
    debug.thrust_norm = thrust_norm;
    
    debug_pub.publish(debug);
    
    // accumulate integrators
    // if(ki2_body != 0.)
    //   x_i_body = std::max(-fabs(i_angle_limit*g/ki2_body), std::min(fabs(i_angle_limit*g/ki2_body), x_i_body + x_e_body));
    // if(ki1_body != 0.)
    //   y_i_body = std::max(-fabs(i_angle_limit*g/ki1_body), std::min(fabs(i_angle_limit*g/ki1_body), y_i_body + y_e_body));
    // if(ki3_body != 0.)
    //   z_i_body = std::max(-fabs(z_i_limit), std::min(fabs(z_i_limit), z_i_body + z_e));
    double ki_ground = std::max(fabs(ki1_ground), fabs(ki2_ground));
    if(ki_ground != 0.)
      x_i_ground = std::max(-fabs(i_angle_limit*g/ki_ground), std::min(fabs(i_angle_limit*g/ki_ground), x_i_ground + x_e));
    if(ki_ground != 0.)
      y_i_ground = std::max(-fabs(i_angle_limit*g/ki_ground), std::min(fabs(i_angle_limit*g/ki_ground), y_i_ground + y_e));
    z_i_ground += z_e;

    double ki_vel_ground = std::max(fabs(ki1_vel_ground), fabs(ki2_vel_ground));
    if(ki_vel_ground != 0.)
      vx_i_ground = std::max(-fabs(i_angle_limit*g/ki_vel_ground), 
          std::min(fabs(i_angle_limit*g/ki_vel_ground), vx_i_ground + vx_e));

    if(ki_vel_ground != 0.)
      vy_i_ground = std::max(-fabs(i_angle_limit*g/ki_vel_ground), 
          std::min(fabs(i_angle_limit*g/ki_vel_ground), vy_i_ground + vy_e));

    vz_i_ground += vz_e;

    double ki_body = std::max(fabs(ki1_body), fabs(ki2_body));
    if(ki_body != 0.)
      x_i_body = std::max(-fabs(i_angle_limit*g/ki_body), 
          std::min(fabs(i_angle_limit*g/ki_body), x_i_body + x_e_body));

    if(ki_body != 0.)
      y_i_body = std::max(-fabs(i_angle_limit*g/ki_body), 
          std::min(fabs(i_angle_limit*g/ki_body), y_i_body + y_e_body));

    double ki_vel_body = std::max(fabs(ki1_vel_body), fabs(ki2_vel_body));
    if(ki_vel_body != 0.)
      vx_i_body = std::max(-fabs(i_angle_limit*g/ki_vel_body), 
          std::min(fabs(i_angle_limit*g/ki_vel_body), vx_i_body + vx_e_body));

    if(ki_vel_body != 0.)
      vy_i_body = std::max(-fabs(i_angle_limit*g/ki_vel_body), 
          std::min(fabs(i_angle_limit*g/ki_vel_body), vy_i_body + vy_e_body));

    // x_i_body += x_e_body;
    // y_i_body += y_e_body;
    // vx_i_body += vx_e_body;
    // vy_i_body += vy_e_body;
    
    // set previous errors
    x_e_prev = x_e;
    y_e_prev = y_e;
    z_e_prev = z_e; 

    mav_msgs::RollPitchYawrateThrust command;
    command.header.stamp = ros::Time::now();
    command.roll = roll;
    command.pitch = pitch;
    command.thrust.z = throttle;
    command.yaw_rate = yawrate;
    command_pub.publish(command); 
  }
      
  return true;
}

void DroneFlightControl::odometry_callback(nav_msgs::Odometry msg){
  got_odom = true;
  odom = msg;
}
/*
void DroneFlightControl::tracking_point_callback(nav_msgs::Odometry msg){
  got_tracking_point = true;
  tracking_point = msg;
}
*/
void DroneFlightControl::closest_point_callback(nav_msgs::Odometry msg){
  got_closest_point = true;
  closest_point = msg;
}

void DroneFlightControl::vtp_jerk_callback(core_trajectory_msgs::Odometry msg){
  got_vtp_jerk = true;
  vtp_jerk = msg;
}
/*
void DroneFlightControl::vtp_callback(nav_msgs::Odometry msg){
  got_vtp = true;
  vtp = msg;
}
*/
bool DroneFlightControl::reset_integrator_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  reset_integrators();
  low_thrust_start_time = ros::Time::now();
  return true;
}

void DroneFlightControl::reset_integrators(){
  x_i_body = 0;
  y_i_body = 0;
  vx_i_body=0; 
  vy_i_body=0;
  // z_i_body = 0;
  x_i_ground = 0;
  y_i_ground = 0;
  z_i_ground = 0;
  vx_i_ground = 0;
  vy_i_ground = 0;
  vz_i_ground = 0;
}

void DroneFlightControl::imu_callback(sensor_msgs::Imu imu){
  //Assumes IMU is aligned with axis on vehicle and avoid transform
  //Rotate to level
  tf2::Quaternion quat_tf;
  double roll,pitch,yaw;
  tf2::convert(imu.orientation,quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);
  tf2::Matrix3x3 levelMat;
  levelMat.setRPY(roll,pitch,0.0);
  tf2::Vector3 preRotateAccl,rotatedAccel;
  tf2::convert(imu.linear_acceleration,preRotateAccl);
  rotatedAccel = levelMat * preRotateAccl;//Undo the rotation by roll/pitch
 
  angle_tilt = cos(pitch)*cos(roll);
  
  //Already filter and remove
  //This is the long term world acceleration (such as gravity)
  longterm_accel_filter = longterm_accel_filter + accelLongTermAlpha * (rotatedAccel - longterm_accel_filter); 

  //This is the actual acceleration on the vehicle
  shortterm_accel_filter = shortterm_accel_filter + accelAlpha *(rotatedAccel - shortterm_accel_filter); 
  actual_accel_target_frame = -(shortterm_accel_filter - longterm_accel_filter);
  //ROS_INFO_STREAM(roll<<","<<pitch<<","<<actual_accel_target_frame.z()<<","<<shortterm_accel_filter.z()<<","<<longterm_accel_filter.z());
  //Abuse this message type for debugging:
  imu.header.stamp = ros::Time::now();
  tf2::convert(actual_accel_target_frame,imu.linear_acceleration);
  /*
  imu.angular_velocity.x = actual_vel_target_frame.x();
  imu.angular_velocity.y = actual_vel_target_frame.y();
  imu.angular_velocity.z = actual_vel_target_frame.z();
  */
  debug.rotated_accel.x = rotatedAccel.x();
  debug.rotated_accel.y = rotatedAccel.y();
  debug.rotated_accel.z = rotatedAccel.z();
  //accel_pub.publish(imu);
  
  got_accel = true;

}

void DroneFlightControl::in_air_callback(std_msgs::Bool msg){
  got_in_air = true;
  in_air = msg;
}

void DroneFlightControl::filtered_odometry_callback(nav_msgs::Odometry msg){
  if(!got_filtered_odom)
    start_time_ekf = ros::Time::now();

  got_filtered_odom = true;
  filtered_odom = msg;
}

void DroneFlightControl::filtered_state_callback(subt_control_ekf::kfState msg){
  got_filtered_state = true;
  filtered_state = msg;
}

void DroneFlightControl::pixhawk_odom_callback(nav_msgs::Odometry msg){

  got_odom = true;
  px4_odom = msg;
}

void DroneFlightControl::run_local_planner_callback(std_msgs::Bool msg){

  got_run_local_planner = true;
  run_local_planner = msg;
}

void DroneFlightControl::battery_volt_callback(sensor_msgs::BatteryState msg){

  got_battery_volt = true;
  battery_volt = msg.voltage;
}

double DroneFlightControl::thrustToThrottle(double thrust_normIn){	

  // this if for sim
  if(thrust_sim)
    return sqrt( thrust_normIn/g ) * hover_throttle;
  
  // this is for real drone
  else{
    double temp = p1*hover_throttle*hover_throttle + p2*hover_throttle + p3;
    double quad_a = p1, quad_b = p2, quad_c = -temp*thrust_normIn/g + p3;
    return (-quad_b + sqrt( quad_b*quad_b - 4*quad_a*quad_c ) )/(2*quad_a);
  }
  
}

DroneFlightControl::~DroneFlightControl(){
}

BaseNode* BaseNode::get(){
  DroneFlightControl* drone_flight_control = new DroneFlightControl("DroneFlightControl");
  return drone_flight_control;
}
