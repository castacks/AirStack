#include "subt_control_ekf/subt_control_ekf.h"
#include <base/BaseNode.h>
#include <string>
#include <vector>
#include <math.h>

// using namespace Eigen;

// #define N_dof 6

EKFControl::EKFControl(std::string node_name): BaseNode(node_name){}

bool EKFControl::initialize(){

	ros::NodeHandle* nh = get_node_handle();
	ros::NodeHandle* pnh = get_private_node_handle();

	// init params
	tf_prefix = pnh->param("tf_prefix", std::string(""));
  target_frame = pnh->param("target_frame", std::string("map"));
  min_dt = pnh->param("min_dt", 0.01);
  min_command_dt = pnh->param("min_command_dt", 0.01);
	gravity =pnh->param("gravity", 9.81);
  hover_throttle_param =pnh->param("hover_throttle", 0.66);
  p1 = pnh->param("p1", 24.28); p2 = pnh->param("p2", 6.287); p3 = pnh->param("p3", -1.844);
  targetDT = 1.0/pnh->param("execute_target", 50);
  init_cov_pos_xy = pnh->param("init_cov_pos_xy", 1e-3);
  init_cov_pos_z = pnh->param("init_cov_pos_z", 1e-3);
  init_cov_vel_xy = pnh->param("init_cov_vel_xy", 1e-2);
  init_cov_vel_z = pnh->param("init_cov_vel_z", 1e-2);
  init_cov_dis_xy = pnh->param("init_cov_dis_xy", 1e-4); // 2
  init_cov_dis_z = pnh->param("init_cov_dis_z", 1e-4);
  model_cov_pos_xy = pnh->param("model_cov_pos_xy", 1e-6); //3
  model_cov_pos_z = pnh->param("model_cov_pos_z", 1e-6);
  model_cov_vel_xy = pnh->param("model_cov_vel_xy", 1e-6); // 2
  model_cov_vel_z = pnh->param("model_cov_vel_z", 1e-6);
  model_cov_dis_xy = pnh->param("model_cov_dis_xy", 1e-6); // 2
  model_cov_dis_z = pnh->param("model_cov_dis_z", 1e-6);
  meas_cov_pos_xy = pnh->param("meas_cov_pos_xy", 1e-3);
  meas_cov_pos_z = pnh->param("meas_cov_pos_z", 1e-3);
  meas_cov_vel_xy = pnh->param("meas_cov_vel_xy", 1e-2);
  meas_cov_vel_z = pnh->param("meas_cov_vel_z", 1e-2);
  disturb_est_on = pnh->param("disturb_est_on", true);
  disturb_est_body_frame = pnh->param("disturb_est_body_frame", false);
  print_flag = pnh->param("print_flag", false);
  thrust_sim = pnh->param("thrust_sim", false);
  set_attitude = pnh->param("set_attitude", 0);
  alpha_motor = pnh->param("alpha_motor", 1.0);
  p1_volt = pnh->param("p1_volt", 1.0);
  p2_volt = pnh->param("p2_volt", 1.0);
  reject_threshold = pnh->param("reject_threshold", 3.0);
  attitude_comp_lim = pnh->param("attitude_comp_lim", 10.0);
  g_lim = pnh->param("g_lim", 0.5);
  sat_rc_limit = pnh->param("sat_rc_limit", 1900);

  // init variables
  odomCt=0; commandCt=0;
  in_air.data = false;
  ekf_active.data = false;

  // roll_prev=0; pitch_prev=0; thrust_prev=0;
  got_odom = false; got_command=false; got_in_air = false; got_battery_volt = false; got_ekf_active = false;
  // prev_odom_time = odom.header.stamp;
  prev_ekf_time = odom.header.stamp; // ros::Time::now();
  // prev_odom = odom; prev_command = command;
  prev_command.roll = 0.0; prev_command.pitch = 0.0; prev_command.thrust.z = 0.0;  

  H <<1., 0., 0., 0., 0., 0., 0., 0., 0.,
      0., 1., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 1., 0., 0., 0., 0., 0., 0., 
      0., 0., 0., 1., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 1., 0., 0., 0.; 

  P << init_cov_pos_xy, 0., 0., 0., 0., 0., 0., 0., 0.,
      0., init_cov_pos_xy, 0., 0., 0., 0., 0., 0., 0.,
      0., 0., init_cov_pos_z, 0., 0., 0., 0., 0., 0.,
      0., 0., 0., init_cov_vel_xy, 0., 0., 0., 0., 0.,
      0., 0., 0., 0., init_cov_vel_xy, 0., 0., 0., 0.,
      0., 0., 0., 0., 0., init_cov_vel_z, 0., 0., 0.,
      0., 0., 0., 0., 0., 0., init_cov_dis_xy, 0., 0.,
      0., 0., 0., 0., 0., 0., 0., init_cov_dis_xy, 0.,
      0., 0., 0., 0., 0., 0., 0., 0., init_cov_dis_z;

  Q << model_cov_pos_xy, 0., 0., 0., 0., 0., 0., 0., 0.,
      0., model_cov_pos_xy, 0., 0., 0., 0., 0., 0., 0.,
      0., 0., model_cov_pos_z, 0., 0., 0., 0., 0., 0.,
      0., 0., 0., model_cov_vel_xy, 0., 0., 0., 0., 0.,
      0., 0., 0., 0., model_cov_vel_xy, 0., 0., 0., 0.,
      0., 0., 0., 0., 0., model_cov_vel_z, 0., 0., 0.,
      0., 0., 0., 0., 0., 0., model_cov_dis_xy, 0., 0.,
      0., 0., 0., 0., 0., 0., 0., model_cov_dis_xy, 0.,
      0., 0., 0., 0., 0., 0., 0., 0., model_cov_dis_z;

  R << meas_cov_pos_xy, 0., 0., 0., 0., 0.,
      0., meas_cov_pos_xy, 0., 0., 0., 0., 
      0., 0., meas_cov_pos_z, 0., 0., 0., 
      0., 0., 0., meas_cov_vel_xy, 0., 0.,
      0., 0., 0., 0., meas_cov_vel_xy, 0.,
      0., 0., 0., 0., 0., meas_cov_vel_z;

  // try{
  //   tf::StampedTransform transform;
  //   listener->waitForTransform(target_frame, odom.header.frame_id, odom.header.stamp, ros::Duration(0.1));
  //   ROS_INFO_STREAM("wait for velo transform done\n");
  //   listener->lookupTransform(target_frame, odom.header.frame_id, odom.header.stamp, transform);
  //   ROS_INFO_STREAM("look up velo transform done\n");
  //   tf::StampedTransform velocity_transform;
  //   listener->waitForTransform(target_frame, odom.child_frame_id, odom.header.stamp, ros::Duration(0.1));
  //   listener->lookupTransform(target_frame, odom.child_frame_id, odom.header.stamp, velocity_transform);
  //   velocity_transform.setOrigin(tf::Vector3(0,
  //   tf::Vector3 position_target_frame = transform*tflib::to_tf(odom.pose.pose.position);
  //   tf::Quaternion q_target_frame = transform*tflib::to_tf(odom.pose.pose.orientation);
  //   x_prev = position_target_frame.x();
  //   y_prev = position_target_frame.y();
  //   z_prev = position_target_frame.z();
  //   tf::Matrix3x3(q_target_frame).getRPY(roll_prev, pitch_prev, yaw_prev);

  //   tf::Vector3 velocity_target_frame = velocity_transform*tflib::to_tf(odom.twist.twist.linear);
  //   xdot_prev = velocity_target_frame.x();
  //   ydot_prev = velocity_target_frame.y();
  //   zdot_prev = velocity_target_frame.z();
  // }
  // catch(tf::TransformException& te){
  //   ROS_ERROR_STREAM("TransformException while transform odometry in initialize: " << te.what());
  //   return true;
  // }

  // ^ TODO may not work in bagfile, can comment out and init prev to 0
  // see below v

  x_prev = 0.0; y_prev = 0.0; z_prev = 0.0;
  xdot_prev = 0.0; ydot_prev = 0.0; zdot_prev = 0.0;
  roll_prev=0.0; roll_comp_prev=0.0;
  pitch_prev=0.0; pitch_comp_prev=0.0;
  thrust_comp_prev = -1.0;
  roll_px4_odom=0.0; pitch_px4_odom = 0.0;
  hover_throttle = hover_throttle_param;
	thrust_achieved_prev = 0.; thrust_command_prev = throttleToThrust(hover_throttle); thrust_in_prev = 0.; 
	prev_ekf_time = ros::Time::now(); start_time_command = ros::Time::now();
  battery_volt = 16.0;

  // init publishers
  stateOut_pub = nh->advertise<nav_msgs::Odometry>("filtered_odom_control", 1);
  debug_pub = nh->advertise<subt_control_ekf::EKFControlDebug>("subt_control_ekf_debug", 1);
  kfState_pub = nh->advertise<subt_control_ekf::kfState>("subt_control_ekf_state", 1);

  // init subscribers
  listener = new tf::TransformListener();
  broadcaster = new tf::TransformBroadcaster();
  odometry_sub = nh->subscribe("odometry", 1, &EKFControl::odometry_callback, this, ros::TransportHints().tcpNoDelay());
  command_sub = nh->subscribe("roll_pitch_yawrate_thrust_command", 1, &EKFControl::command_callback, this);
  in_air_sub = nh->subscribe("in_air", 1, &EKFControl::in_air_callback, this);
  // pixhawk_imu_sub = nh->subscribe("mavros/imu/data", 1, &EKFControl::pixhawk_imu_callback, this);
  pixhawk_odom_sub = nh->subscribe("mavros/local_position/odom", 1, &EKFControl::pixhawk_odom_callback, this, ros::TransportHints().tcpNoDelay());
  battery_volt_sub = nh->subscribe("mavros/battery", 1, &EKFControl::battery_volt_callback, this);
  ekf_active_sub = nh->subscribe("ekf_active", 1, &EKFControl::ekf_active_callback, this);
  rc_out_sub = nh->subscribe("mavros/rc/out", 1, &EKFControl::rc_out_callback, this);

  return true;
}

bool EKFControl::execute(){

  ros::Time start_time = ros::Time::now();
  double now = start_time.toSec();
  double prev_time = start_time.toSec();

  if(got_battery_volt && !thrust_sim)
    hover_throttle = p1_volt * battery_volt + p2_volt;
  else
    hover_throttle = hover_throttle_param;
  
  if(got_odom && odom.header.seq != prev_odom.header.seq){

    ros::Time curr_time = ros::Time::now();
    // ROS_INFO_STREAM("Time according to odom is "<<odom.header.stamp.toSec()<<", Time according to ROS is "<<ros::Time::now().toSec());

    // TODO what's going on here?

  	// // double dt = (curr_time - prev_time).toSec();
   //  // double command_dt = (command.header.stamp - prev_command_time).toSec();
  	// prev_time = curr_time; 
    // prev_command_time = command.header.stamp;

    // if(dt < min_dt || command_dt < min_command_dt )
    //   return true

    double unused;
    try{
      
      //monitor.tic("px4tf");
      tf::StampedTransform transform_px4;
      //listener->waitForTransform(target_frame, px4_odom.header.frame_id, px4_odom.header.stamp, ros::Duration(0.1));
      //listener->lookupTransform(target_frame, px4_odom.header.frame_id, px4_odom.header.stamp, transform_px4);
      listener->lookupTransform(target_frame, px4_odom.header.frame_id, ros::Time(0), transform_px4);
      //ROS_INFO_STREAM("elapsed: " << monitor.toc("px4tf")/1000000.);
      
      tf::Quaternion px4_q_target_frame = transform_px4*tflib::to_tf(px4_odom.pose.pose.orientation); 
      tf::Matrix3x3(px4_q_target_frame).getRPY(roll_px4_odom, pitch_px4_odom, unused);
      
      //monitor.tic("other4tf");
      auto start1 = std::chrono::steady_clock::now();
      tf::StampedTransform transform;
      listener->waitForTransform(target_frame, odom.header.frame_id, odom.header.stamp, ros::Duration(0.1));
      listener->lookupTransform(target_frame, odom.header.frame_id, odom.header.stamp, transform);
      tf::StampedTransform velocity_transform;
      listener->waitForTransform(target_frame, odom.child_frame_id, odom.header.stamp, ros::Duration(0.1));
      listener->lookupTransform(target_frame, odom.child_frame_id, odom.header.stamp, velocity_transform);
      velocity_transform.setOrigin(tf::Vector3(0, 0, 0));      
      //ROS_INFO_STREAM("elapsed: " << monitor.toc("other4tf")/1000000.);
      
      tf::Vector3 position_target_frame = transform*tflib::to_tf(odom.pose.pose.position);
      tf::Quaternion q_target_frame = transform*tflib::to_tf(odom.pose.pose.orientation);
      x_meas = position_target_frame.x();
      y_meas = position_target_frame.y();
      z_meas = position_target_frame.z();
      tf::Matrix3x3(q_target_frame).getRPY(roll_meas, pitch_meas, yaw_meas);
      
      tf::Vector3 velocity_target_frame = velocity_transform*tflib::to_tf(odom.twist.twist.linear);
      xdot_meas = velocity_target_frame.x();
      ydot_meas = velocity_target_frame.y();
      zdot_meas = velocity_target_frame.z();
      // if (zdot_meas > 6.0 ) {
      //   zdot_meas = 6.0;
      // }
      // else if (zdot_meas < -30.0) {
      //   zdot_meas = -30.0;
      // }      
      // ROS_INFO_STREAM("transform time: " <<elapsed_seconds1.count());
    }
    catch(tf::TransformException& te){
      ROS_ERROR_STREAM("TransformException while transform odometry in execute: " << te.what());
      return true;
    }

    // ** find a-priori estimate ** //
    x_ap = x_prev; y_ap = y_prev; z_ap = z_prev;
    xdot_ap = xdot_prev; ydot_ap = ydot_prev; zdot_ap = zdot_prev;
    roll_comp_ap = roll_comp_prev; pitch_comp_ap = pitch_comp_prev; thrust_comp_ap = thrust_comp_prev;   

    if(in_air.data){
      // auto start2 = std::chrono::steady_clock::now();

      double dt_ekf = (ros::Time::now() - prev_ekf_time).toSec();
      
      double pitch_in;
      double roll_in; 
      double yaw_in = yaw_meas; 

      // if(set_attitude==0){
      //   pitch_in = (pitch_prev + prev_command.pitch)/2.0;
      //   roll_in = (roll_prev + prev_command.roll)/2.0;
      // }
      if(set_attitude==1){
        pitch_in = (pitch_prev + pitch_meas)/2.0;
        roll_in = (roll_prev + roll_meas)/2.0;
      }
      else if(set_attitude==2){
        pitch_in = (pitch_prev + pitch_px4_odom)/2.0;
        roll_in = (roll_prev + roll_px4_odom)/2.0;
      }

      double dt_command = (ros::Time::now() - start_time_command).toSec();
      double thrust_in = thrust_achieved_prev + (thrust_command_prev - thrust_achieved_prev)*(1 - exp(-alpha_motor*dt_command));

      calcAP( dt_ekf, thrust_in, thrust_in_prev, x_ap, y_ap, z_ap, xdot_ap, ydot_ap, zdot_ap, roll_comp_ap, pitch_comp_ap, 
      	thrust_comp_ap, roll_in, pitch_in, yaw_in );      
      
      /// ***Kalman Filter*** ///
      if (prev_ekf_time.toSec() == 0.0) {
        prev_ekf_time = curr_time;
      }


      if (disturb_est_on && disturb_est_body_frame){

        A_model << 1., 0., 0., dt_ekf, 0., 0., -gravity*pow(dt_ekf, 2)/2.0*sin(yaw_in), -gravity*pow(dt_ekf, 2)/2.0*cos(yaw_in), 0.,
                  0., 1., 0., 0., dt_ekf, 0., gravity*pow(dt_ekf, 2)/2.0*cos(yaw_in), -gravity*pow(dt_ekf, 2)/2.0*sin(yaw_in), 0.,
                  0., 0., 1., 0., 0., dt_ekf, 0., 0., -gravity*pow(dt_ekf, 2)/2.0,
                  0., 0., 0., 1., 0., 0., -gravity*dt_ekf*sin(yaw_in), -gravity*dt_ekf*cos(yaw_in), 0.,
                  0., 0., 0., 0., 1., 0., gravity*dt_ekf*cos(yaw_in), -gravity*dt_ekf*sin(yaw_in), 0.,
                  0., 0., 0., 0., 0., 1., 0., 0., -gravity*dt_ekf,
                  0., 0., 0., 0., 0., 0., 1., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 1., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 1.;
      }

      else if (disturb_est_on && not disturb_est_body_frame) {

        A_model << 1., 0., 0., dt_ekf, 0., 0., -pow(dt_ekf, 2)/2., 0., 0.,
                  0., 1., 0., 0., dt_ekf, 0., 0., -pow(dt_ekf, 2)/2., 0.,
                  0., 0., 1., 0., 0., dt_ekf, 0., 0., -gravity*pow(dt_ekf, 2)/2.,
                  0., 0., 0., 1., 0., 0., -dt_ekf, 0., 0.,
                  0., 0., 0., 0., 1., 0., 0., -dt_ekf, 0.,
                  0., 0., 0., 0., 0., 1., 0., 0., -gravity*dt_ekf,
                  0., 0., 0., 0., 0., 0., 1., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 1., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 1.;
      }

      else{
        A_model << 1., 0., 0., dt_ekf, 0., 0., 0., 0., 0.,
                  0., 1., 0., 0., dt_ekf, 0., 0., 0., 0.,
                  0., 0., 1., 0., 0., dt_ekf, 0., 0., 0.,
                  0., 0., 0., 1., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 1., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 1., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0.;
      }

      P = A_model * P * A_model.transpose() + Q;

      Matrix6d S = H * P * H.transpose() + R;
      Matrix96d K = P * H.transpose() * S.inverse();

      state_ap(0) = x_ap; state_ap(1) = y_ap; state_ap(2) = z_ap;
      state_ap(3) = xdot_ap; state_ap(4) = ydot_ap; state_ap(5) = zdot_ap;
      state_ap(6) = roll_comp_ap; state_ap(7) = pitch_comp_ap; state_ap(8) = thrust_comp_ap;
      state_meas(0) = x_meas; state_meas(1) = y_meas; state_meas(2) = z_meas;
      state_meas(3) = xdot_meas; state_meas(4) = ydot_meas; state_meas(5) = zdot_meas;
      Vector6d innovation = state_meas - H * state_ap;
      Vector9d state_out;

      double sqMahanolobis = innovation.dot(S.inverse() * innovation);
      // ROS_INFO_STREAM(" S inverse is "<<S.inverse()<< ", innovation is "<<innovation);
      // ROS_INFO_STREAM("Sq Mahanolobis dist is "<<sqMahanolobis);

      bool saturated=false;
      if( rc_out.channels[0] >= sat_rc_limit || rc_out.channels[1] >= sat_rc_limit  ||
        rc_out.channels[2] >= sat_rc_limit || rc_out.channels[3] >= sat_rc_limit )        
        saturated = true;

      if( !saturated && ekf_active.data && (sqMahanolobis >= reject_threshold*reject_threshold) ){

      	state_out = state_ap;
      	ROS_INFO_STREAM("Outlier measurement rejected. Sq Mahanolobis is "<< sqMahanolobis);
      }
      else{
      	state_out = state_ap + K * innovation;
      	P = P - K * H * P;
      }

      double radian_lim  = attitude_comp_lim*3.1415/180.0;
      state_out(6) = std::min(std::max( state_out(6), -radian_lim ), radian_lim);
      state_out(7) = std::min(std::max( state_out(7), -radian_lim ), radian_lim);
      state_out(8) = std::min(std::max( state_out(8), -g_lim ), g_lim);

      double xOut = state_out(0), xdotOut = state_out(3);
      double yOut = state_out(1), ydotOut = state_out(4);
      double zOut = state_out(2), zdotOut = state_out(5);
      double roll_compOut = state_out(6);
      double pitch_compOut = state_out(7);
      double thrust_compOut = state_out(8);

      // set previous //

      // if(set_attitude==0){
      //   pitch_prev = postCommandBuffer.back().pitch;
      //   roll_prev = postCommandBuffer.back().roll;
      // }
      if(set_attitude==1){
        pitch_prev = pitch_meas;
        roll_prev = roll_meas;
      }
      else if(set_attitude==2){
        pitch_prev = pitch_px4_odom;
        roll_prev = roll_px4_odom;
      }

      x_prev = xOut; y_prev = yOut; z_prev = zOut;
      xdot_prev = xdotOut; ydot_prev = ydotOut; zdot_prev = zdotOut;
      roll_comp_prev=roll_compOut; pitch_comp_prev=pitch_compOut; thrust_comp_prev=thrust_compOut;

      prev_ekf_time = ros::Time::now();
      thrust_in_prev = thrust_in;
      prev_odom = odom;

      //publish debug info
      debug.header = odom.header;
      debug.positionOut.x = xOut;
      debug.positionOut.y = yOut;
      debug.positionOut.z = zOut;
      debug.velOut.x = xdotOut;
      debug.velOut.y = ydotOut;
      debug.velOut.z = zdotOut;
      debug.disOut.x = roll_compOut;
      debug.disOut.y = pitch_compOut;
      debug.disOut.z = thrust_compOut;

      debug.position_ap.x = x_ap;
      debug.position_ap.y = y_ap;
      debug.position_ap.z = z_ap;
      debug.vel_ap.x = xdot_ap;
      debug.vel_ap.y = ydot_ap;
      debug.vel_ap.z = zdot_ap;
      debug.dis_ap.x = roll_comp_ap;
      debug.dis_ap.y = pitch_comp_ap;
      debug.dis_ap.z = thrust_comp_ap;

      debug.position_meas.x = x_meas;
      debug.position_meas.y = y_meas;
      debug.position_meas.z = z_meas;
      debug.vel_meas.x = xdot_meas;
      debug.vel_meas.y = ydot_meas;
      debug.vel_meas.z = zdot_meas;  

      debug.roll_meas = roll_meas;
      debug.pitch_meas = pitch_meas;
      debug.yaw_meas = yaw_meas;
      // debug.roll_px4 = roll_px4;
      // debug.pitch_px4 = pitch_px4;
      debug.roll_px4_odom = roll_px4_odom;
      debug.pitch_px4_odom = pitch_px4_odom;

      debug.model_cov_pos_xy = model_cov_pos_xy;
      debug.model_cov_pos_z = model_cov_pos_z;
      debug.model_cov_vel_xy = model_cov_vel_xy;
      debug.model_cov_vel_z = model_cov_vel_z;

      debug.model_cov_dis_xy= model_cov_dis_xy;
      debug.model_cov_dis_z= model_cov_dis_z;

      debug.meas_cov_pos_xy = meas_cov_pos_xy;
      debug.meas_cov_pos_z = meas_cov_pos_z;
      debug.meas_cov_vel_xy = meas_cov_vel_xy;
      debug.meas_cov_vel_z = meas_cov_vel_z;
      
      debug.thrust_in = thrust_in;

      debug_pub.publish(debug);
      // ROS_INFO_STREAM("Published Debug");

      nav_msgs::Odometry stateOut;
      stateOut.header.stamp = odom.header.stamp; //  ros::Time::now() in actual system;
      stateOut.header.frame_id = odom.header.frame_id;
      stateOut.pose.pose.position.x = xOut;
      stateOut.pose.pose.position.y = yOut;
      stateOut.pose.pose.position.z = zOut;

      // stateOut.child_frame_id = odom.child_frame_id;
      stateOut.child_frame_id = odom.header.frame_id;      
      stateOut.twist.twist.linear.x = xdotOut;
      stateOut.twist.twist.linear.y = ydotOut;
      stateOut.twist.twist.linear.z = zdotOut;

      stateOut.twist.twist.angular = odom.twist.twist.angular;
      stateOut.pose.pose.orientation = odom.pose.pose.orientation;

      stateOut_pub.publish(stateOut);

      // publish Kalman Filter state
      subt_control_ekf::kfState augmentedState;
      augmentedState.header = odom.header;

      augmentedState.disOut.x = roll_compOut;
      augmentedState.disOut.y = pitch_compOut;
      augmentedState.disOut.z = thrust_compOut;

      kfState_pub.publish(augmentedState);

      // create tf for filtered_state
      tf::StampedTransform transform = tflib::to_tf(stateOut, tf_prefix + "/filtered_odom");
      tf::StampedTransform transform_stabilized = tflib::get_stabilized(transform);
      transform_stabilized.child_frame_id_ = tf_prefix + "/filtered_odom_stabilized";
      broadcaster->sendTransform(transform);
      broadcaster->sendTransform(transform_stabilized);

    }
  }

  //monitor.print_time_statistics();
  
  return true;
}


void EKFControl::odometry_callback(nav_msgs::Odometry msg){
  got_odom = true;
  odom = msg;
}

void EKFControl::command_callback(mav_msgs::RollPitchYawrateThrust msg){
  got_command = true;
  // command = msg;
  ros::Time time_curr = ros::Time::now();
  double dt_temp_command = (time_curr - start_time_command).toSec();
  thrust_achieved_prev += (thrust_command_prev - thrust_achieved_prev)*(1 - exp(-alpha_motor*dt_temp_command));
  thrust_command_prev = throttleToThrust(msg.thrust.z);
  start_time_command = time_curr;
}

void EKFControl::in_air_callback(std_msgs::Bool msg){
  got_in_air = true;
  in_air = msg;
}

// void EKFControl::pixhawk_imu_callback(sensor_msgs::Imu msg){

//   tf::Matrix3x3 m(tf::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
//   double yaw;
//   m.getRPY(roll_px4, pitch_px4, yaw);
// }

void EKFControl::pixhawk_odom_callback(nav_msgs::Odometry msg){

  got_odom = true;
  px4_odom = msg;
}

void EKFControl::battery_volt_callback(sensor_msgs::BatteryState msg){

  got_battery_volt = true;
  battery_volt = msg.voltage;
}

void EKFControl::ekf_active_callback(std_msgs::Bool msg){
  got_ekf_active = true;
  ekf_active = msg;
}

void EKFControl::rc_out_callback(mavros_msgs::RCOut msg){
  got_rc_out = true;
  rc_out = msg;
}

void EKFControl::calcAP( double dtIn, double thrust_In, double thrust_In_prev, double& x_ap_,
  double& y_ap_, double& z_ap_, double& xdot_ap_, double& ydot_ap_, double& zdot_ap_, double roll_comp_ap_, 
  double pitch_comp_ap_, double thrust_comp_ap_, double roll, double pitch, double yaw){

  // incorporate thrust delay
	// double thrust_eff_av = thrust_eff + (thrust_prev_running - thrust_eff) * (1 - pow(e, (-alpha_motor * dtIn)) )/alpha_motor;
	// thrust_prev_running += (thrust_eff - thrust_prev_running) * (1 - exp(-alpha_motor*dtIn));

	double thrust_eff_av = (thrust_In + thrust_In_prev)/2.0;// - thrust_comp_ap_*gravity;
	double roll_eff = roll; double pitch_eff = pitch;
	if(disturb_est_on && disturb_est_body_frame){
		roll_eff += -roll_comp_ap_;
		pitch_eff += -pitch_comp_ap_;
	} 

  double a_x_comm, a_y_comm, a_z_comm;
  a_x_comm = thrust_eff_av*(cos(yaw)*sin(pitch_eff)*cos(roll_eff) + sin(yaw)*sin(roll_eff));
  a_y_comm = thrust_eff_av*(sin(yaw)*sin(pitch_eff)*cos(roll_eff) - cos(yaw)*sin(roll_eff));
  a_z_comm = - gravity + thrust_eff_av*(cos(pitch_eff)*cos(roll_eff));    

  if(disturb_est_on && !disturb_est_body_frame){
  	a_x_comm += -roll_comp_ap_;
  	a_y_comm += -pitch_comp_ap_;
  }
  if(disturb_est_on) 
  	a_z_comm += -thrust_comp_ap_ * gravity;

  // this is Ax+Bu (pretty sure need to use yaw_meas)
  x_ap_ += xdot_ap_*dtIn + 0.5*a_x_comm*dtIn*dtIn;
  y_ap_ += ydot_ap_*dtIn + 0.5*a_y_comm*dtIn*dtIn;
  z_ap_ += zdot_ap_*dtIn + 0.5*a_z_comm*dtIn*dtIn;// - 0.5*gravity*dtIn*dtIn*thrust_comp_ap_;

  xdot_ap_ += dtIn*( a_x_comm ); 
  ydot_ap_ += dtIn*( a_y_comm ); 
  zdot_ap_ += dtIn*( a_z_comm );// - gravity*dtIn*thrust_comp_ap_;

  // if (disturb_est_body_frame) {
  //   x_ap_ += -0.5*gravity*dtIn*dtIn*sin(yaw_meas)*roll_comp_ap_ - 0.5*gravity*dtIn*dtIn*cos(yaw_meas)*pitch_comp_ap_;
  //   y_ap_ += 0.5*gravity*dtIn*dtIn*cos(yaw_meas)*roll_comp_ap_ - 0.5*gravity*dtIn*dtIn*sin(yaw_meas)*pitch_comp_ap_;
  // }
  // else {
  //   x_ap_ -= 0.5*dtIn*dtIn*roll_comp_ap_;
  //   y_ap_ -= 0.5*dtIn*dtIn*pitch_comp_ap_;
  // }  
  // if (disturb_est_body_frame) {
  //   xdot_ap_ += -gravity*dtIn*sin(yaw_meas)*roll_comp_ap_ - gravity*dtIn*cos(yaw_meas)*pitch_comp_ap_;
  //   ydot_ap_ += gravity*dtIn*cos(yaw_meas)*roll_comp_ap_ - gravity*dtIn*sin(yaw_meas)*pitch_comp_ap_;
  // }
  // else {
  //   xdot_ap_ -= dtIn*roll_comp_ap_;
  //   ydot_ap_ -= dtIn*pitch_comp_ap_;
  // }  

  // TODO may not need to multiply by gravity above since you multiply by gravity in A but idk, it may matter for the controller

  // roll_comp_ap, pitch_comp_ap, and thrust_comp_ap are just their prev values

  // ROS_INFO_STREAM("exiting calcAP");
}

double EKFControl::throttleToThrust(double throttle){

  if(thrust_sim)
    return gravity* (throttle / hover_throttle)*(throttle / hover_throttle);
  else
    return gravity * ( p1*throttle*throttle + p2*throttle + p3 ) 
                / ( p1*(hover_throttle)*(hover_throttle) + p2*(hover_throttle) + p3 ) ;
}


EKFControl::~EKFControl(){}


BaseNode* BaseNode::get(){
  EKFControl* subt_control_ekf = new EKFControl("EKFControl");
  return subt_control_ekf;
}
