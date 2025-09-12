#ifndef _SUBT_CONTROL_EKF_
#define _SUBT_CONTROL_EKF_

#include <base/BaseNode.h>
// #include <sensor_msgs/Imu.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tflib/tflib.h>
#include <subt_control_ekf/EKFControlDebug.h>
#include <subt_control_ekf/kfState.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/console.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <chrono>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/RCOut.h>

using namespace Eigen;

typedef Matrix<double, 9, 9> Matrix9d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 9, 6> Matrix96d;
typedef Matrix<double, 6, 9> Matrix69d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;

class EKFControl: public BaseNode{

private:

	//params
	std::string target_frame;
	double gravity, hover_throttle, hover_throttle_param;
	double p1, p2, p3, p1_volt, p2_volt;
	double init_cov_pos_xy, init_cov_pos_z, init_cov_vel_xy, init_cov_vel_z, init_cov_dis_xy, init_cov_dis_z;
	double model_cov_pos_xy, model_cov_pos_z, model_cov_vel_xy, model_cov_vel_z, model_cov_dis_xy, model_cov_dis_z,
		meas_cov_pos_xy, meas_cov_pos_z, meas_cov_vel_xy, meas_cov_vel_z; 
	double alpha_motor;
	double reject_threshold, attitude_comp_lim, g_lim;
	int sat_rc_limit;

	//variables
	ros::Time prev_ekf_time, start_time_command;
	double x_meas, y_meas, z_meas, yaw_meas, xdot_meas, ydot_meas, zdot_meas;
    double roll_meas, pitch_meas;

    double x_prev, y_prev, z_prev, yaw_prev, xdot_prev, ydot_prev, zdot_prev, roll_comp_prev, pitch_comp_prev, 
    thrust_comp_prev, thrust_achieved_prev, roll_prev, pitch_prev, thrust_in_prev, thrust_command_prev;

    std::string tf_prefix;

    double x_ap, y_ap, z_ap, xdot_ap, ydot_ap, zdot_ap, roll_comp_ap, pitch_comp_ap, thrust_comp_ap;
    subt_control_ekf::EKFControlDebug debug;    
    bool got_odom, got_command, got_in_air, got_px4_odom, got_battery_volt, got_ekf_active, got_rc_out;
    nav_msgs::Odometry odom, prev_odom, px4_odom;
    mav_msgs::RollPitchYawrateThrust command, prev_command;
    std_msgs::Bool in_air, ekf_active;
    mavros_msgs::RCOut rc_out;

    double targetDT, min_dt, min_command_dt;
    double dt;
    int odomCt, commandCt;
    double roll_px4, pitch_px4, roll_px4_odom, pitch_px4_odom;
    double battery_volt;

    // double P_x[2][2], P_y[2][2], P_z[2][2];
    // double Q_x[2][2], Q_y[2][2], Q_z[2][2];
    Matrix3d rot_mat;
    Matrix3d dis_pos;
    Matrix3d dis_vel;
    Matrix3d dis_acc;
    Matrix9d P;
    Matrix9d Q; 
    Matrix6d R;

    Matrix9d A_model; 
    Matrix69d H;

    Vector9d state_ap;
    Vector6d state_meas;

    bool disturb_est_on;
    bool disturb_est_body_frame;
    bool print_flag, thrust_sim;
    int set_attitude;
    tf::TransformBroadcaster* broadcaster;

	//publishers
	ros::Publisher stateOut_pub;
	ros::Publisher debug_pub;
	ros::Publisher  kfState_pub;

	//subscribers
	tf::TransformListener* listener;
	ros::Subscriber odometry_sub, command_sub, in_air_sub, pixhawk_imu_sub, pixhawk_odom_sub;
	ros::Subscriber battery_volt_sub, ekf_active_sub, rc_out_sub;

	//services

	//callbacks
	void odometry_callback(nav_msgs::Odometry msg);
	void command_callback(mav_msgs::RollPitchYawrateThrust msg);
	void in_air_callback(std_msgs::Bool msg);
	// void pixhawk_imu_callback(sensor_msgs::Imu msg);
	void pixhawk_odom_callback(nav_msgs::Odometry msg);
	void battery_volt_callback(sensor_msgs::BatteryState msg);
	void ekf_active_callback(std_msgs::Bool msg);
	void rc_out_callback(mavros_msgs::RCOut msg);

	void calcAP( double dtIn, double thrust_In, double thrust_In_prev, double& x_ap_,
	  double& y_ap_, double& z_ap_, double& xdot_ap_, double& ydot_ap_, double& zdot_ap_, double roll_comp_ap_, 
	  double pitch_comp_ap_, double thrust_comp_ap_, double roll, double pitch, double yaw);

	double throttleToThrust(double throttle);

public:
	EKFControl(std::string node_name);

	virtual bool initialize();
	virtual bool execute();
	virtual ~EKFControl();
};


#endif
