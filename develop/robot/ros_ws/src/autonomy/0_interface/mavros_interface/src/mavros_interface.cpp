/**
 * @file mavros_interface.cpp
 * @author John Keller (jkeller2@andrew.cmu.edu), Andrew Jong
 * (ajong@andrew.cmu.edu)
 * @brief overrides the RobotInterface class to implement the PX4 flight control
 * interface.
 * @version 0.1
 * @date 2024-07-01
 *
 * @copyright Copyright (c) 2024. This file is developed as part of software
 * from the AirLab at the Robotics Institute at Carnegie Mellon University
 * (https://theairlab.org).
 *
 */
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <airstack_common/ros2_helper.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <robot_interface/robot_interface.hpp>

#include "GeographicLib/Geoid.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace mavros_interface {

  void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
          double &UTMNorthing, double &UTMEasting, char* UTMZone);
  void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const char* UTMZone,
          double& Lat, double& Long);
  char UTMLetterDesignator(double Lat);
  void LLtoSwissGrid(const double Lat, const double Long,
          double &SwissNorthing, double &SwissEasting);
  void SwissGridtoLL(const double SwissNorthing, const double SwissEasting,
          double& Lat, double& Long);
   
  class Ellipsoid {
  public:
   
      Ellipsoid() {
      };
   
      Ellipsoid(int Id, const char* name, double radius, double ecc) {
          id = Id;
          ellipsoidName = name;
          EquatorialRadius = radius;
          eccentricitySquared = ecc;
      }
   
      int id;
      const char* ellipsoidName;
      double EquatorialRadius;
      double eccentricitySquared;
   
  };
   
 const double PI = 3.14159265;
 const double FOURTHPI = PI / 4;
 const double deg2rad = PI / 180;
 const double rad2deg = 180.0 / PI;
  
 static Ellipsoid ellipsoid[] ={//  id, Ellipsoid name, Equatorial Radius, square of eccentricity    
     Ellipsoid(-1, "Placeholder", 0, 0), //placeholder only, To allow array indices to match id numbers
     Ellipsoid(1, "Airy", 6377563, 0.00667054),
     Ellipsoid(2, "Australian National", 6378160, 0.006694542),
     Ellipsoid(3, "Bessel 1841", 6377397, 0.006674372),
     Ellipsoid(4, "Bessel 1841 (Nambia) ", 6377484, 0.006674372),
     Ellipsoid(5, "Clarke 1866", 6378206, 0.006768658),
     Ellipsoid(6, "Clarke 1880", 6378249, 0.006803511),
     Ellipsoid(7, "Everest", 6377276, 0.006637847),
     Ellipsoid(8, "Fischer 1960 (Mercury) ", 6378166, 0.006693422),
     Ellipsoid(9, "Fischer 1968", 6378150, 0.006693422),
     Ellipsoid(10, "GRS 1967", 6378160, 0.006694605),
     Ellipsoid(11, "GRS 1980", 6378137, 0.00669438),
     Ellipsoid(12, "Helmert 1906", 6378200, 0.006693422),
     Ellipsoid(13, "Hough", 6378270, 0.00672267),
     Ellipsoid(14, "International", 6378388, 0.00672267),
     Ellipsoid(15, "Krassovsky", 6378245, 0.006693422),
     Ellipsoid(16, "Modified Airy", 6377340, 0.00667054),
     Ellipsoid(17, "Modified Everest", 6377304, 0.006637847),
     Ellipsoid(18, "Modified Fischer 1960", 6378155, 0.006693422),
     Ellipsoid(19, "South American 1969", 6378160, 0.006694542),
     Ellipsoid(20, "WGS 60", 6378165, 0.006693422),
     Ellipsoid(21, "WGS 66", 6378145, 0.006694542),
     Ellipsoid(22, "WGS-72", 6378135, 0.006694318),
     Ellipsoid(23, "WGS-84", 6378137, 0.00669438)
 };

 /*Reference ellipsoids derived from Peter H. Dana's website- 
 http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
 Department of Geography, University of Texas at Austin
 Internet: pdana@mail.utexas.edu
 3/22/95
  
 Source
 Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
 1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
  */
  
  
  
 void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
         double &UTMNorthing, double &UTMEasting, char* UTMZone) {
     //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
     //East Longitudes are positive, West longitudes are negative. 
     //North latitudes are positive, South latitudes are negative
     //Lat and Long are in decimal degrees
     //Written by Chuck Gantz- chuck.gantz@globalstar.com
  
     double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
     double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
     double k0 = 0.9996;
  
     double LongOrigin;
     double eccPrimeSquared;
     double N, T, C, A, M;
  
     //Make sure the longitude is between -180.00 .. 179.9
     double LongTemp = (Long + 180) - int((Long + 180) / 360)*360 - 180; // -180.00 .. 179.9;
  
     double LatRad = Lat*deg2rad;
     double LongRad = LongTemp*deg2rad;
     double LongOriginRad;
     int ZoneNumber;
  
     ZoneNumber = int((LongTemp + 180) / 6) + 1;
  
     if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
         ZoneNumber = 32;
  
     // Special zones for Svalbard
     if (Lat >= 72.0 && Lat < 84.0) {
         if (LongTemp >= 0.0 && LongTemp < 9.0) ZoneNumber = 31;
         else if (LongTemp >= 9.0 && LongTemp < 21.0) ZoneNumber = 33;
         else if (LongTemp >= 21.0 && LongTemp < 33.0) ZoneNumber = 35;
         else if (LongTemp >= 33.0 && LongTemp < 42.0) ZoneNumber = 37;
     }
     LongOrigin = (ZoneNumber - 1)*6 - 180 + 3; //+3 puts origin in middle of zone
     LongOriginRad = LongOrigin * deg2rad;
  
     //compute the UTM Zone from the latitude and longitude
     sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  
     eccPrimeSquared = (eccSquared) / (1 - eccSquared);
  
     N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
     T = tan(LatRad) * tan(LatRad);
     C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
     A = cos(LatRad)*(LongRad - LongOriginRad);
  
     M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad
             - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad)
             + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad)
             - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));
  
     UTMEasting = (double) (k0 * N * (A + (1 - T + C) * A * A * A / 6
             + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120)
             + 500000.0);
  
     UTMNorthing = (double) (k0 * (M + N * tan(LatRad)*(A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
             + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));
     if (Lat < 0)
         UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
 }
  
 char UTMLetterDesignator(double Lat) {
     //This routine determines the correct UTM letter designator for the given latitude
     //returns 'Z' if latitude is outside the UTM limits of 84N to 80S
     //Written by Chuck Gantz- chuck.gantz@globalstar.com
     char LetterDesignator;
  
     if ((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
     else if ((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
     else if ((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
     else if ((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
     else if ((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
     else if ((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
     else if ((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
     else if ((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
     else if ((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
     else if ((8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
     else if ((0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
     else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
     else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
     else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
     else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
     else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
     else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
     else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
     else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
     else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
     else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits
  
     return LetterDesignator;
 }
  
 void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const char* UTMZone,
         double& Lat, double& Long) {
     //converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
     //East Longitudes are positive, West longitudes are negative. 
     //North latitudes are positive, South latitudes are negative
     //Lat and Long are in decimal degrees. 
     //Written by Chuck Gantz- chuck.gantz@globalstar.com
  
     double k0 = 0.9996;
     double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
     double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
     double eccPrimeSquared;
     double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
     double N1, T1, C1, R1, D, M;
     double LongOrigin;
     double mu, phi1Rad;
     //double phi1;
     double x, y;
     int ZoneNumber;
     char* ZoneLetter;
     //  int NorthernHemisphere; //1 for northern hemispher, 0 for southern
  
     x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
     y = UTMNorthing;
  
     ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
     /*
             if((*ZoneLetter - 'N') >= 0)
                     NorthernHemisphere = 1;//point is in northern hemisphere
             else
             {
                     NorthernHemisphere = 0;//point is in southern hemisphere
                     y -= 10000000.0;//remove 10,000,000 meter offset used for southern hemisphere
             }
      */
     if ((*ZoneLetter - 'N') < 0) {
         y -= 10000000.0; //remove 10,000,000 meter offset used for southern hemisphere
     }
  
     LongOrigin = (ZoneNumber - 1)*6 - 180 + 3; //+3 puts origin in middle of zone
  
     eccPrimeSquared = (eccSquared) / (1 - eccSquared);
  
     M = y / k0;
     mu = M / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256));
  
     phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
             + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu)
             +(151 * e1 * e1 * e1 / 96) * sin(6 * mu);
     // phi1 = phi1Rad*rad2deg;
  
     N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
     T1 = tan(phi1Rad) * tan(phi1Rad);
     C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
     R1 = a * (1 - eccSquared) / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
     D = x / (N1 * k0);
  
     Lat = phi1Rad - (N1 * tan(phi1Rad) / R1)*(D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24
             + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D / 720);
     Lat = Lat * rad2deg;
  
     Long = (D - (1 + 2 * T1 + C1) * D * D * D / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1)
             * D * D * D * D * D / 120) / cos(phi1Rad);
     Long = LongOrigin + Long * rad2deg;
  
 }





  








  
  
class MAVROSInterface : public robot_interface::RobotInterface {
   private:
    // parameters
    bool is_ardupilot;
    float post_takeoff_command_delay_time;
    bool do_global_pose_command;

    bool is_state_received_ = false;
    mavros_msgs::msg::State current_state_;
    bool in_air = false;
    rclcpp::Time in_air_start_time;

    // data from the flight control unit (FCU)
    bool is_yaw_received_ = false;
    float yaw_ = 0.0;

    bool is_home_received_ = false;
    mavros_msgs::msg::HomePosition home_;
    bool estimates_received_ = false;

    rclcpp::CallbackGroup::SharedPtr service_callback_group;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ardupilot_takeoff_client_;

    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_target_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_position_target_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr velocity_target_pub_;
    rclcpp::Publisher<mavros_msgs::msg::HomePosition>::SharedPtr set_home_pub_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odometry_sub_;
  
    message_filters::Subscriber<nav_msgs::msg::Odometry> local_sub_;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> global_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
						    nav_msgs::msg::Odometry, sensor_msgs::msg::NavSatFix>>> sync_;
    nav_msgs::msg::Odometry local_estimate;
    sensor_msgs::msg::NavSatFix global_estimate;
  
    std::shared_ptr<GeographicLib::Geoid> egm96_5;

   public:
    MAVROSInterface() : RobotInterface("mavros_interface") {
        try {
	  // Using smallest dataset with 5' grid,
	  // From default location,
	  // Use cubic interpolation, Thread safe
	  egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);
	} catch (const std::exception & e) {
	  RCLCPP_ERROR_STREAM(this->get_logger(),
			      "UAS: GeographicLib exception: %s " <<
			      "| Run install_geographiclib_dataset.sh script in order to install Geoid Model dataset!" <<
			      e.what());
	  exit(1);
	}
      
        // params
        is_ardupilot = airstack::get_param(this, "is_ardupilot", false);
	post_takeoff_command_delay_time = airstack::get_param(this, "post_takeoff_command_delay_time", 5.);
	do_global_pose_command = airstack::get_param(this, "do_global_pose_command", false);

        // services
        service_callback_group =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "mavros/set_mode", rmw_qos_profile_services_default, service_callback_group);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "mavros/cmd/arming", rmw_qos_profile_services_default, service_callback_group);
        ardupilot_takeoff_client_ = this->create_client<std_srvs::srv::Trigger>(
            "ardupilot_takeoff", rmw_qos_profile_services_default, service_callback_group);

        // publishers
        attitude_target_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "mavros/setpoint_raw/attitude", 1);
        local_position_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 1);
        global_position_target_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
            "mavros/setpoint_position/global", 1);
	velocity_target_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/local", 1);

	rclcpp::QoS qos_profile(1);
	qos_profile.reliable(); 
	set_home_pub_ = this->create_publisher<mavros_msgs::msg::HomePosition>(
            "mavros/home_position/set", qos_profile);

        // subscribers
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 1,
            std::bind(&MAVROSInterface::state_callback, this, std::placeholders::_1));
        home_sub_ = this->create_subscription<mavros_msgs::msg::HomePosition>(
            "mavros/home_position/home", 1,
            std::bind(&MAVROSInterface::home_callback, this, std::placeholders::_1));

	rclcpp::QoS qos = rclcpp::SensorDataQoS();
	local_sub_.subscribe(this, "mavros/global_position/local", qos.get_rmw_qos_profile());
	global_sub_.subscribe(this, "mavros/global_position/global", qos.get_rmw_qos_profile());
	/*
	local_sub_.registerCallback([this](const nav_msgs::msg::Odometry::ConstSharedPtr &msg){
				      RCLCPP_INFO(this->get_logger(), "local timestamp: %f",
						  rclcpp::Time(msg->header.stamp).seconds());
				    });
	
	global_sub_.registerCallback([this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg){
				       RCLCPP_INFO(this->get_logger(), "global timestamp: %f",
						   rclcpp::Time(msg->header.stamp).seconds());
				     });
	*/
	using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
	  nav_msgs::msg::Odometry,
	  sensor_msgs::msg::NavSatFix
	  >;

	using ExactSyncPolicy = message_filters::sync_policies::ExactTime<
	  nav_msgs::msg::Odometry,
	  sensor_msgs::msg::NavSatFix
	  >;

	 sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(100), local_sub_, global_sub_);
	//sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(10), local_sub_, global_sub_);

	sync_->registerCallback(std::bind(&MAVROSInterface::local_global_callback, this, std::placeholders::_1, std::placeholders::_2));
	
    }

    virtual ~MAVROSInterface() {}

  void local_global_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &local,
			     const sensor_msgs::msg::NavSatFix::ConstSharedPtr &global){
    estimates_received_ = true;
    this->local_estimate = *local;
    this->global_estimate = *global;
  }

    // Control Callbacks. Translates commands to fit the MAVROS API.
    // The MAVROS API only has two types of control: Attitude Control and
    // Position Control.

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd) {
        if (!is_ardupilot ||
    	    (in_air && ((this->get_clock()->now() - in_air_start_time).seconds() > post_takeoff_command_delay_time))) {
 	    mavros_msgs::msg::PositionTarget msg;
	    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
	    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
 	        mavros_msgs::msg::PositionTarget::IGNORE_PY |
  	        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
	        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
	        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
	        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
	        mavros_msgs::msg::PositionTarget::IGNORE_YAW;
	    msg.velocity.x = cmd->twist.linear.x;
	    msg.velocity.y = cmd->twist.linear.y;
	    msg.velocity.z = cmd->twist.linear.z;
	    msg.yaw_rate = cmd->twist.angular.z;
	    velocity_target_pub_->publish(msg);
	}
    }

    void attitude_thrust_callback(const mav_msgs::msg::AttitudeThrust::SharedPtr cmd) override {
        mavros_msgs::msg::AttitudeTarget mavros_cmd;
        mavros_cmd.header.stamp = this->get_clock()->now();  //.to_msg();
        mavros_cmd.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                               mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                               mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

        mavros_cmd.thrust = cmd->thrust.z;
        mavros_cmd.orientation = cmd->attitude;

        attitude_target_pub_->publish(mavros_cmd);
    }

    void roll_pitch_yawrate_thrust_callback(
        const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr cmd) override {
        if (!is_yaw_received_) {
            RCLCPP_ERROR(this->get_logger(),
                         "roll_pitch_yawrate_thrust command called but haven't yet "
                         "received drone current yaw");
            return;
        }

        mavros_msgs::msg::AttitudeTarget mavros_cmd;
        mavros_cmd.header.stamp = this->get_clock()->now();  //.to_msg();
        mavros_cmd.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                               mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;
        tf2::Matrix3x3 m;
        m.setRPY(cmd->roll, cmd->pitch, yaw_);
        tf2::Quaternion q;
        m.getRotation(q);
        mavros_cmd.body_rate.z = cmd->yaw_rate;
        mavros_cmd.thrust = cmd->thrust.z;

        mavros_cmd.orientation.x = q.x();
        mavros_cmd.orientation.y = q.y();
        mavros_cmd.orientation.z = q.z();
        mavros_cmd.orientation.w = q.w();

        attitude_target_pub_->publish(mavros_cmd);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr cmd) override {
      // TODO check if home_ has been set
        if (!is_ardupilot ||
            (in_air && ((this->get_clock()->now() - in_air_start_time).seconds() > post_takeoff_command_delay_time))) {

	  if(!do_global_pose_command){
	    geometry_msgs::msg::PoseStamped cmd_copy = *cmd;
	    local_position_target_pub_->publish(cmd_copy);
	  }
	  else{
	    if(!estimates_received_)
	      return;
	    double utm_n, utm_e;
	    char utm_z[8];
	    LLtoUTM(23, global_estimate.latitude, global_estimate.longitude, utm_n, utm_e, utm_z);

	    utm_e += cmd->pose.position.x - local_estimate.pose.pose.position.x;
	    utm_n += cmd->pose.position.y - local_estimate.pose.pose.position.y;
	  
	    double lat, lon;
	    UTMtoLL(23, utm_n, utm_e, utm_z, lat, lon);
	  
	    double alt = global_estimate.altitude + cmd->pose.position.z - local_estimate.pose.pose.position.z +
	      egm96_5->ConvertHeight(lat, lon, 0.0, GeographicLib::Geoid::ELLIPSOIDTOGEOID);

	    geographic_msgs::msg::GeoPoseStamped msg;
	    msg.pose.position.latitude = lat;
	    msg.pose.position.longitude = lon;
	    msg.pose.position.altitude = alt;
	    msg.pose.orientation = cmd->pose.orientation;

	    global_position_target_pub_->publish(msg);
	  
	  
	    /*
	      double alt_offset = egm96_5->ConvertHeight(home_.geo.latitude, home_.geo.longitude, 0.0,
	      GeographicLib::Geoid::ELLIPSOIDTOGEOID);

	      double utm_n, utm_e;
	      char utm_z[8];
	      LLtoUTM(23, home_.geo.latitude, home_.geo.longitude, utm_n, utm_e, utm_z);

	      tf2::Quaternion q(home_.orientation.x, home_.orientation.y, home_.orientation.z, home_.orientation.w);
	      tf2::Vector3 v(cmd->pose.position.x, cmd->pose.position.y, 0.);
	      v = tf2::quatRotate(q.inverse(), v);
	    
	      utm_n += v.x();//cmd->pose.position.y;
	      utm_e += v.y();//cmd->pose.position.x;
	      double alt = home_.geo.altitude + alt_offset + cmd->pose.position.z;

	      double lat, lon;
	      UTMtoLL(23, utm_n, utm_e, utm_z, lat, lon);

	      RCLCPP_INFO_STREAM(this->get_logger(), "home: " << home_.geo.latitude << " " << home_.geo.longitude << " "
	      << utm_n << " " << utm_e << " " << utm_z << " " << lat << " " << lon);

	      geographic_msgs::msg::GeoPoseStamped msg;
	      msg.pose.position.latitude = lat;
	      msg.pose.position.longitude = lon;
	      msg.pose.position.altitude = alt;
	      msg.pose.orientation = home_.orientation;

	      global_position_target_pub_->publish(msg);
	    */
	  }
        }
    }

    // Command Functions

    bool request_control() override {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        if (is_ardupilot)
            request->custom_mode = "GUIDED";  //"OFFBOARD";
        else
            request->custom_mode = "OFFBOARD";

        auto result = set_mode_client_->async_send_request(request);
        std::cout << "waiting rc" << std::endl;
        result.wait();
        std::cout << "done rc" << std::endl;

        return result.get()->mode_sent;
    }

    bool arm() override {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto result = arming_client_->async_send_request(request);
        std::cout << "waiting arm" << std::endl;
        result.wait();
        std::cout << "done arm" << std::endl;

        return result.get()->success;
    }

    bool disarm() override {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto result = arming_client_->async_send_request(request);
        std::cout << "waiting disarm" << std::endl;
        result.wait();
        std::cout << "done disarm" << std::endl;

        return result.get()->success;
    }

    bool is_armed() override { return is_state_received_ && current_state_.armed; }

    bool has_control() override {
        return is_state_received_ &&
               (is_ardupilot ? current_state_.mode == "GUIDED" : current_state_.mode == "OFFBOARD");
    }

    bool takeoff() override {
        if (is_ardupilot) {
	  std_srvs::srv::Trigger::Request::SharedPtr takeoff_request =
	    std::make_shared<std_srvs::srv::Trigger::Request>();

            std::cout << "calling ardupilot takeoff 1" << std::endl;
            auto takeoff_result = ardupilot_takeoff_client_->async_send_request(takeoff_request);
            takeoff_result.wait();
            std::cout << "calling ardupilot takeoff 2" << std::endl;
            if (takeoff_result.get()->success) {
                in_air = true;
                in_air_start_time = this->get_clock()->now();
                return true;
            } else
                return false;
        }

        return true;
    }

    bool land() override {}

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        is_state_received_ = true;
        current_state_ = *msg;
    }

    void home_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg) {
        home_ = *msg;

	if(!is_home_received_){
	  home_.orientation.x = 0.;
	  home_.orientation.y = 0.;
	  home_.orientation.z = 0.;
	  home_.orientation.w = 1.;
	  
	  set_home_pub_->publish(home_);
	}
	
        is_home_received_ = true;
    }
};
}  // namespace mavros_interface
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mavros_interface::MAVROSInterface, robot_interface::RobotInterface)
