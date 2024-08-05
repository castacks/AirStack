#include <ros/ros.h>
#include <base/BaseNode.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <core_trajectory_controller/Trajectory.h>
#include <core_trajectory_library/trajectory_library.h>
#include <std_srvs/SetBool.h>
#include <core_trajectory_controller/TrajectoryMode.h>
#include <tflib/tflib.h>

//===================================================================================
//----------------------------- Trajectory Control Node -----------------------------
//===================================================================================

class TrajectoryControlNode : public BaseNode {
private:
  ros::Subscriber traj_sub, traj_track_sub, odom_sub;

  ros::Publisher marker_vis_pub, segment_marker_vis_pub, tracking_point_pub, look_ahead_pub,
    trajectory_completion_percentage_pub, trajectory_time_pub, segment_pub, tracking_error_pub,
    velocity_pub;
  tf::TransformBroadcaster* broadcaster;
  tf::TransformListener* listener;

  ros::ServiceServer traj_style_srv, traj_mode_srv;
  
  nav_msgs::Odometry odom;
  bool got_odom;
  

  std::string tf_prefix;
  std::string target_frame;
  int trajectory_mode;
  Trajectory* trajectory;
  ros::Time start_time;
  double prev_time;
  nav_msgs::Odometry tracking_point, look_ahead_point;
  double tracking_point_distance_limit;
  double velocity_look_ahead_time;
  
public:
  TrajectoryControlNode();
  virtual bool initialize();
  virtual bool execute();
  virtual ~TrajectoryControlNode();

  void traj_callback(core_trajectory_msgs::TrajectoryXYZVYaw traj);
  void traj_track_callback(core_trajectory_msgs::TrajectoryXYZVYaw traj);
  void odom_callback(nav_msgs::Odometry odom);

  bool set_trajectory_style_service(std_srvs::SetBool::Request& req,
				    std_srvs::SetBool::Response& res);
  bool set_trajectory_mode(core_trajectory_controller::TrajectoryMode::Request& req,
			   core_trajectory_controller::TrajectoryMode::Response& res);
  
  std::string mode;
  float velocity_target;
};

TrajectoryControlNode::TrajectoryControlNode()
  : BaseNode("trajectory_control_node"){
}

bool TrajectoryControlNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();
  
  // init params
  tf_prefix = pnh->param("tf_prefix", std::string(""));
  target_frame = pnh->param("target_frame", std::string("world"));
  tracking_point_distance_limit = pnh->param("tracking_point_distance_limit", 0.5);
  velocity_look_ahead_time = pnh->param("velocity_look_ahead_time", 0.0);
  got_odom = false;

  trajectory_mode = core_trajectory_controller::TrajectoryMode::Request::ROBOT_POSE;//TRACK;
  trajectory = new Trajectory(target_frame);
  
  // init subscribers
  traj_sub = nh->subscribe("trajectory", 10, &TrajectoryControlNode::traj_callback, this);
  traj_track_sub = nh->subscribe("trajectory_track", 10, &TrajectoryControlNode::traj_track_callback, this);
  odom_sub = nh->subscribe("odometry", 10, &TrajectoryControlNode::odom_callback, this);
  
  // init publishers
  segment_pub = nh->advertise<core_trajectory_msgs::TrajectoryXYZVYaw>("trajectory_segment", 10);
  marker_vis_pub = nh->advertise<visualization_msgs::MarkerArray>("trajectory_vis", 10);
  segment_marker_vis_pub = nh->advertise<visualization_msgs::MarkerArray>("trajectory_segment_vis", 10);
  tracking_point_pub = nh->advertise<nav_msgs::Odometry>("tracking_point", 10);
  look_ahead_pub = nh->advertise<nav_msgs::Odometry>("look_ahead", 10);
  trajectory_completion_percentage_pub = nh->advertise<std_msgs::Float32>("trajectory_completion_percentage", 10);
  trajectory_time_pub = nh->advertise<std_msgs::Float32>("trajectory_time", 10);
  tracking_error_pub = nh->advertise<std_msgs::Float32>("tracking_error", 10);
  velocity_pub = nh->advertise<std_msgs::Float32>("tracking_point_velocity_magnitude", 1);
  broadcaster = new tf::TransformBroadcaster();
  listener = new tf::TransformListener();

  // init services
  traj_mode_srv = nh->advertiseService("set_trajectory_mode",
				       &TrajectoryControlNode::set_trajectory_mode, this);

  prev_time = 0;
  
  return true;
}

bool TrajectoryControlNode::execute(){
  if(got_odom){
    // publish trajectory visualization
    marker_vis_pub.publish(trajectory->get_markers(1, 1, 0));
  
    // figure out what duration into the trajectory we are
    ros::Time now = ros::Time::now();
    double current_time = (now - start_time).toSec();
    double time_past_end = current_time - trajectory->get_duration();

    double tracking_error = tflib::to_tf(tracking_point.pose.pose.position).distance(tflib::to_tf(odom.pose.pose.position));
    std_msgs::Float32 tracking_error_msg;
    tracking_error_msg.data = tracking_error;
    tracking_error_pub.publish(tracking_error_msg);
    
    if(time_past_end >= 0 && trajectory_mode != core_trajectory_controller::TrajectoryMode::Request::REWIND){
      start_time += ros::Duration(time_past_end);
      current_time = trajectory->get_duration();
    }
    else if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::PAUSE ||
	    tracking_error >= tracking_point_distance_limit){
      start_time += ros::Duration(current_time - prev_time);
      current_time = (now - start_time).toSec();
    }
    // TODO: zero tracking point velocity in rewind and pause modes
    else if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::REWIND &&
	    current_time > 0){
      start_time += ros::Duration(2.0*(current_time - prev_time));
      current_time = (now - start_time).toSec();
    }
    prev_time = current_time;

    // get tracking point and look ahead point. If they aren't valid use the drone's current pose
    //monitor.tic("get_odom");
    bool valid = trajectory->get_odom(current_time, &tracking_point);
    // set the velocity of the tracking point based on the velocity from a different time
    if(valid && velocity_look_ahead_time != 0){
      nav_msgs::Odometry velocity_look_ahead_point;
      bool vel_valid = trajectory->get_odom(current_time + velocity_look_ahead_time, &velocity_look_ahead_point);
      if(vel_valid){
	tracking_point.twist = velocity_look_ahead_point.twist;
      }
    }
      
    //double elapsed = monitor.toc("get_odom");
    //ROS_INFO_STREAM("get_odom elapsed: " << elapsed);
    if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::ROBOT_POSE){
    //if(!valid){
      tracking_point = odom;
      tracking_point.twist.twist.linear.x = 0;
      tracking_point.twist.twist.linear.y = 0;
      tracking_point.twist.twist.linear.z = 0;
      tracking_point.twist.twist.angular.x = 0;
      tracking_point.twist.twist.angular.y = 0;
      tracking_point.twist.twist.angular.z = 0;
      look_ahead_point = tracking_point;
    }
    if(valid){
      trajectory->get_odom(current_time + 1.0, &look_ahead_point);

      tf::Vector3 tp = tflib::to_tf(tracking_point.pose.pose.position);
      double start;
      bool success = trajectory->get_trajectory_distance_at_closest_point(tp, &start);
      if(success){
	Trajectory sub_trajectory = trajectory->get_subtrajectory_distance(start, start+1000.);
	segment_marker_vis_pub.publish(sub_trajectory.get_markers(0, 1, 1));
	core_trajectory_msgs::TrajectoryXYZVYaw segment_msg = sub_trajectory.get_TrajectoryXYZVYaw();
	segment_msg.header.stamp = tracking_point.header.stamp;
	segment_pub.publish(segment_msg);
      }
    }
    
    tracking_point.header.stamp = now;
    look_ahead_point.header.stamp = now;
    
    // TODO: decide whether or not this is a good idea
    // When the tracking point reaches the end of the trajectory, the velocity gets set to zero
    if(time_past_end >= 0 ||
       trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::PAUSE ||
       trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::REWIND){
      tracking_point.twist.twist.linear.x = 0;
      tracking_point.twist.twist.linear.y = 0;
      tracking_point.twist.twist.linear.z = 0;
      tracking_point.twist.twist.angular.x = 0;
      tracking_point.twist.twist.angular.y = 0;
      tracking_point.twist.twist.angular.z = 0;
      
      look_ahead_point.twist.twist.linear.x = 0;
      look_ahead_point.twist.twist.linear.y = 0;
      look_ahead_point.twist.twist.linear.z = 0;
      look_ahead_point.twist.twist.angular.x = 0;
      look_ahead_point.twist.twist.angular.y = 0;
      look_ahead_point.twist.twist.angular.z = 0;
    }

    std_msgs::Float32 velocity_msg;
    velocity_msg.data = sqrt(tracking_point.twist.twist.linear.x*tracking_point.twist.twist.linear.x +
			     tracking_point.twist.twist.linear.y*tracking_point.twist.twist.linear.y +
			     tracking_point.twist.twist.linear.z*tracking_point.twist.twist.linear.z);
    velocity_pub.publish(velocity_msg);
    tracking_point_pub.publish(tracking_point);
    look_ahead_pub.publish(look_ahead_point);
    
    // create a tf for the tracking point odom
    tf::StampedTransform transform = tflib::to_tf(tracking_point, tf_prefix + "/tracking_point");
    tf::StampedTransform transform_stabilized = tflib::get_stabilized(transform);
    transform_stabilized.child_frame_id_ = tf_prefix + "/tracking_point_stabilized";
    broadcaster->sendTransform(transform);
    broadcaster->sendTransform(transform_stabilized);
    
    // create a tf for the look ahead odom
    tf::StampedTransform look_ahead_transform = tflib::to_tf(look_ahead_point, tf_prefix + "/look_ahead_point");
    tf::StampedTransform look_ahead_transform_stabilized =
      tflib::get_stabilized(look_ahead_transform);
    look_ahead_transform_stabilized.child_frame_id_ = tf_prefix + "/look_ahead_point_stabilized";

    broadcaster->sendTransform(look_ahead_transform);
    broadcaster->sendTransform(look_ahead_transform_stabilized);

    // publish completion percentage
    std_msgs::Float32 trajectory_completion_percentage;
    trajectory_completion_percentage.data = current_time/trajectory->get_duration() * 100.f;
    trajectory_completion_percentage_pub.publish(trajectory_completion_percentage);

    // publish current trajectory time
    std_msgs::Float32 trajectory_time;
    trajectory_time.data = current_time;
    trajectory_time_pub.publish(trajectory_time);
  }
  
  return true;
}

bool TrajectoryControlNode::set_trajectory_mode(core_trajectory_controller::TrajectoryMode::Request& req,
						core_trajectory_controller::TrajectoryMode::Response& res){
  int prev_trajectory_mode = trajectory_mode;
  trajectory_mode = req.mode;
  
  if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::PAUSE){
    
  }
  if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::ROBOT_POSE){
    trajectory->clear();
  }
  else if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::TRACK){
    trajectory->clear();
  }
  else if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::SEGMENT){
    if(prev_trajectory_mode != core_trajectory_controller::TrajectoryMode::Request::PAUSE &&
       prev_trajectory_mode != core_trajectory_controller::TrajectoryMode::Request::REWIND &&
       prev_trajectory_mode != core_trajectory_controller::TrajectoryMode::Request::SEGMENT){
      trajectory->clear();
      start_time = ros::Time::now();
    }
  }
  else if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::REWIND){
    
  }
  
  res.success = true;
  return true;
}

void TrajectoryControlNode::traj_callback(core_trajectory_msgs::TrajectoryXYZVYaw traj){
  if(trajectory_mode == core_trajectory_controller::TrajectoryMode::Request::SEGMENT)
    trajectory->merge(Trajectory(traj));
}

void TrajectoryControlNode::traj_track_callback(core_trajectory_msgs::TrajectoryXYZVYaw traj){
  start_time = ros::Time::now();
  trajectory->clear();
  trajectory->merge(Trajectory(traj));
}

void TrajectoryControlNode::odom_callback(nav_msgs::Odometry odom){
  //this->odom = odom;
  if(tflib::transform_odometry(listener, odom, target_frame, target_frame, &(this->odom)))
    got_odom = true;
}

TrajectoryControlNode::~TrajectoryControlNode(){
  
}


BaseNode* BaseNode::get(){
  TrajectoryControlNode* trajectory_control_node = new TrajectoryControlNode();
  return trajectory_control_node;
}
