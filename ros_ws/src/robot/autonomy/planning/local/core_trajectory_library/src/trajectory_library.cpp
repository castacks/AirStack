#include <ros/ros.h>
#include <core_trajectory_library/trajectory_library.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <std_msgs/ColorRGBA.h>
#include <tflib/tflib.h>

Waypoint::Waypoint(double x, double y, double z, double yaw,
		   double vx, double vy, double vz, double time){
  x_ = x;
  y_ = y;
  z_ = z;
  vx_ = vx;
  vy_ = vy;
  vz_ = vz;
  yaw_ = yaw;
  time_ = time;
}

tf::Quaternion Waypoint::q() const {
  tf::Quaternion quat;
  quat.setRPY(0, 0, yaw_);
  return quat;
}

tf::Vector3 Waypoint::position() const {
  return tf::Vector3(x_, y_, z_);
}

tf::Vector3 Waypoint::velocity() const {
  return tf::Vector3(vx_, vy_, vz_);
}


nav_msgs::Odometry Waypoint::odometry(ros::Time stamp, std::string frame_id) const {
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = frame_id;
  
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = z_;
  
  tf::Quaternion quat = q();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();

  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.linear.z = vz_;

  return odom;
}

Waypoint Waypoint::interpolate(Waypoint wp, double t){
  tf::Vector3 pos_interp = position() + t*(wp.position() - position());
  tf::Quaternion q_interp = q().slerp(wp.q(), t);
  tf::Vector3 vel_interp = velocity() + t*(wp.velocity() - velocity());
  double time_interp = time() + t*(time() - wp.time());

  Waypoint wp_interp(pos_interp.x(), pos_interp.y(), pos_interp.z(),
		     tf::getYaw(q_interp),
		     vel_interp.x(), vel_interp.y(), vel_interp.z(),
		     time_interp);
  
  return wp_interp;
}

static int trajectory_class_counter = 0;

Trajectory::Trajectory(){
  //listener = NULL;
  generated_waypoint_times = false;
  
  // init marker namespace to be unique
  std::stringstream ss;
  ss << "trajectory_" << trajectory_class_counter;
  marker_namespace = ss.str();
  trajectory_class_counter++;
}

Trajectory::Trajectory(std::string frame_id){
  this->frame_id = frame_id;
  //listener = NULL;
  generated_waypoint_times = false;
  
  // init marker namespace to be unique
  std::stringstream ss;
  ss << "trajectory_" << trajectory_class_counter;
  marker_namespace = ss.str();
  trajectory_class_counter++;
}

Trajectory::Trajectory(core_trajectory_msgs::TrajectoryXYZVYaw path){
  frame_id = path.header.frame_id;
  stamp = path.header.stamp;
  //listener = NULL;
  generated_waypoint_times = false;

  // init marker namespace to be unique
  std::stringstream ss;
  ss << "trajectory_" << trajectory_class_counter;
  marker_namespace = ss.str();
  trajectory_class_counter++;

  // remove any consecutive duplicate waypoints
  core_trajectory_msgs::TrajectoryXYZVYaw cleaned_path;
  cleaned_path.header = path.header;
  for(int i = 0; i < path.waypoints.size(); i++){
    core_trajectory_msgs::WaypointXYZVYaw wp = path.waypoints[i];
    if(i == 0)
      cleaned_path.waypoints.push_back(wp);
    else if(!(wp.position.x == cleaned_path.waypoints.back().position.x &&
	      wp.position.y == cleaned_path.waypoints.back().position.y &&
	      wp.position.z == cleaned_path.waypoints.back().position.z))
      cleaned_path.waypoints.push_back(wp);
  }
  
  //ROS_INFO_STREAM("TRAJECTORY CONSTRUCTOR");
  for(int i = 0; i < cleaned_path.waypoints.size(); i++){
    core_trajectory_msgs::WaypointXYZVYaw wp = cleaned_path.waypoints[i];

    // calculate the x, y, z components of velocity
    tf::Vector3 wp1(wp.position.x, wp.position.y, wp.position.z);
    tf::Vector3 wp2(wp1);
    if(i == 0){ // for the first waypoint use the next waypoint to figure out direction of travel
      if(cleaned_path.waypoints.size() > 1)
	wp2 = tf::Vector3(cleaned_path.waypoints[i+1].position.x, cleaned_path.waypoints[i+1].position.y, cleaned_path.waypoints[i+1].position.z);
    }
    else{ // otherwise use the previous waypoint
      wp1 = tf::Vector3(cleaned_path.waypoints[i-1].position.x, cleaned_path.waypoints[i-1].position.y, cleaned_path.waypoints[i-1].position.z);
    }
    
    tf::Vector3 direction = (wp2-wp1).normalized();
    tf::Vector3 vel = wp.velocity*direction;
    
    //ROS_INFO_STREAM("WAYPOINT " << i << " vel: " << vel.x() << " " << vel.y() << " " << vel.z());
    //ROS_INFO_STREAM("\twp1: " << wp1.x() << " " << wp1.y() << " " << wp1.z());
    //ROS_INFO_STREAM("\twp2: " << wp2.x() << " " << wp2.y() << " " << wp2.z());
    //ROS_INFO_STREAM("\tdirection: " << direction.x() << " " << direction.y() << " " << direction.z() << " wp.vel: " << wp.vel);
    
    Waypoint waypoint(wp.position.x, wp.position.y, wp.position.z, wp.yaw, vel.x(), vel.y(), vel.z());
    waypoints.push_back(waypoint);
  }
}

void Trajectory::init_listener(){
  if(listener == NULL)
    listener = new tf::TransformListener();
}

void Trajectory::clear(){
  waypoints.clear();
  generated_waypoint_times = false;
}

void Trajectory::generate_waypoint_times(){
  if(generated_waypoint_times)
    return;

  /*
  if(waypoints.size() > 1){
    ROS_INFO_STREAM("WAYPOINT " << 0 << waypoints[0]);
  }
  */
  
  for(int i = 1; i < waypoints.size(); i++){
    Waypoint& curr_wp = waypoints[i];
    Waypoint& prev_wp = waypoints[i-1];

    double distance = curr_wp.position().distance(prev_wp.position());
    double velocity = std::max(0.01, curr_wp.velocity().length());
    //if(i+1 < waypoints.size()){
    //  velocity = (velocity + waypoints[i+1].velocity().length())/2.0;
    //}

    //ROS_INFO_STREAM("WAYPOINT " << i << " dist: " << distance << " vel: " << velocity << " time: " << (prev_wp.time() + distance/velocity) << " prev_time: " << prev_wp.time() << " inc: " << (distance/velocity));

    curr_wp.set_time(prev_wp.time() + distance/velocity);
  }
  
  generated_waypoint_times = true;
}

bool Trajectory::get_closest_point(tf::Vector3 point, tf::Vector3* closest, int* wp_index, double* path_distance){
  double closest_distance = std::numeric_limits<double>::max();
  double best_t = 0;
  int best_wp_index = 0;
  
  if(waypoints.size() == 0){
    if(wp_index != NULL)
      *wp_index = 0;
    if(path_distance != NULL)
      path_distance = 0;
    return false;
  }
  else if(waypoints.size() == 1){
    *closest = waypoints[0].position();
    best_wp_index = 0;
    if(path_distance != NULL)
      path_distance = 0;
  }
  else{
    for(int i = 1; i < waypoints.size(); i++){
      // parameteric representation of segment between waypoints: segment_start + t*segment_vec
      tf::Vector3 segment_start = waypoints[i-1].position();
      tf::Vector3 segment_end = waypoints[i].position();
      tf::Vector3 segment_vec = segment_end - segment_start;

      // project the vector (point - segment_start) onto the parametric line
      double t = ((point - segment_start).dot(segment_vec)) / (segment_vec.dot(segment_vec));
      t = std::max(0.0, std::min(1.0, t));
      tf::Vector3 closest_point = segment_start + t*segment_vec;

      // find the distance between the point and the closest point on the current segment
      double distance = closest_point.distance(point);
      if(distance < closest_distance){
	closest_distance = distance;
	*closest = closest_point;
	best_wp_index = i-1;
	best_t = t;
      }
    }

    if(path_distance != NULL){
      *path_distance = 0;
      for(int i = 0; i <= best_wp_index; i++){
	tf::Vector3 segment_start = waypoints[i].position();
	tf::Vector3 segment_end = waypoints[i+1].position();
	tf::Vector3 segment_vec = segment_end - segment_start;
	
	if(i == best_wp_index)
	  *path_distance += best_t*segment_vec.length();
	else
	  *path_distance += segment_vec.length();
      }
    }
  }

  
  if(wp_index != NULL)
    *wp_index = best_wp_index;
  
  return true;
}

bool Trajectory::get_trajectory_distance_at_closest_point(tf::Vector3 point,
							  double* trajectory_distance){
  double closest_distance = std::numeric_limits<double>::max();
  *trajectory_distance = 0;
  double current_trajectory_distance = 0;
  
  if(waypoints.size() == 0)
    return false;
  else if(waypoints.size() == 1)
    return true;
  else{
    for(int i = 1; i < waypoints.size(); i++){
      // parameteric representation of segment between waypoints: segment_start + t*segment_vec
      tf::Vector3 segment_start = waypoints[i-1].position();
      tf::Vector3 segment_end = waypoints[i].position();
      tf::Vector3 segment_vec = segment_end - segment_start;
      double segment_length = segment_start.distance(segment_end);

      // project the vector (point - segment_start) onto the parametric line
      double t = ((point - segment_start).dot(segment_vec)) / (segment_vec.dot(segment_vec));
      t = std::max(0.0, std::min(1.0, t));
      tf::Vector3 closest_point = segment_start + t*segment_vec;

      // find the distance between the point and the closest point on the current segment
      double distance = closest_point.distance(point);
      if(distance < closest_distance){
	closest_distance = distance;
	*trajectory_distance = current_trajectory_distance + t*segment_length;
      }
      
      current_trajectory_distance += segment_length;
    }
  }
  
  return true;
}

bool Trajectory::merge(Trajectory traj){
  if(traj.waypoints.size() == 0){
    return true;
  }
  generated_waypoint_times = false;
  
  Trajectory transformed_traj;
  try{
    transformed_traj = traj.to_frame(frame_id, stamp);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform exception while merging trajectories: " << ex.what());
  }
  if(transformed_traj.waypoints.size() == 0){
    return true;
  }
  
  if(waypoints.size() == 0){
    waypoints.insert(waypoints.end(), transformed_traj.waypoints.begin(), transformed_traj.waypoints.end());
    return true;
  }
  
  tf::Vector3 closest_point;
  int wp_index;
  bool valid = get_closest_point(transformed_traj.waypoints[0].position(), &closest_point, &wp_index);
  
  waypoints.erase(waypoints.begin() + wp_index+1, waypoints.end());
  waypoints.insert(waypoints.end(), transformed_traj.waypoints.begin(), transformed_traj.waypoints.end());
  return true;
}

double Trajectory::get_duration(){
  generate_waypoint_times();
  
  if(waypoints.size() == 0)
    return 0;
  
  return waypoints.back().time();
}

bool Trajectory::get_odom(double time, nav_msgs::Odometry* odom){
  generate_waypoint_times();
  
  if(waypoints.size() == 0){
    return false;
  }
  else if(waypoints.size() == 1){
    *odom = waypoints[0].odometry(ros::Time::now(), frame_id);
    return true;
  }

  // figure out what waypoints we are between
  Waypoint prev_wp = waypoints.back();
  Waypoint curr_wp = waypoints.back();
  Waypoint next_wp = waypoints.back();
  for(int i = 1; i < waypoints.size(); i++){
    if(time <= waypoints[i].time()){
      prev_wp = waypoints[i-1];
      curr_wp = waypoints[i];
      next_wp = curr_wp;
      if(i+1 < waypoints.size())
	next_wp = waypoints[i+1];
      break;
    }
  }

  // figure out where we are in between the current and previous waypoints
  double t = (time - prev_wp.time()) / (curr_wp.time() - prev_wp.time());
  t = std::max(0.0, std::min(1.0, t));

  // set frame and timestamp
  odom->header.stamp = ros::Time::now(); // TODO: figure out if this is correct
  odom->header.frame_id = frame_id;
  odom->child_frame_id = frame_id;

  // interpolate position
  tf::Vector3 position = prev_wp.position() + t*(curr_wp.position() - prev_wp.position());
  odom->pose.pose.position.x = position.x();
  odom->pose.pose.position.y = position.y();
  odom->pose.pose.position.z = position.z();

  // interpolate velocity
  tf::Vector3 velocity = curr_wp.velocity() + t*(next_wp.velocity() - curr_wp.velocity());
  odom->twist.twist.linear.x = velocity.x();
  odom->twist.twist.linear.y = velocity.y();
  odom->twist.twist.linear.z = velocity.z();

  // interpolate orientation
  tf::Quaternion q = prev_wp.q().slerp(curr_wp.q(), t);
  odom->pose.pose.orientation.x = q.x();
  odom->pose.pose.orientation.y = q.y();
  odom->pose.pose.orientation.z = q.z();
  odom->pose.pose.orientation.w = q.w();
  
  return true;
}

Trajectory Trajectory::to_frame(std::string target_frame, ros::Time time) {
  init_listener();
  
  tf::StampedTransform transform;
  listener->waitForTransform(target_frame, frame_id, time, ros::Duration(0.1));
  listener->lookupTransform(target_frame, frame_id, time, transform);
  
  Trajectory transformed_traj;
  transformed_traj.stamp = time;
  transformed_traj.frame_id = target_frame;
  
  for(int i = 0; i < waypoints.size(); i++){
    Waypoint wp = waypoints[i];
    
    tf::Vector3 transformed_position = transform*wp.position();
    tf::Vector3 transformed_velocity = tf::Transform(transform.getRotation())*wp.velocity();
    tf::Quaternion transformed_q = transform*wp.q();
    
    Waypoint transformed_wp(transformed_position.x(), transformed_position.y(), transformed_position.z(), tf::getYaw(transformed_q),
			    transformed_velocity.x(), transformed_velocity.y(), transformed_velocity.z());

    transformed_traj.waypoints.push_back(transformed_wp);
  }
  
  return transformed_traj;
}

Trajectory Trajectory::respace(double spacing){
  Trajectory traj;
  traj.stamp = stamp;
  traj.frame_id = frame_id;
  //ROS_INFO("RESPACE");
  
  if(waypoints.size() > 0){
    double distance = 0;
    traj.waypoints.push_back(waypoints[0]); // always add the first waypoint
    
    for(int i = 1; i < waypoints.size(); i++){
      if(i == waypoints.size()-1){ // always add the last waypoint
	traj.waypoints.push_back(waypoints[i]);
	continue;
      }
      //ROS_INFO_STREAM("Waypoint " << i);
      Waypoint prev_wp = waypoints[i-1];
      Waypoint curr_wp = waypoints[i];
      double segment_length = prev_wp.position().distance(curr_wp.position());
      //ROS_INFO_STREAM("segment_length: " << segment_length << " distance: " << distance << " spacing: " << spacing);
      
      if(distance + segment_length >= spacing){
	//ROS_INFO("interpolating");
	Waypoint interp_wp = prev_wp.interpolate(curr_wp, (spacing - distance)/segment_length);
	traj.waypoints.push_back(interp_wp);
	distance = 0;
      }
      else
	distance += segment_length;
    }
  }

  return traj;
}

Trajectory Trajectory::shorten(double new_length){
  Trajectory traj;
  traj.stamp = stamp;
  traj.frame_id = frame_id;
  
  if(waypoints.size() > 0){
    double distance = 0;
    traj.waypoints.push_back(waypoints[0]); // always add the first waypoint
    
    for(int i = 1; i < waypoints.size(); i++){
      Waypoint prev_wp = waypoints[i-1];
      Waypoint curr_wp = waypoints[i];
      double segment_length = prev_wp.position().distance(curr_wp.position());

      if(distance + segment_length >= new_length){
	Waypoint interp_wp = prev_wp.interpolate(curr_wp, (new_length - distance)/segment_length);
	traj.waypoints.push_back(interp_wp);
	break;
      }
      else{
	traj.waypoints.push_back(curr_wp);
	distance += segment_length;
      }
    }
  }

  return traj;
}

Trajectory Trajectory::get_subtrajectory_distance(double start, double end){
  Trajectory traj;
  traj.stamp = stamp;
  traj.frame_id = frame_id;
  
  if(waypoints.size() > 0){
    double distance = 0;
    
    for(int i = 1; i < waypoints.size(); i++){
      Waypoint prev_wp = waypoints[i-1];
      Waypoint curr_wp = waypoints[i];
      double segment_length = prev_wp.position().distance(curr_wp.position());

      if(start >= distance && start <= distance+segment_length){
	Waypoint interp_start_wp = prev_wp.interpolate(curr_wp, (start-distance)/segment_length);
	traj.waypoints.push_back(interp_start_wp);
      }

      if(distance+segment_length > start && distance+segment_length < end){
	traj.waypoints.push_back(curr_wp);
      }
      
      if(end >= distance && end <= distance+segment_length){
	Waypoint interp_end_wp = prev_wp.interpolate(curr_wp, (end-distance)/segment_length);
	traj.waypoints.push_back(interp_end_wp);
      }

      distance += segment_length;
    }
  }

  return traj;
}

void Trajectory::set_fixed_height(double height){
  for(int i = 0; i < waypoints.size(); i++)
    waypoints[i].z_ = height;
}

int Trajectory::waypoint_count(){
  return waypoints.size();
}

Waypoint Trajectory::get_waypoint(int index){
  return waypoints[index];
}

std::string Trajectory::get_frame_id(){
  return frame_id;
}

core_trajectory_msgs::TrajectoryXYZVYaw Trajectory::get_TrajectoryXYZVYaw(){
  core_trajectory_msgs::TrajectoryXYZVYaw path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id;

  for(int i = 0; i < waypoints.size(); i++){
    Waypoint wp = waypoints[i];

    core_trajectory_msgs::WaypointXYZVYaw w;
    w.position.x = wp.x();
    w.position.y = wp.y();
    w.position.z = wp.z();
    w.yaw = tf::getYaw(wp.q());
    w.velocity = wp.velocity().length();
    
    path.waypoints.push_back(w);
  }

  return path;
}

std::vector<geometry_msgs::PointStamped> Trajectory::get_vector_PointStamped(){
  std::vector<geometry_msgs::PointStamped> points;

  for(int i = 0; i < waypoints.size(); i++){
    Waypoint wp = waypoints[i];
    
    geometry_msgs::PointStamped point;
    point.header.stamp = stamp;
    point.header.frame_id = frame_id;
    point.point.x = wp.x();
    point.point.y = wp.y();
    point.point.z = wp.z();
    
    points.push_back(point);
  }

  return points;
}

visualization_msgs::MarkerArray Trajectory::get_markers(float r, float g, float b, float a, bool show_poses, bool show_velocity){
  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();

  visualization_msgs::Marker clear;
  clear.ns = marker_namespace;
  clear.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  visualization_msgs::Marker lines;
  lines.header.stamp = now;
  lines.header.frame_id = frame_id;
  lines.ns = marker_namespace;
  lines.id = waypoints.size();
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.1;
  
  for(int i = 0; i < waypoints.size(); i++){
    Waypoint wp = waypoints[i];

    if(show_poses){
      visualization_msgs::Marker arrow;
      arrow.header.stamp = now;
      arrow.header.frame_id = frame_id;
      arrow.ns = marker_namespace;
      arrow.id = i;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
    
      arrow.pose.position.x = wp.position().x();
      arrow.pose.position.y = wp.position().y();
      arrow.pose.position.z = wp.position().z();
      arrow.pose.orientation.x = wp.q().x();
      arrow.pose.orientation.y = wp.q().y();
      arrow.pose.orientation.z = wp.q().z();
      arrow.pose.orientation.w = wp.q().w();
      arrow.scale.x = 0.5; // length
      arrow.scale.y = 0.1; // width
      arrow.scale.z = 0.1; // height
      arrow.color.r = r;
      arrow.color.g = g;
      arrow.color.b = b;
      arrow.color.a = a;
    
      marker_array.markers.push_back(arrow);
    }

    if(show_velocity){
      visualization_msgs::Marker vel_arrow;
      vel_arrow.header.stamp = now;
      vel_arrow.header.frame_id = frame_id;
      vel_arrow.ns = marker_namespace + "_velocities";
      vel_arrow.id = i;
      vel_arrow.type = visualization_msgs::Marker::ARROW;
      vel_arrow.action = visualization_msgs::Marker::ADD;
    
      geometry_msgs::Point point1, point2;
      point1.x = wp.position().x();
      point1.y = wp.position().y();
      point1.z = wp.position().z()+0.2;
      point2.x = wp.position().x() + wp.velocity().normalized().x()/3.;
      point2.y = wp.position().y() + wp.velocity().normalized().y()/3.;
      point2.z = wp.position().z()+0.2 + wp.velocity().normalized().z()/3.;
      vel_arrow.points.push_back(point1);
      vel_arrow.points.push_back(point2);
      vel_arrow.scale.x = 0.1;
      vel_arrow.scale.y = 0.15;
      vel_arrow.scale.z = 0.1;
      vel_arrow.color.r = r;
      vel_arrow.color.g = g;
      vel_arrow.color.b = b;
      vel_arrow.color.a = a;
    
      marker_array.markers.push_back(vel_arrow);
    }
    
    geometry_msgs::Point p;
    p.x = wp.position().x();
    p.y = wp.position().y();
    p.z = wp.position().z();
    lines.points.push_back(p);
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    lines.colors.push_back(color);
  }

  marker_array.markers.push_back(lines);
  
  return marker_array;
}

//===================================================================================
//--------------------------------- Dynamic Trajectories ----------------------------
//===================================================================================

TakeoffTrajectory::TakeoffTrajectory(double height, double velocity, double path_roll, double path_pitch, bool relative_to_orientation){
  this->height = height;
  this->velocity = velocity;
  this->path_roll = path_roll;
  this->path_pitch = path_pitch;
  this->relative_to_orientation = relative_to_orientation;
}

core_trajectory_msgs::TrajectoryXYZVYaw TakeoffTrajectory::get_trajectory(nav_msgs::Odometry odom){
  core_trajectory_msgs::TrajectoryXYZVYaw traj;
  traj.header.frame_id = odom.header.frame_id;
  traj.header.stamp = odom.header.stamp;

  core_trajectory_msgs::WaypointXYZVYaw wp1, wp2;
  
  wp1.position.x = odom.pose.pose.position.x;
  wp1.position.y = odom.pose.pose.position.y;
  wp1.position.z = odom.pose.pose.position.z;

  tf::Quaternion q(odom.pose.pose.orientation.x,
		   odom.pose.pose.orientation.y,
		   odom.pose.pose.orientation.z,
		   odom.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  wp1.yaw = yaw;//tf::getYaw(q);
  wp1.velocity = velocity;


  
  wp2.position.x = odom.pose.pose.position.x + height*sin(path_pitch + (relative_to_orientation ? pitch : 0));
  wp2.position.y = odom.pose.pose.position.y + height*sin(path_roll - (relative_to_orientation ? roll : 0));
  wp2.position.z = odom.pose.pose.position.z + height;
  wp2.yaw = wp1.yaw;
  wp2.velocity = velocity;

  traj.waypoints.push_back(wp1);
  traj.waypoints.push_back(wp2);
  
  return traj;
}


AccelerationTrajectory::AccelerationTrajectory(tf::TransformListener* listener, std::string frame, double ax, double ay, double az, double dt, double ht, double max_velocity){
  this->listener = listener;

  ros::NodeHandle nh("acceleration_trajectory");
  set_max_velocity_sub = nh.subscribe("set_max_velocity", 1, &AccelerationTrajectory::set_max_velocity_callback, this);
  
  this->ax = ax;
  this->ay = ay;
  this->az = az;
  this->dt = dt;
  this->ht = ht;
  this->max_velocity = max_velocity;
  this->frame = frame;
}

void AccelerationTrajectory::set_max_velocity_callback(std_msgs::Float32 msg){
  max_velocity = msg.data;
}

core_trajectory_msgs::TrajectoryXYZVYaw AccelerationTrajectory::get_trajectory(nav_msgs::Odometry odom){
  core_trajectory_msgs::TrajectoryXYZVYaw traj;
  traj.header.frame_id = frame;
  traj.header.stamp = odom.header.stamp;
  
  try{
    tf::StampedTransform pose_tf;
    listener->waitForTransform(frame, odom.header.frame_id, odom.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(frame, odom.header.frame_id, odom.header.stamp, pose_tf);
    tf::StampedTransform velocity_tf;
    listener->waitForTransform(frame, odom.child_frame_id, odom.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(frame, odom.child_frame_id, odom.header.stamp, velocity_tf);
    velocity_tf.setOrigin(tf::Vector3(0, 0, 0)); // only use rotation to transform the velocity
    
    tf::Vector3 position = pose_tf*tflib::to_tf(odom.pose.pose.position);
    tf::Quaternion orientation = pose_tf*tflib::to_tf(odom.pose.pose.orientation);
    tf::Vector3 velocity = velocity_tf*tflib::to_tf(odom.twist.twist.linear);

    tf::Vector3 acceleration(ax, ay, az);

    core_trajectory_msgs::WaypointXYZVYaw wp0;
    wp0.position.x = position.x();
    wp0.position.y = position.y();
    wp0.position.z = position.z();
    wp0.yaw = 0;
    wp0.velocity = velocity.length();
    traj.waypoints.push_back(wp0);
    
    for(double t = 0; t < ht; t += dt){
      velocity += acceleration*dt;
      position += velocity*dt;
      
      core_trajectory_msgs::WaypointXYZVYaw wp;
      wp.position.x = position.x();
      wp.position.y = position.y();
      wp.position.z = position.z();
      wp.yaw = 0;
      wp.velocity = std::min(velocity.length(), max_velocity);
      traj.waypoints.push_back(wp);
    }
  }
  catch(tf::TransformException& e){
    ROS_ERROR_STREAM("TransformException in AccelerationTrajectory::get_trajectory: " << e.what());
  }

  return traj;
}

//===================================================================================
//--------------------------------- Static Trajectories -----------------------------
//===================================================================================

CurveTrajectory::CurveTrajectory(float linear_velocity, float angular_velocity, std::string frame,
				 float time, float dt, bool use_heading, float yaw){
  this->linear_velocity = linear_velocity;
  this->angular_velocity = angular_velocity;
  this->frame = frame;
  this->time = time;
  this->dt = dt;
  this->use_heading = use_heading;
  this->yaw = yaw;

  generate_trajectory();
}


void CurveTrajectory::generate_trajectory(){
  float prev_x = 0.f;
  float prev_y = 0.f;
  float prev_heading = 0.f;

  //TODO: when to fill in trajectory.header.stamp?
  trajectory.header.frame_id = frame;
  
  for(float t = 0; t <= time; t += dt){
    core_trajectory_msgs::WaypointXYZVYaw wp;
    
    if(t > 0){
      wp.position.x = prev_x + cos(prev_heading)*linear_velocity*dt;
      wp.position.y = prev_y + sin(prev_heading)*linear_velocity*dt;
      wp.yaw = prev_heading + angular_velocity*dt;
      prev_x = wp.position.x;
      prev_y = wp.position.y;
      prev_heading = wp.yaw;
      
    }
    
    wp.velocity = linear_velocity;
    if(!use_heading)
      wp.yaw = yaw;
    
    trajectory.waypoints.push_back(wp);
  }
}

core_trajectory_msgs::TrajectoryXYZVYaw CurveTrajectory::get_trajectory(){
  return trajectory;
}

//===================================================================================
//--------------------------------- Trajectory Library ------------------------------
//===================================================================================

TrajectoryLibrary::TrajectoryLibrary(std::string config_filename, tf::TransformListener* listener){
  this->listener = listener;
  pnh = new ros::NodeHandle("~");
  
  YAML::Node config = YAML::LoadFile(config_filename);

  YAML::Node trajectories = config["trajectories"];
  for(int i = 0; i < trajectories.size(); i++){
    YAML::Node traj_node = trajectories[i];
    std::string type = parse<std::string>(traj_node["type"]);

    if(type == "curve"){
      float linear_velocity = parse<double>(traj_node["linear_velocity"]);
      float angular_velocity = M_PI/180. * parse<double>(traj_node["angular_velocity"]);
      std::string frame = parse<std::string>(traj_node["frame"]);
      float time = parse<double>(traj_node["time"]);
      float dt = parse<double>(traj_node["dt"]);
      bool use_heading = true;
      float yaw = 0.f;
      if(parse<std::string>(traj_node["yaw"]) == std::string("heading"))
	use_heading = true;
      else{
	use_heading = false;
	yaw = M_PI/180. * parse<double>(traj_node["yaw"]);
      }
      
      CurveTrajectory* traj = new CurveTrajectory(linear_velocity, angular_velocity, frame, time, dt, use_heading, yaw);
      
      static_trajectories.push_back(traj);
    }
    else if(type == "acceleration"){
      std::string frame = parse<std::string>(traj_node["frame"]);
      double x;
      double y;
      double z;
      
      if(traj_node["x"] && traj_node["y"] && traj_node["z"]){
	x = parse<double>(traj_node["x"]);
	y = parse<double>(traj_node["y"]);
	z = parse<double>(traj_node["z"]);
      }
      else if(traj_node["magnitude"]){
	double magnitude = parse<double>(traj_node["magnitude"]);
	double magnitude_yaw = M_PI/180. * parse<double>(traj_node["magnitude_yaw"]);
	double magnitude_pitch = M_PI/180. * parse<double>(traj_node["magnitude_pitch"]);
	
	tf::Quaternion q;
	q.setRPY(0, magnitude_pitch, magnitude_yaw);
	tf::Vector3 vector = tf::Transform(q)*tf::Vector3(magnitude, 0, 0);
	x = vector.x();
	y = vector.y();
	z = vector.z();
      }
      else{
	std::cout << "Couldn't construct acceleration trajectory. Need to have either x, y, z or magnitude, magnitude_yaw, magnitude_pitch." << std::endl;
	continue;
      }
      double dt = parse<double>(traj_node["dt"]);
      double ht = parse<double>(traj_node["ht"]);
      double max_velocity = parse<double>(traj_node["max_velocity"]);

      AccelerationTrajectory* traj = new AccelerationTrajectory(listener, frame, x, y, z, dt, ht, max_velocity);

      dynamic_trajectories.push_back(traj);
    }
  }
}

std::vector<Trajectory> TrajectoryLibrary::get_static_trajectories(){
  std::vector<Trajectory> trajectories;
  
  for(int i = 0; i < static_trajectories.size(); i++){
    trajectories.push_back(Trajectory(static_trajectories[i]->get_trajectory()));
  }
  
  return trajectories;
}

std::vector<Trajectory> TrajectoryLibrary::get_dynamic_trajectories(nav_msgs::Odometry odom){
  std::vector<Trajectory> trajectories;
  
  for(int i = 0; i < dynamic_trajectories.size(); i++){
    core_trajectory_msgs::TrajectoryXYZVYaw path = dynamic_trajectories[i]->get_trajectory(odom);
    if(path.waypoints.size() > 0)
      trajectories.push_back(Trajectory(path));
  }
  
  return trajectories;
}

