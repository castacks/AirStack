#include <core_disparity_map_representation/disparity_map_representation.h>
#include <pluginlib/class_list_macros.h>

DisparityMapRepresentation::DisparityMapRepresentation(){
  ros::NodeHandle* nh = new ros::NodeHandle();
  ros::NodeHandle* pnh = new ros::NodeHandle("~");
  disp_graph = new nabla::disparity_graph::disparity_graph();

  points.ns = "obstacles";
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.action = visualization_msgs::Marker::ADD;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.scale.z = 0.1;

  debug_pub = nh->advertise<visualization_msgs::MarkerArray>("disparity_map_debug", 1);

  obstacle_check_points = pnh->param("disparity_map/obstacle_check_points", 5);
  obstacle_check_radius = pnh->param("disparity_map/obstacle_check_radius", 2.);
  
  listener = new tf::TransformListener();
}


std::vector< std::vector<double> > DisparityMapRepresentation::get_values(std::vector<std::vector<geometry_msgs::PointStamped> > trajectories){
  std::vector< std::vector<double> > values(trajectories.size());
  
  for(int i = 0; i < trajectories.size(); i++)
    for(int j = 0; j < trajectories[i].size(); j++)
      values[i].push_back(0);
  
  for(int i = 0; i < trajectories.size(); i++){
    //core_trajectory_msgs::TrajectoryXYZVYaw trajectory = trajectories[i];
    for(int j = 0; j < trajectories[i].size(); j++){
      tf::Vector3 wp = tflib::to_tf(trajectories[i][j].point);
      
      // find the direction of the current trajectory segment between waypoints
      // this direction will be used to check for obstacles in a perpendicular direction
      tf::Vector3 direction;
      tf::Vector3 wp2 = wp;
      if(trajectories[i].size() < 2){
	direction = tf::Vector3(1, 0, 0); // TODO: make this point in the direction of the waypoints quaternion
      }
      else{
	if(j == 0){
	  wp2 = tflib::to_tf(trajectories[i][1].point);
	  direction = wp2 - wp;
	}
	else{
	  wp2 = tflib::to_tf(trajectories[i][j-1].point);
	  direction = wp - wp2;
	}
      }
      
      points.header.frame_id = trajectories[i][j].header.frame_id;
      std_msgs::ColorRGBA green;
      green.r = 0;
      green.b = 0;
      green.g = 1;
      green.a = 1;
      std_msgs::ColorRGBA red;
      red.r = 1;
      red.b = 0;
      red.g = 0;
      red.a = 1;

      tf::Quaternion q_up, q_down, q_left, q_right;
      q_up.setRPY(0, -M_PI/2, 0);
      q_down.setRPY(0, M_PI/2, 0);
      q_left.setRPY(0, 0, M_PI/2);
      q_right.setRPY(0, 0, -M_PI/2);
      std::vector<tf::Quaternion> directions;
      directions.push_back(q_up);
      directions.push_back(q_down);
      directions.push_back(q_left);
      directions.push_back(q_right);

      tf::Vector3 position = wp;//tflib::to_tf(pose.pose.position);
      tf::Quaternion q = tf::Quaternion(0, 0, 0, 1); // TODO: figure out if this makes sense //tflib::to_tf(pose.pose.orientation);
      tf::Vector3 unit(1, 0, 0);
      double closest_obstacle_distance = obstacle_check_radius;
  
      direction.normalize();
      tf::Vector3 up = tf::Transform(q_up*q)*unit;
      up.normalize();

      tf::Vector3 side = up.cross(direction);
      side.normalize();

      up = side.cross(direction);
      up.normalize();

      std::vector<tf::Vector3> direction_vectors;
      direction_vectors.push_back(up);
      direction_vectors.push_back(side);
      direction_vectors.push_back(-up);
      direction_vectors.push_back(-side);

      for(int m = 0; m < directions.size(); m++){
	tf::Quaternion q_curr = directions[m];
	for(int k = 1; k < obstacle_check_points+1; k++){
	  double dist = (double)k*obstacle_check_radius/(double)(obstacle_check_points+1);
      
	  tf::Vector3 point = position + dist*direction_vectors[m];//tf::Transform(q_curr)*(dist*direction);
	  geometry_msgs::PoseStamped check_pose;
	  check_pose.header = trajectories[i][j].header;
	  check_pose.pose.position = trajectories[i][j].point;
	  check_pose.pose.orientation.w = 1.0;

	  double occupancy;
	  bool collision = !disp_graph->isStateValidDepth_pose(check_pose, 0.9, occupancy);
      
	  std_msgs::ColorRGBA c;
	  if(collision){
	    closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
	    c = red;
	  }
	  else
	    c = green;
      
	  points.points.push_back(check_pose.pose.position);
	  points.colors.push_back(c);
	}
      }
  
      points.points.push_back(trajectories[i][j].point);

      geometry_msgs::PoseStamped pose;
      pose.header = trajectories[i][j].header;//trajectory.header;
      pose.pose.position = trajectories[i][j].point;
      pose.pose.orientation.w = 1.0;      
  
      double occupancy;
      if(!disp_graph->isStateValidDepth_pose(pose, 0.9, occupancy)){
	values[i][j] = 0.f;
	points.colors.push_back(red);
      }
      else{
	points.colors.push_back(green);
      }

      values[i][j] = closest_obstacle_distance;
    }
  }
  
  
  return values;
}

/*std::vector< std::vector<double> > DisparityMapRepresentation::get_values(std::vector<core_trajectory_msgs::TrajectoryXYZVYaw> trajectories){
  std::vector< std::vector<double> > values(trajectories.size());
  
  for(int i = 0; i < trajectories.size(); i++)
    for(int j = 0; j < trajectories[i].waypoints.size(); j++)
      values[i].push_back(0);
  
  for(int i = 0; i < trajectories.size(); i++){
    core_trajectory_msgs::TrajectoryXYZVYaw trajectory = trajectories[i];
    for(int j = 0; j < trajectory.waypoints.size(); j++){
      tf::Vector3 wp = tflib::to_tf(trajectory.waypoints[j].position);
      
      // find the direction of the current trajectory segment between waypoints
      // this direction will be used to check for obstacles in a perpendicular direction
      tf::Vector3 direction;
      tf::Vector3 wp2 = wp;
      if(trajectory.waypoints.size() < 2){
	direction = tf::Vector3(1, 0, 0); // TODO: make this point in the direction of the waypoints quaternion
      }
      else{
	if(j == 0){
	  wp2 = tflib::to_tf(trajectory.waypoints[1].position);
	  direction = wp2 - wp;
	}
	else{
	  wp2 = tflib::to_tf(trajectory.waypoints[j-1].position);
	  direction = wp - wp2;
	}
      }
      
      points.header.frame_id = trajectory.header.frame_id;
      std_msgs::ColorRGBA green;
      green.r = 0;
      green.b = 0;
      green.g = 1;
      green.a = 1;
      std_msgs::ColorRGBA red;
      red.r = 1;
      red.b = 0;
      red.g = 0;
      red.a = 1;

      tf::Quaternion q_up, q_down, q_left, q_right;
      q_up.setRPY(0, -M_PI/2, 0);
      q_down.setRPY(0, M_PI/2, 0);
      q_left.setRPY(0, 0, M_PI/2);
      q_right.setRPY(0, 0, -M_PI/2);
      std::vector<tf::Quaternion> directions;
      directions.push_back(q_up);
      directions.push_back(q_down);
      directions.push_back(q_left);
      directions.push_back(q_right);

      tf::Vector3 position = wp;//tflib::to_tf(pose.pose.position);
      tf::Quaternion q = tf::Quaternion(0, 0, 0, 1); // TODO: figure out if this makes sense //tflib::to_tf(pose.pose.orientation);
      tf::Vector3 unit(1, 0, 0);
      double closest_obstacle_distance = obstacle_check_radius;
  
      direction.normalize();
      tf::Vector3 up = tf::Transform(q_up*q)*unit;
      up.normalize();

      tf::Vector3 side = up.cross(direction);
      side.normalize();

      up = side.cross(direction);
      up.normalize();

      std::vector<tf::Vector3> direction_vectors;
      direction_vectors.push_back(up);
      direction_vectors.push_back(side);
      direction_vectors.push_back(-up);
      direction_vectors.push_back(-side);

      for(int m = 0; m < directions.size(); m++){
	tf::Quaternion q_curr = directions[m];
	for(int k = 1; k < obstacle_check_points+1; k++){
	  double dist = (double)k*obstacle_check_radius/(double)(obstacle_check_points+1);
      
	  tf::Vector3 point = position + dist*direction_vectors[m];//tf::Transform(q_curr)*(dist*direction);
	  geometry_msgs::PoseStamped check_pose;
	  check_pose.header = trajectory.header;
	  check_pose.pose.position.x = point.x();
	  check_pose.pose.position.y = point.y();
	  check_pose.pose.position.z = point.z();
	  check_pose.pose.orientation.w = 1.0;

	  double occupancy;
	  bool collision = !disp_graph->isStateValidDepth_pose(check_pose, 0.9, occupancy);
      
	  std_msgs::ColorRGBA c;
	  if(collision){
	    closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
	    c = red;
	  }
	  else
	    c = green;
      
	  points.points.push_back(check_pose.pose.position);
	  points.colors.push_back(c);
	}
      }
  
      points.points.push_back(trajectory.waypoints[j].position);

      geometry_msgs::PoseStamped pose;
      pose.header = trajectory.header;
      pose.pose.position.x = wp.x();
      pose.pose.position.y = wp.y();
      pose.pose.position.z = wp.z();
      pose.pose.orientation.w = 1.0;      
  
      double occupancy;
      if(!disp_graph->isStateValidDepth_pose(pose, 0.9, occupancy)){
	values[i][j] = 0.f;
	points.colors.push_back(red);
      }
      else{
	points.colors.push_back(green);
      }

      values[i][j] = closest_obstacle_distance;
    }
  }
  
  
  return values;
  }*/

double DisparityMapRepresentation::distance_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction){
  points.header.frame_id = pose.header.frame_id;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.b = 0;
  green.g = 1;
  green.a = 1;
  std_msgs::ColorRGBA red;
  red.r = 1;
  red.b = 0;
  red.g = 0;
  red.a = 1;


  tf::Quaternion q_up, q_down, q_left, q_right;
  q_up.setRPY(0, -M_PI/2, 0);
  q_down.setRPY(0, M_PI/2, 0);
  q_left.setRPY(0, 0, M_PI/2);
  q_right.setRPY(0, 0, -M_PI/2);
  std::vector<tf::Quaternion> directions;
  directions.push_back(q_up);
  directions.push_back(q_down);
  directions.push_back(q_left);
  directions.push_back(q_right);

  tf::Vector3 position = tflib::to_tf(pose.pose.position);
  tf::Quaternion q = tflib::to_tf(pose.pose.orientation);
  tf::Vector3 unit(1, 0, 0);
  double closest_obstacle_distance = obstacle_check_radius;
  
  direction.normalize();
  tf::Vector3 up = tf::Transform(q_up*q)*unit;
  up.normalize();

  tf::Vector3 side = up.cross(direction);
  side.normalize();

  up = side.cross(direction);
  up.normalize();

  std::vector<tf::Vector3> direction_vectors;
  direction_vectors.push_back(up);
  direction_vectors.push_back(side);
  direction_vectors.push_back(-up);
  direction_vectors.push_back(-side);

  for(int m = 0; m < directions.size(); m++){
    tf::Quaternion q_curr = directions[m];
    for(int k = 1; k < obstacle_check_points+1; k++){
      double dist = (double)k*obstacle_check_radius/(double)(obstacle_check_points+1);
      
      tf::Vector3 point = position + dist*direction_vectors[m];//tf::Transform(q_curr)*(dist*direction);
      geometry_msgs::PoseStamped check_pose;
      check_pose.header = pose.header;
      check_pose.pose.position.x = point.x();
      check_pose.pose.position.y = point.y();
      check_pose.pose.position.z = point.z();
      check_pose.pose.orientation.w = 1.0;

      double occupancy;
      bool collision = !disp_graph->isStateValidDepth_pose(check_pose, 0.9, occupancy);
      
      std_msgs::ColorRGBA c;
      if(collision){
	closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
	c = red;
      }
      else
	c = green;
      
      points.points.push_back(check_pose.pose.position);
      points.colors.push_back(c);
    }
  }
  
  points.points.push_back(pose.pose.position);
  points.colors.push_back(green);
  
  
  double occupancy;
  if(!disp_graph->isStateValidDepth_pose(pose, 0.9, occupancy))
    return 0.f;

  return closest_obstacle_distance;
}

void DisparityMapRepresentation::publish_debug(){
  points.header.stamp = ros::Time::now();
  markers.markers.push_back(points);

  debug_pub.publish(markers);

  markers.markers.clear();
  points.points.clear();
  points.colors.clear();
}

/*
DisparityMapRepresentation::DisparityMapRepresentation(ros::NodeHandle* nh){
  // init params
  target_frame = nh->param("target_frame", std::string("world"));
  distance_threshold = nh->param("distance_threshold", 0.5);
  angle_threshold = M_PI/180.*nh->param("angle_threshold", 5.);
  buffer_size = nh->param("buffer_size", 10);
  
  got_camera_info = false;
  got_odom = false;

  debug_image = NULL;

  // subscribers
  foreground_disparity_sub =
    new message_filters::Subscriber<sensor_msgs::Image>(*nh, "/foreground_disparity", 1);
  background_disparity_sub =
    new message_filters::Subscriber<sensor_msgs::Image>(*nh, "/background_disparity", 1);
  fg_bg_sync =
    new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
    (*foreground_disparity_sub, *background_disparity_sub, 10);
  fg_bg_sync->registerCallback(boost::bind(&DisparityMapRepresentation::fg_bg_disparity_callback,
					   this, _1, _2));
  camera_info_sub = nh->subscribe("/camera_info", 1,
				  &DisparityMapRepresentation::camera_info_callback, this);
  odom_sub = nh->subscribe("/odometry", 1, &DisparityMapRepresentation::odom_callback, this);
  listener = new tf::TransformListener();

  // publishers
  debug_pub = nh->advertise<sensor_msgs::Image>("debug_image", 10);
}

void DisparityMapRepresentation::fg_bg_disparity_callback
(const sensor_msgs::ImageConstPtr& foreground,
 const sensor_msgs::ImageConstPtr& background){
  if(!got_camera_info || !got_odom)
    return;

  // convert the message to cv_bridge type
  ImagePair image_pair;
  try{
    image_pair.foreground = cv_bridge::toCvCopy(*foreground, foreground->encoding);
    image_pair.background = cv_bridge::toCvCopy(*background, background->encoding);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR_STREAM("cv_bridge exception: %s" << e.what());
    return;
  }

  // get the tf
  try{
    listener->waitForTransform(foreground->header.frame_id, target_frame, foreground->header.stamp,
			       ros::Duration(0.1));
    listener->lookupTransform(foreground->header.frame_id, target_frame, foreground->header.stamp,
			      image_pair.target_frame_to_image_tf);
  }
  catch(tf::TransformException& e){
    ROS_ERROR_STREAM("transform exception: " << e.what());
  }

  // for debugging
  if(debug_image == NULL)
    debug_image = new cv::Mat(foreground->height, foreground->width, CV_8UC1);

  // always keeps the most recent image pair at the beginning of the vector
  if(image_pairs.size() == 0)
    image_pairs.push_back(image_pair);

  image_pairs[0] = image_pair;

  // check if we should keep the image pair in the vector
  if(should_add_image_pair()){
    // TODO: fill this in
  }
}

bool DisparityMapRepresentation::should_add_image_pair(){
  // TODO: fill this in
  return false;
}

void DisparityMapRepresentation::camera_info_callback(sensor_msgs::CameraInfo info){
  fx = info.P[0];
  fy = info.P[5];
  cx = info.P[2];
  cy = info.P[6];

  baseline = -info.P[3]/info.P[0];

  got_camera_info = true;
}

void DisparityMapRepresentation::odom_callback(nav_msgs::Odometry odom){
  this->odom = odom;
  got_odom = true;
}

bool DisparityMapRepresentation::is_obstacle(geometry_msgs::PoseStamped pose, double threshold){
  tf::Vector3 point_target_frame;
  double occupancy = 0;
  
  // transform the pose into the target frame
  try{
    tf::StampedTransform pose_to_target_frame_tf;
    listener->waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp,
			       ros::Duration(0.1));
    listener->lookupTransform(target_frame, pose.header.frame_id, pose.header.stamp,
			      pose_to_target_frame_tf);

    tf::Vector3 point_pose_frame(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    point_target_frame = pose_to_target_frame_tf*point_pose_frame;
  }
  catch(tf::TransformException& e){
    ROS_ERROR_STREAM("transform exception: " << e.what());
    return false;
  }


  for(int i = 0; i < image_pairs.size(); i++){
    ImagePair image_pair = image_pairs[i];
    
    // project the point into the image
    tf::Vector3 point_image_frame = image_pair.target_frame_to_image_tf*point_target_frame;
    //ROS_INFO_STREAM(fx << " " << fy << " " << cx << " " << cy << " " << baseline);
    int c = point_image_frame.x()/point_image_frame.z()*fx + cx;
    int r = point_image_frame.y()/point_image_frame.z()*fy + cy;;
    double point_disparity = baseline*fx/point_image_frame.z();
    //ROS_INFO_STREAM(point_image_frame.x() << " " << point_image_frame.y() << " " << point_image_frame.z() << " " << c << " " << r);

    // check occupancy
    if(point_image_frame.z() >= 0 &&
       c >= 0 && c < image_pair.foreground->image.cols &&
       r >= 0 && r < image_pair.foreground->image.rows){
      
      // if the point is between the foreground and background
      if(image_pair.foreground->image.at<float>(r, c) > point_disparity &&
	 image_pair.background->image.at<float>(r, c) < point_disparity){
	occupancy += (point_disparity - 0.5)/point_disparity;

	ROS_INFO_STREAM("disp: " << point_disparity << " " << image_pair.foreground->image.at<float>(r, c) << " " << image_pair.background->image.at<float>(r, c) << " " << occupancy);
	// debugging
	debug_image->at<uint8_t>(r, c) = 255;
      }
      else{
	occupancy -= 0.5*(point_disparity - 0.5)/point_disparity;
	occupancy = std::max(occupancy, 0.0);
	
	// debugging
	debug_image->at<uint8_t>(r, c) = 128;
      }


      if(occupancy >= threshold)
	return false;
    }
  }

  return true;
}


void DisparityMapRepresentation::publish_debug(){
  if(debug_image != NULL){
    cv_bridge::CvImage cv_image;
    cv_image.header.frame_id = "";
    cv_image.header.stamp = ros::Time::now();
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_image.image = *debug_image;

    debug_pub.publish(cv_image.toImageMsg());
    *debug_image = 0;
  }
}
*/

PLUGINLIB_EXPORT_CLASS(DisparityMapRepresentation, MapRepresentation)
