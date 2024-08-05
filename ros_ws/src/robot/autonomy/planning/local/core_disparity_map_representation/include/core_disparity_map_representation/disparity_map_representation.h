#ifndef _DISPARITY_MAP_REPRESENTATION_
#define _DISPARITY_MAP_REPRESENTATION_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
//#include <map_representation_interface/map_representation_interface.h>
#include <core_map_representation_interface/map_representation.h>
#include <disparity_graph/disparity_graph.h>
#include <tflib/tflib.h>
#include <core_trajectory_msgs/TrajectoryXYZVYaw.h>

class DisparityMapRepresentation : public MapRepresentation {
 private:
  nabla::disparity_graph::disparity_graph* disp_graph;
  
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker points;

  tf::TransformListener* listener;
  
  ros::Publisher debug_pub;

  int obstacle_check_points;
  double obstacle_check_radius;
  
 public:
  DisparityMapRepresentation();
  virtual double distance_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction);
  virtual void publish_debug();

  //virtual std::vector< std::vector<double> > get_values(std::vector<core_trajectory_msgs::TrajectoryXYZVYaw> trajectories);
  virtual std::vector< std::vector<double> > get_values(std::vector<std::vector<geometry_msgs::PointStamped> > trajectories);
};

/*
struct ImagePair{
  cv_bridge::CvImagePtr foreground;
  cv_bridge::CvImagePtr background;

  tf::StampedTransform target_frame_to_image_tf;
};

class DisparityMapRepresentation {
private:
  std::string target_frame;
  bool got_camera_info;
  double fx, fy, cx, cy, baseline;

  nav_msgs::Odometry odom;
  bool got_odom;
  double distance_threshold, angle_threshold;
  int buffer_size;

  std::vector<ImagePair> image_pairs;

  cv::Mat* debug_image;

  // subscribers
  message_filters::Subscriber<sensor_msgs::Image>* foreground_disparity_sub;
  message_filters::Subscriber<sensor_msgs::Image>* background_disparity_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>* fg_bg_sync;
  ros::Subscriber camera_info_sub, odom_sub;
  tf::TransformListener* listener;

  // publishers
  ros::Publisher debug_pub;
  
public:
  DisparityMapRepresentation(ros::NodeHandle* nh);
  void fg_bg_disparity_callback(const sensor_msgs::ImageConstPtr& foreground,
				const sensor_msgs::ImageConstPtr& background);
  void camera_info_callback(sensor_msgs::CameraInfo info);
  void odom_callback(nav_msgs::Odometry odom);

  void publish_debug();

  bool should_add_image_pair();
  
  bool is_obstacle(geometry_msgs::PoseStamped pose, double threshold);
};
*/
#endif
