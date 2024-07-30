/**
 * @attention Copyright (C) 2017
 * @attention Carnegie Mellon University
 * @attention All rights reserved
 *
 * @author: AirLab / Field Robotics Center
 * @author: Geetesh Dubey
 *
 * @attention This code was modified under award #A018532.
 * @attention LIMITED RIGHTS:
 * @attention The US Government is granted Limited Rights to this Data.
 *            Use, duplication, or disclosure is subject to the
 *            restrictions as stated in award #A014692
 *  @author: Geetesh Dubey
 *
 */
/*
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "visualization_msgs/MarkerArray.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "deque"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include "image_geometry/pinhole_camera_model.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>

#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"

#include <boost/thread/mutex.hpp>
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <deque>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_geometry/pinhole_camera_model.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

namespace nabla { namespace disparity_graph {

class disparity_graph : public rclcpp::Node {

private:
 struct node {
   //sensor_msgs::Image Im_fg;
   //sensor_msgs::Image Im_bg;
   cv_bridge::CvImagePtr Im_fg;
   cv_bridge::CvImagePtr Im_bg;
   std_msgs::msg::Header header;
   tf2::Transform w2s_tf;
   tf2::Transform s2w_tf;
 };

public:
 void disp_cb(const sensor_msgs::msg::Image::ConstPtr &disp_fg,
              const sensor_msgs::msg::Image::ConstPtr &disp_bg);

 std::deque<node> disp_graph;
 size_t graph_size;
 double thresh_;
 //tf::TransformListener listener;
 std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
 std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
 std::string sensor_frame, fixed_frame, stabilized_frame;
 visualization_msgs::msg::Marker marker;
 //ros::Publisher disparity_graph_marker_pub;
 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr disparity_graph_marker_pub; 
 disparity_graph();
 rclcpp::TimerBase::SharedPtr timer1;
 double angle_tol, displacement_tol;
 bool first;
 bool got_cam_info;
 double fx_,fy_,cx_,cy_,baseline_,downsample_scale;
 unsigned int width_,height_;
 //ros::Subscriber cam_info_sub_;
 rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_; 
 //boost::mutex io_mutex;
 std::mutex io_mutex;
 message_filters::Subscriber<sensor_msgs::msg::Image> disp_fg_sub_, disp_bg_sub_;
 typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
 typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
 std::shared_ptr<ExactSync> exact_sync_;
 void getCamInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg_info);
 bool isStateValidDepth_pose(geometry_msgs::msg::PoseStamped checked_state, double thresh, double& occupancy);
 bool getStateCost(geometry_msgs::msg::Pose checked_state, double &cost);

 //TEST PCD
 PointCloud pcd_checked_states;
 rclcpp::TimerBase::SharedPtr pcd_test_timer;
 rclcpp::TimerBase::SharedPtr pcd_test3_timer;
 rclcpp::TimerBase::SharedPtr pcd_test4_timer;

 void pcd_test();
 //void pcd_test2(const ros::TimerEvent &event);
 void pcd_test3();
 void pcd_test4();
 void clear_graph() { std::lock_guard<std::mutex> lock(io_mutex); disp_graph.clear(); first = true; RCLCPP_ERROR(this->get_logger(), "<<< DISP GRAPH CLEARED >>>"); };
 //ros::Publisher pcdPub, occPub_;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcdPub, occPub_;
 //nav_msgs::OccupancyGrid occ_map;
 //ros::Publisher expansion_poly_pub;
 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr expansion_poly_pub;
 float orig_z;
};

} }
