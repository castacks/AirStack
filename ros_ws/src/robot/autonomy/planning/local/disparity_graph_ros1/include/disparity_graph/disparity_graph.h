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
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/mutex.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "deque"
#include "image_geometry/pinhole_camera_model.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/core/core.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

namespace nabla {
namespace disparity_graph {

class disparity_graph {
   private:
    struct node {
        // sensor_msgs::Image Im_fg;
        // sensor_msgs::Image Im_bg;
        cv_bridge::CvImagePtr Im_fg;
        cv_bridge::CvImagePtr Im_bg;
        std_msgs::Header header;
        tf::Transform w2s_tf;
        tf::Transform s2w_tf;
    };

   public:
    void disp_cb(const sensor_msgs::Image::ConstPtr &disp_fg,
                 const sensor_msgs::Image::ConstPtr &disp_bg);

    std::deque<node> disp_graph;
    size_t graph_size;
    double thresh_;
    tf::TransformListener listener;
    std::string sensor_frame, fixed_frame, stabilized_frame;
    visualization_msgs::Marker marker;
    ros::Publisher disparity_graph_marker_pub;
    disparity_graph();
    ros::Timer timer1;
    double angle_tol, displacement_tol;
    bool first;
    bool got_cam_info;
    double fx_, fy_, cx_, cy_, baseline_, downsample_scale;
    unsigned int width_, height_;
    ros::Subscriber cam_info_sub_;
    boost::mutex io_mutex;
    message_filters::Subscriber<sensor_msgs::Image> disp_fg_sub_, disp_bg_sub_;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>
        ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr &msg_info);
    bool is_state_valid_depth_pose(geometry_msgs::PoseStamped checked_state, double thresh,
                                   double &occupancy);
    bool getStateCost(geometry_msgs::Pose checked_state, double &cost);

    // TEST PCD
    PointCloud pcd_checked_states;
    void pcd_test(const ros::TimerEvent &event);
    // void pcd_test2(const ros::TimerEvent &event);
    void pcd_test3(const ros::TimerEvent &event);
    void pcd_test4(const ros::TimerEvent &event);
    void clear_graph(void) {
        boost::mutex::scoped_lock lock(io_mutex);
        disp_graph.clear();
        first = true;
        ROS_ERROR_STREAM("<<< DISP GRAPH CLEARED >>>");
    };
    ros::Publisher pcdPub, occPub_;
    // nav_msgs::OccupancyGrid occ_map;
    ros::Publisher expansion_poly_pub;
    float orig_z;
};

}  // namespace disparity_graph
}  // namespace nabla
