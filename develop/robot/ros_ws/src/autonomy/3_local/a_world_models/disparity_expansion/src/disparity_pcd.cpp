/*
* Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

//Outputs a point cloud using disparity images.
/*
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
*/

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <image_transport/image_transport.hpp>
//#include <stereo_msgs/msg/disparity_image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <visualization_msgs/msg/marker_array.hpp>
//#include <visualization_msgs/msg/marker.hpp>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdio>
#include <fstream>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class disparity_pcd : public rclcpp::Node
{
public:
    disparity_pcd(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    //ros::Subscriber cam_info_sub_;
    //ros::Subscriber disparity_sub_;
    //ros::Publisher disparity_pcd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr disparity_pcd_pub_;

    double baseline;
    double downsample_scale;
    bool got_cam_info;

    void stereoDisparityCb(const sensor_msgs::msg::Image::ConstSharedPtr msg_disp);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg_disp, const sensor_msgs::msg::Image::ConstSharedPtr msg_image);
    void getCamInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_info);

    double fx_,fy_,cx_,cy_;
    unsigned int width,height;
};

void disparity_pcd::getCamInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_info)
{
    if(got_cam_info)
        return;
    image_geometry::PinholeCameraModel model_;model_.fromCameraInfo ( msg_info );
    RCLCPP_INFO_ONCE(this->get_logger(), "Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
    cx_ = model_.cx()/downsample_scale;
    cy_ = model_.cy()/downsample_scale;
    fx_ = fy_ = model_.fx()/downsample_scale;
    width = msg_info->width/downsample_scale;
    height = msg_info->height/downsample_scale;
    baseline = -msg_info->p[3]/msg_info->p[0];
    baseline *=downsample_scale;
    got_cam_info = true;
}

void disparity_pcd::stereoDisparityCb(const sensor_msgs::msg::Image::ConstSharedPtr msg_disp){
    RCLCPP_INFO_ONCE(this->get_logger(),"Disparity CB");

    rclcpp::Time start = this->get_clock()->now();
    cv_bridge::CvImageConstPtr cv_ptrdisparity;
    cv_bridge::CvImagePtr di_msg(new cv_bridge::CvImage());

    cv::Mat disparity32F;
    disparity32F = cv::Mat::zeros(msg_disp->height,msg_disp->width,CV_32FC1);
    try
    {
        cv_ptrdisparity = cv_bridge::toCvShare(msg_disp);
        disparity32F = cv_ptrdisparity->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if(disparity_pcd_pub_->get_subscription_count() > 0 )//create expansion PCD
    {
        PointCloud::Ptr cloud (new PointCloud);
        cloud->header.frame_id = msg_disp->header.frame_id;
        cloud->height = 1;
        cloud->width =1;
        cloud->is_dense = false;
        int point_counter=0;
        pcl::PointXYZI pt_;
        for ( int v = ( int ) height - 1; v >= 0; v-=4 )
        {
            for ( int u = ( int ) width - 1; u >= 0; u-=4 )
            {

                // Fill in XYZ
                float depth_value =  depth_value = baseline * fx_ / disparity32F.at<float>(v,u);//new_disparity_bridgePtr->image.at<float>(v,u);
                        pt_.x = ( u - cx_ ) * depth_value / fx_;
                        pt_.y = ( v - cy_ ) * depth_value / fy_;
                        pt_.z = depth_value;
                        pt_.intensity = 170;
//                        if(fg_msg->image.at<float>(v,u) >= 0.1 && bg_msg->image.at<float>(v,u) >= 0.1 )
//                        if(new_disparity_bridgePtr->image.at<float>(v,u) > 0.1)
                {
//                            ROS_INFO("depth: %f",new_disparity_bridgePtr->image.at<float>(v,u));
                    point_counter++;
                    cloud->points.push_back ( pt_ );
                }
            }
        }
        cloud->width  = point_counter;
        //cloud->header.seq = msg_disp->header.seq;
        cloud->header.stamp = rclcpp::Time(msg_disp->header.stamp).nanoseconds();
        cloud->header.stamp = pcl_conversions::toPCL(rclcpp::Time(msg_disp->header.stamp));
    

        sensor_msgs::msg::PointCloud2 cloud_PC2;
        pcl::toROSMsg(*cloud,cloud_PC2);
        disparity_pcd_pub_->publish(cloud_PC2);
    }

//    ROS_INFO("Time: %f \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
    return;
}

void disparity_pcd::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg_disp, const sensor_msgs::msg::Image::ConstSharedPtr msg_image)
{
    if(!got_cam_info)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1,"Cam Info not received, not processing");
        return;
    }
    RCLCPP_INFO_ONCE(this->get_logger(),"Disparity CB");

    rclcpp::Time start = this->get_clock()->now();
    cv_bridge::CvImageConstPtr cv_ptrdisparity;
    cv_bridge::CvImageConstPtr cv_ptrImage;
    cv_bridge::CvImagePtr di_msg(new cv_bridge::CvImage());

    cv::Mat disparity32F,image32F;
    disparity32F = cv::Mat::zeros(msg_disp->height,msg_disp->width,CV_32FC1);
    image32F = cv::Mat::zeros(msg_image->height,msg_image->width,CV_32FC1);
    try
    {
        cv_ptrdisparity = cv_bridge::toCvShare(msg_disp);
        disparity32F = cv_ptrdisparity->image;

        cv_ptrImage= cv_bridge::toCvShare(msg_image);
        cv_ptrImage->image.convertTo(image32F,CV_32F);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
        return;
    }

    if(disparity_pcd_pub_->get_subscription_count() > 0 )//create expansion PCD
    {
        PointCloud::Ptr cloud (new PointCloud);
        cloud->header.frame_id = msg_disp->header.frame_id;
        cloud->height = 1;
        cloud->width =1;
        cloud->is_dense = false;
        int point_counter=0;
        pcl::PointXYZI pt_;
        for ( int v = ( int ) height - 1; v >= 0; v-=4 )
        {
            for ( int u = ( int ) width - 1; u >= 0; u-=4 )
            {

                // Fill in XYZ
                float depth_value =  depth_value = baseline * fx_ / disparity32F.at<float>(v,u);//new_disparity_bridgePtr->image.at<float>(v,u);
                        pt_.x = ( u - cx_ ) * depth_value / fx_;
                        pt_.y = ( v - cy_ ) * depth_value / fy_;
                        pt_.z = depth_value;
                        pt_.intensity = image32F.at<float>(v,u);
//                        if(fg_msg->image.at<float>(v,u) >= 0.1 && bg_msg->image.at<float>(v,u) >= 0.1 )
//                        if(new_disparity_bridgePtr->image.at<float>(v,u) > 0.1)
                {
//                            ROS_INFO("depth: %f",new_disparity_bridgePtr->image.at<float>(v,u));
                    point_counter++;
                    cloud->points.push_back ( pt_ );
                }
            }
        }
        cloud->width  = point_counter;
        //cloud->header.seq = msg_disp->header.seq;
        //cloud->header.stamp = pcl_conversions::toPCL(msg_disp->header.stamp);
        cloud->header.stamp = rclcpp::Time(msg_disp->header.stamp).nanoseconds();
        cloud->header.stamp = pcl_conversions::toPCL(rclcpp::Time(msg_disp->header.stamp));

        sensor_msgs::msg::PointCloud2 cloud_PC2;
        pcl::toROSMsg(*cloud,cloud_PC2);
        disparity_pcd_pub_->publish(cloud_PC2);
    }

//    ROS_INFO("Time: %f \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
    return;
}

disparity_pcd::disparity_pcd(const rclcpp::NodeOptions & options)
: Node("disparity_pcd", options)
{
    disparity_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd", 10);
    //cam_info_sub_ = nh.subscribe("/nerian_sp1/right/camera_info", 1,&disparity_pcd::getCamInfo,this);
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/nerian_sp1/right/camera_info", 10, std::bind(&disparity_pcd::getCamInfo, this, std::placeholders::_1));

    //nh.param("downsample_scale", downsample_scale, 1.0);
    //ROS_INFO("into constr");
    this->declare_parameter("downsample_scale", 1.0);
    this->get_parameter("downsample_scale", downsample_scale);

    RCLCPP_INFO(this->get_logger(), "into constr");
//    disparity_sub_ = nh.subscribe("/disparity", 1,&disparity_pcd::stereoDisparityCb,this);

    //    Q-Matrix
    //    [1.0     0.0     0.0     -317.20617294311523,
    //     0.0     1.0     0.0     -233.2914752960205,
    //     0.0     0.0     0.0      307.4838344732113,
    //     0.0     0.0     1.7930881143612616  -0.0]

//    cx_ = 317.20617294311523;
//    cy_ = 233.2914752960205;
//    fx_ = fy_ = 307.4838344732113;
//    baseline = 0.5576007548439765;
//    downsample_scale =1.0;
    got_cam_info = false;
//    width = 640;
//    height = 480;

//    auto disp_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node.get(), "/disparity", 10);
//    auto image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "/nerian_sp1/left/image_raw", 10);

//    auto sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(*disp_sub_, *image_sub_, 10);
//    sync->registerCallback(std::bind(&disparity_pcd::callback, this, std::placeholders::_1, std::placeholders::_2));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    //	cv::initModule_nonfree();//THIS LINE IS IMPORTANT for using surf and sift features of opencv
    //ros::NodeHandle nh("~");
    auto node = std::make_shared<disparity_pcd>(rclcpp::NodeOptions());

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image>;
    auto disp_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node.get(), "/nerian_sp1/disparity_map", rmw_qos_profile_default);
    auto image_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node.get(), "/nerian_sp1/left/image_raw", rmw_qos_profile_default);

    auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),
                                                                            *disp_sub, *image_sub);

    sync->registerCallback(
        std::bind(&disparity_pcd::callback, node, std::placeholders::_1, std::placeholders::_2));


    //disparity_pcd d(nh);
    //message_filters::Subscriber<sensor_msgs::Image> disp_sub(nh, "/disparity", 1);
    //message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/nerian_sp1/left/image_raw", 1);
    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(disp_sub, image_sub, 10);
    //sync.registerCallback(boost::bind(&disparity_pcd::callback,&d, _1, _2));
   
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
