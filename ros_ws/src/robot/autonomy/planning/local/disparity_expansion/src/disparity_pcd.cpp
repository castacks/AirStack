/*
* Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

//Outputs a point cloud using disparity images.
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

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class disparity_pcd{
public:
    disparity_pcd(ros::NodeHandle& nh);
    ros::Subscriber cam_info_sub_;
    ros::Subscriber disparity_sub_;
    ros::Publisher disparity_pcd_pub_;

    double baseline;
    double downsample_scale;
    bool got_cam_info;

    void stereoDisparityCb(const sensor_msgs::Image::ConstPtr &msg_disp);
    void callback(const sensor_msgs::Image::ConstPtr &msg_disp, const sensor_msgs::Image::ConstPtr &msg_image);
    void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr &msg_info);

    double fx_,fy_,cx_,cy_;
    unsigned int width,height;
};

void disparity_pcd::getCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg_info)
{
    if(got_cam_info)
        return;
    image_geometry::PinholeCameraModel model_;model_.fromCameraInfo ( msg_info );
    ROS_INFO_ONCE("Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
    cx_ = model_.cx()/downsample_scale;
    cy_ = model_.cy()/downsample_scale;
    fx_ = fy_ = model_.fx()/downsample_scale;
    width = msg_info->width/downsample_scale;
    height = msg_info->height/downsample_scale;
    baseline = -msg_info->P[3]/msg_info->P[0];
    baseline *=downsample_scale;
    got_cam_info = true;
}

void disparity_pcd::stereoDisparityCb(const sensor_msgs::Image::ConstPtr &msg_disp){
    ROS_INFO_ONCE("Disparity CB");

    ros::Time start = ros::Time::now();
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
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(disparity_pcd_pub_.getNumSubscribers() > 0 )//create expansion PCD
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
        cloud->header.seq = msg_disp->header.seq;
        cloud->header.stamp = pcl_conversions::toPCL(msg_disp->header.stamp);

        sensor_msgs::PointCloud2 cloud_PC2;
        pcl::toROSMsg(*cloud,cloud_PC2);
        disparity_pcd_pub_.publish(cloud_PC2);
    }

//    ROS_INFO("Time: %f \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
    return;
}

void disparity_pcd::callback(const sensor_msgs::Image::ConstPtr &msg_disp, const sensor_msgs::Image::ConstPtr &msg_image)
{
    if(!got_cam_info)
    {
        ROS_INFO_THROTTLE(1,"Cam Info not received, not processing");
        return;
    }
    ROS_INFO_ONCE("Disparity CB");

    ros::Time start = ros::Time::now();
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
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(disparity_pcd_pub_.getNumSubscribers() > 0 )//create expansion PCD
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
        cloud->header.seq = msg_disp->header.seq;
        cloud->header.stamp = pcl_conversions::toPCL(msg_disp->header.stamp);

        sensor_msgs::PointCloud2 cloud_PC2;
        pcl::toROSMsg(*cloud,cloud_PC2);
        disparity_pcd_pub_.publish(cloud_PC2);
    }

//    ROS_INFO("Time: %f \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
    return;
}

disparity_pcd::disparity_pcd(ros::NodeHandle& nh){
    disparity_pcd_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/pcd", 10);
    cam_info_sub_ = nh.subscribe("/nerian_sp1/right/camera_info", 1,&disparity_pcd::getCamInfo,this);
    nh.param("downsample_scale", downsample_scale, 1.0);
    ROS_INFO("into constr");
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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disparity_pcd");
    //	cv::initModule_nonfree();//THIS LINE IS IMPORTANT for using surf and sift features of opencv
    ros::NodeHandle nh("~");
    disparity_pcd d(nh);
    message_filters::Subscriber<sensor_msgs::Image> disp_sub(nh, "/disparity", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/nerian_sp1/left/image_raw", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(disp_sub, image_sub, 10);
    sync.registerCallback(boost::bind(&disparity_pcd::callback,&d, _1, _2));
    ros::spin();
    return 0;
}
