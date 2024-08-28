/*
 * Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

// Postprocessing of disparity image.
/*

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>

#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"

*/

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.hpp>
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

#include <cstdio>
#include <fstream>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

class disparity_conv : public rclcpp::Node {
   public:
    disparity_conv(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg_disp,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msg_image);

   private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_conv_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_image_pub_;

    double baseline;

    void stereoDisparityCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg_disp);
    // void callback(const sensor_msgs::msg::Image::SharedPtr msg_disp,
    // const sensor_msgs::msg::Image::SharedPtr msg_image);

    double fx_, fy_, cx_, cy_;
    unsigned int width, height;
    cv::Mat prev_disp;
    bool first;
};

// ###########################################################################################################
// ###########################################################################################################
void disparity_conv::stereoDisparityCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg_disp) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Disparity CB");

    rclcpp::Time start = this->get_clock()->now();
    cv_bridge::CvImageConstPtr cv_ptrdisparity;
    // cv_bridge::CvImagePtr di_msg(new cv_bridge::CvImage());
    cv_bridge::CvImagePtr di_msg = std::make_shared<cv_bridge::CvImage>();
    cv::Mat disparity32F;
    disparity32F = cv::Mat::zeros(msg_disp->height, msg_disp->width, CV_32FC1);
    try {
        cv_ptrdisparity = cv_bridge::toCvShare(msg_disp);
        cv_ptrdisparity->image.convertTo(disparity32F, CV_32F);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask = disparity32F == 4095.0;
    disparity32F.setTo(NAN, mask);       // setting invalid entries tp 0 or NAN
    disparity32F = disparity32F / 16.0;  // actual disparities are obtained here as floats

    //    if(cv_ptrdisparity->image.type()!=CV_32F)
    //    {
    //        //convert invalidate disparity to zero
    //        for (int32_t j = 0; j< disparity32F.rows ; j++)
    //        {
    //            for (int32_t i = 0; i< disparity32F.cols ; i++)
    //            {
    //                u_int16_t val = cv_ptrdisparity->image.at<u_int16_t>(j,i);
    //                if (val == 0xFFF)
    //                {
    //                    disparity32F.at<float>(j,i) = 0.0f;
    //                }
    //                else
    //                {
    //                    disparity32F.at<float>(j,i) = float(val/16);
    //                }
    //            }
    //        }
    //    }

    int filt_mode = 2;
    if (filt_mode == 1) {
        int dilation_size = 3;
        cv::Mat disparity32F_filtered, disparity32F_filtered2;
        cv::Mat element = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
            cv::Point(dilation_size, dilation_size));
        cv::erode(disparity32F, disparity32F_filtered, element, cv::Point(-1, -1), 2);
        //    cv::dilate(disparity32F_filtered2,disparity32F_filtered,element, cv::Point(-1, -1),
        //    20);

        disparity32F_filtered.convertTo(disparity32F_filtered2, CV_8U);

        disparity32F_filtered.copyTo(di_msg->image, disparity32F_filtered2);
    } else if (filt_mode == 2) {
        int kernel_size = 5;
        double sig = 1, th = 0, lm = 1.0, gm = 0.02, ps = 0;
        cv::Mat kernel =
            cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
        cv::filter2D(disparity32F, di_msg->image, CV_32F, kernel);
    } else {
        disparity32F.copyTo(di_msg->image);
    }
    di_msg->header = msg_disp->header;
    di_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    //    ROS_INFO("Time: %f
    //    \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
    disparity_conv_pub_->publish(*(di_msg->toImageMsg()));
    return;
}

// ####################################################################################################################
// ###################################################################################################################

void disparity_conv::callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg_disp,
                              const sensor_msgs::msg::Image::ConstSharedPtr &msg_image) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Callback triggered");

    rclcpp::Time start = this->get_clock()->now();
    cv_bridge::CvImageConstPtr cv_ptrdisparity;
    cv_bridge::CvImageConstPtr cv_ptrImage;
    // cv_bridge::CvImagePtr di_msg(new cv_bridge::CvImage());
    cv_bridge::CvImagePtr di_msg = std::make_shared<cv_bridge::CvImage>();

    cv::Mat disparity32F, image32F;
    disparity32F = cv::Mat::zeros(msg_disp->height, msg_disp->width, CV_32FC1);
    image32F = cv::Mat::zeros(msg_image->height, msg_image->width, CV_32FC1);

    try {
        cv_ptrdisparity = cv_bridge::toCvShare(msg_disp);

        if (1)  // speckle filter
        {
            cv::Mat speckleFiltered;
            cv_ptrdisparity->image.convertTo(speckleFiltered, CV_16SC1);
            cv::filterSpeckles(speckleFiltered, 4095, 100, 4);

            speckleFiltered.convertTo(disparity32F, CV_32F);
        } else
            cv_ptrdisparity->image.convertTo(disparity32F, CV_32F);
        cv_ptrImage = cv_bridge::toCvShare(msg_image);
        cv_ptrImage->image.convertTo(image32F, CV_32F);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask = disparity32F == 4095.0;
    disparity32F.setTo(NAN, mask);       // setting invalid entries to 0 or NAN
    disparity32F = disparity32F / 16.0;  // actual disparities are obtained here as floats

    int filt_mode = 3;
    if (filt_mode == 2) {
        int kernel_size = 150;
        double sig = 1, th = 0, lm = 1.0, gm = 0.02, ps = 0;
        cv::Mat kernel =
            cv::getGaborKernel(cv::Size(kernel_size, kernel_size), sig, th, lm, gm, ps);
        cv::filter2D(image32F, di_msg->image, CV_32F, kernel);
    } else if (filt_mode == 3) {
        cv::GaussianBlur(image32F, image32F, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(image32F, di_msg->image, CV_32FC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
        //        di_msg->image.copyTo(image32F);
        //        cv::convertScaleAbs( image32F, di_msg->image );
        di_msg->image = cv::abs(di_msg->image);
        cv::Mat kernel = cv::Mat::ones(1, 2, CV_32FC1);  // 15
        cv::Mat acc;
        cv::filter2D(di_msg->image, acc, CV_32FC1, kernel);
        mask = acc > 10.0;  // 2
        di_msg->image.setTo(NAN);

        if (0)  // dilate
        {
            int dilation_size = 2;
            cv::Mat disparity32F_filtered, disparity32F_filtered2;
            disparity32F.copyTo(disparity32F_filtered, mask);
            cv::Mat element = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                cv::Point(dilation_size, dilation_size));
            //            cv::erode(disparity32F_filtered2,disparity32F_filtered,element,
            //            cv::Point(-1, -1), 2);
            cv::dilate(disparity32F_filtered, disparity32F_filtered2, element, cv::Point(-1, -1),
                       2);
            disparity32F_filtered2.convertTo(disparity32F_filtered, CV_8U);
            disparity32F.copyTo(di_msg->image, disparity32F_filtered);
        } else
            disparity32F.copyTo(di_msg->image, mask);
    } else {
        disparity32F.copyTo(di_msg->image);
    }

    // TEMPORAL DIFFERENCING FILTER
    cv::Mat mask2;
    if (0) {
        if (first) {
            disparity32F.copyTo(prev_disp);
            first = false;
        }
        cv::Mat diff;
        cv::absdiff(disparity32F, prev_disp, diff);
        mask2 = diff < 80.0;
        disparity32F.copyTo(di_msg->image, mask2);
        disparity32F.copyTo(prev_disp);
        //        disparity32F.copyTo(di_msg->image,mask);
    }

    di_msg->header = msg_disp->header;
    di_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    //    ROS_INFO("Time: %f
    //    \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
    disparity_conv_pub_->publish(*(di_msg->toImageMsg()));
    image32F.convertTo(image32F, CV_8U);
    di_msg->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    di_msg->image.setTo(0.0);
    image32F.copyTo(di_msg->image, mask);
    filtered_image_pub_->publish(*(di_msg->toImageMsg()));
    return;
}

// #############################################################################################################################################
// ############################################################################################################################################

disparity_conv::disparity_conv(const rclcpp::NodeOptions &options)
    : Node("disparity_conv", options)

{
    disparity_conv_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("/nerian_sp1/disparity_map_32F", 10);
    filtered_image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("/nerian_sp1/left/image_filtered_sky", 10);

    RCLCPP_INFO(this->get_logger(), "into constr");
    // disparity_conv_pub_ = nh.advertise<sensor_msgs::Image>("/nerian_sp1/disparity_map_32F", 10);
    // filtered_image_pub_ = nh.advertise<sensor_msgs::Image>("/nerian_sp1/left/image_filtered_sky",
    // 10); ROS_INFO("into constr");
    //    disparity_sub_ = nh.subscribe("/nerian_sp1/disparity_map",
    //    1,&disparity_conv::stereoDisparityCb,this);

    //    Q-Matrix
    //    [1.0     0.0     0.0     -317.20617294311523,
    //     0.0     1.0     0.0     -233.2914752960205,
    //     0.0     0.0     0.0      307.4838344732113,
    //     0.0     0.0     1.7930881143612616  -0.0]

    cx_ = 317.20617294311523;
    cy_ = 233.2914752960205;
    fx_ = fy_ = 307.4838344732113;
    baseline = 0.5576007548439765;
    width = 640;
    height = 480;
    first = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //	cv::initModule_nonfree();//THIS LINE IS IMPORTANT for using surf and sift features of opencv
    auto node = std::make_shared<disparity_conv>(rclcpp::NodeOptions());

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image>;

    auto disp_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node.get(), "/nerian_sp1/disparity_map", rmw_qos_profile_default);
    auto image_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node.get(), "/nerian_sp1/left/image_raw", rmw_qos_profile_default);

    auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),
                                                                            *disp_sub, *image_sub);

    sync->registerCallback(
        std::bind(&disparity_conv::callback, node, std::placeholders::_1, std::placeholders::_2));

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
