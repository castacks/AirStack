// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class DisparityExpansionNode : public rclcpp::Node {
   protected:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr expanded_disparity_fg_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr expanded_disparity_bg_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr expansion_poly_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expansion_cloud_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frustum_pub;

    bool LUT_ready = false;
    bool got_cam_info = false;

    double downsample_scale;
    image_geometry::PinholeCameraModel model_;
    double baseline;

    struct LUTCell {
        unsigned int idx1;
        unsigned int idx2;
    };
    int lut_max_disparity;
    double expansion_radius;
    double padding;
    double bg_multiplier;
    double pixel_error;
    double metric_depth_scale;
    std::vector<std::vector<LUTCell>> table_u;
    std::vector<std::vector<LUTCell>> table_v;
    std::vector<double> table_d;
    double fx, fy, cx, cy;
    unsigned int width, height;

    cv::Mat convert_depth_to_disparity(const cv::Mat& depth_image);

    void generate_expansion_lookup_table();

   public:
    DisparityExpansionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void set_cam_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_info);

    /**
     * @brief Use depth image input instead of disparity. Converts depth to disparity, then call
     * process_disparity_image
     *
     * @param msg
     */
    void process_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    void process_disparity_image(const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp);

    void publish_expansion_poly(const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp,
                                const cv_bridge::CvImagePtr& fg_msg,
                                const cv_bridge::CvImagePtr& bg_msg);

    void publish_expansion_cloud(const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp,
                                 const cv_bridge::CvImageConstPtr& cv_ptrdisparity,
                                 const cv_bridge::CvImagePtr& fg_msg,
                                 const cv_bridge::CvImagePtr& bg_msg);

    void publish_frustum(const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg);
}; 
