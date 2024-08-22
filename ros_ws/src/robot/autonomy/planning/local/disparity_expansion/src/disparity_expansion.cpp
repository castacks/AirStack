/*
 * Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

// Applies C-Space expansion on disparity images.
/*
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>

#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "geometry_msgs/PolygonStamped.h"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include "stereo_msgs/DisparityImage.h"
#include "tf/tf.h"
#include "visualization_msgs/MarkerArray.h"
*/

#define SCALE 2.0  // num_units/pixel
// #define SCALE_D 1000.0
//  #define lut_max_disparity_   165 // units
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
// #include <pcl_ros/point_cloud.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #define SCALE 2.0 // num_units/pixel
// #define lut_max_disparity_   165 // units
// #define robot_radius_      2.0 // in meters
// for 2times Robsize = 0.0173 at 40m
// 15 = 0.0677
#define DEPTH_ERROR_COEFF 0.0177
// #define robot_radius_      2.0 // in meters
// for 2times Robsize = 0.0173 at 40m
// 15 = 0.0677/
// #define DEPTH_ERROR_COEFF 0.0177

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class DisparityExpansionNode : public rclcpp::Node {
   public:
    DisparityExpansionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expansion_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr expanded_disparity_fg_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr expanded_disparity_bg_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr expansion_poly_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;

    double baseline;
    double downsample_scale;
    bool LUT_ready;
    bool got_cam_info;
    image_geometry::PinholeCameraModel model_;

    void getCamInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_info);
    void stereoDisparityCb(const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp);
    void generateExpansionLUT();

    struct cell {
        unsigned int idx1;
        unsigned int idx2;
    };
    int lut_max_disparity_;
    double robot_radius_;
    double padding_;
    double bg_multiplier_;
    double pixel_error_;
    std::vector<std::vector<cell>> table_u;
    std::vector<std::vector<cell>> table_v;
    // cell table_u[200][640];
    // cell table_v[200][480];
    double table_d[200];
    double fx_, fy_, cx_, cy_;
    unsigned int width, height;
};

// ################################################################################################################################################################################
// ################################################################################################################################################################################

void DisparityExpansionNode::generateExpansionLUT() {
    bool debug = false;
    if (debug) {
        RCLCPP_WARN(this->get_logger(), "Expansion LUT Debug disabled creation");
        LUT_ready = true;
    }
    if (LUT_ready) {
        RCLCPP_ERROR(this->get_logger(), "LUT all ready");
        //        sleep(1);
        return;
    }
    double center_x = cx_;
    double center_y = cy_;
    double constant_x = 1.0 / fx_;
    double constant_y = 1.0 / fy_;
    RCLCPP_INFO(this->get_logger(), "Fx Fy Cx Cy: %f %f , %f %f \nW H Baseline: %d %d %f", fx_, fy_,
                cx_, cy_, width, height, baseline);
    double r = robot_radius_;  // expansion radius in cm
    table_u = std::vector<std::vector<cell>>(lut_max_disparity_, std::vector<cell>(width));
    table_v = std::vector<std::vector<cell>>(lut_max_disparity_, std::vector<cell>(height));
    int u1, u2, v1, v2;
    double x, y, z;
    double disparity;

    for (unsigned int disp_idx = 1; disp_idx < lut_max_disparity_; disp_idx++) {
        //        z = depth*0.01;//cm to m
        disparity = disp_idx / SCALE;  // 1 cell = 0.5m, z is in meters
        r = robot_radius_;             // * exp(DEPTH_ERROR_COEFF*z);
        z = baseline * fx_ / disparity;

        double disp_new = baseline * fx_ / (z - robot_radius_) + 0.5;
        table_d[disp_idx] = disp_new;
        //        ROS_INFO("REAL EXPANDED: %f , %f",disparity,table_d[disp_idx]);

        for (int v = (int)height - 1; v >= 0; --v) {
            y = (v - center_y) * z * constant_y;

            double beta = atan2(z, y);
            double beta1 = asin(r / sqrt(z * z + y * y));

            double r1 = z / tan(beta + beta1);
            double r2 = z / tan(beta - beta1);
            v1 = fy_ * r1 / z + center_y;
            v2 = fy_ * r2 / z + center_y;

            if ((v2 - v1) < 0)
                RCLCPP_ERROR(this->get_logger(), "Something MESSED %d %d %d", v1, v2, disp_idx);

            if (v1 < 0) v1 = 0;
            if (v1 > (height - 1)) v1 = height - 1;

            if (v2 < 0) v2 = 0;
            if (v2 > (height - 1)) v2 = height - 1;

            table_v[disp_idx][v].idx1 = v1;
            table_v[disp_idx][v].idx2 = v2;
        }

        for (int u = (int)width - 1; u >= 0; --u) {
            x = (u - center_x) * z * constant_x;

            double alpha = atan2(z, x);
            double alpha1 = asin(r / sqrt(z * z + x * x));

            double r1 = z / tan(alpha + alpha1);
            double r2 = z / tan(alpha - alpha1);
            u1 = fx_ * r1 / z + center_x;
            u2 = fx_ * r2 / z + center_x;

            if ((u2 - u1) < 0)
                RCLCPP_ERROR(this->get_logger(), "Something MESSED %d %d %d", u1, u2, disp_idx);

            if (u1 < 0) u1 = 0;
            if (u1 > (width - 1)) u1 = width - 1;

            if (u2 < 0) u2 = 0;
            if (u2 > (width - 1)) u2 = width - 1;

            table_u[disp_idx][u].idx1 = u1;
            table_u[disp_idx][u].idx2 = u2;
        }
    }
    //    for ( int u = (width - 1); u >= 0; --u )
    //    {
    //        int d = 199;
    //        ROS_ERROR("window disp_idx u1 u2 \t %d \t%d
    //        \t%d",u,table_u[d][u].idx1,table_u[d][u].idx2);
    //    }
    RCLCPP_WARN(this->get_logger(), "Expansion LUT created: LUT MAX: %d , ROBOT SIZE: %f",
                lut_max_disparity_ / 2, robot_radius_);
    LUT_ready = true;
}

// ################################################################################################################
// ################################################################################################################
void DisparityExpansionNode::getCamInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_info) {
    if (got_cam_info) return;
    model_.fromCameraInfo(msg_info);
    RCLCPP_INFO_ONCE(this->get_logger(), "Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f", model_.fx(),
                     model_.fy(), model_.cx(), model_.cy());
    cx_ = model_.cx() / downsample_scale;
    cy_ = model_.cy() / downsample_scale;
    fx_ = fy_ = model_.fx() / downsample_scale;
    width = msg_info->width / downsample_scale;
    height = msg_info->height / downsample_scale;
    baseline = -msg_info->p[3] / msg_info->p[0];
    baseline = 0.25;
    baseline *= downsample_scale;
    generateExpansionLUT();
    got_cam_info = true;
    RCLCPP_INFO(this->get_logger(), "Baseline: %.4f meters", baseline);
    RCLCPP_INFO(this->get_logger(), "Focal length (fx): %.4f pixels", fx_);
}

void DisparityExpansionNode::stereoDisparityCb(
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr&
        msg_disp) {  // sensor_msgs::Image::ConstPtr &msg_disp){
    //    ROS_INFO("Disparity CB");
    //    dimage = *msg_disp;
    std::cout << "hello" << std::endl;

    if (!LUT_ready) {
        auto& clk = *this->get_clock();
        RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1,
                             "LUT not ready yet, not processing disparity");
        return;
    }
    //        generateExpansionLUT();
    // #################################################################################
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
    img_msg->header = msg_disp->header;
    img_msg->height = msg_disp->image.height;
    img_msg->width = msg_disp->image.width;
    img_msg->encoding = msg_disp->image.encoding;
    img_msg->is_bigendian = msg_disp->image.is_bigendian;
    img_msg->step = msg_disp->image.step;
    img_msg->data = msg_disp->image.data;
    // #########################################################3

    //    auto disparity_img_msg = msg_disp->image;
    cv_bridge::CvImagePtr fg_msg(new cv_bridge::CvImage());
    cv_bridge::CvImagePtr bg_msg(new cv_bridge::CvImage());

    cv_bridge::CvImageConstPtr cv_ptrdisparity;
    try {
        cv_ptrdisparity = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat disparity32F = cv_ptrdisparity->image;
    if (disparity32F.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty disparity image");
        return;
    }
    cv::resize(disparity32F, disparity32F, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    if (disparity32F.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Resized disparity image is empty");
        return;
    }

    cv::Mat disparity_fg;
    cv::Mat disparity_bg;
    cv::Mat disparity32F_bg;

    //    const sensor_msgs::ImageConstPtr my_ptr = &dimage;
    //    cv::Mat dummy(msg_disp->image.height,msg_disp->image.width,CV_32FC4);
    //    dummy.copyTo(free_msg->image);
    //    free_msg->image.zeros(msg_disp->image.height,msg_disp->image.width,CV_32FC4);
    //    free_msg->header = dimage.header;
    //    free_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
    try {
        //        fg_msg = cv_bridge::toCvCopy(dimage, sensor_msgs::image_encodings::TYPE_32FC1);
        //        bg_msg = cv_bridge::toCvCopy(dimage, sensor_msgs::image_encodings::TYPE_32FC1);
        //        di_msg = cv_bridge::toCvCopy(dimage, sensor_msgs::image_encodings::TYPE_32FC1);

        //        fg_msg->image.zeros(disparity32F.rows,disparity32F.cols,CV_32FC1);// =
        //        disparity32F; bg_msg->image.zeros(disparity32F.rows,disparity32F.cols,CV_32FC1);//
        //        .create(disparity32F.rows,disparity32F.cols,CV_32F);// = disparity32F;
        //        di_msg->image.zeros(disparity32F.rows,disparity32F.cols,CV_32FC1);//
        //        .create(disparity32F.rows,disparity32F.cols,CV_32F);// = disparity32F;
        disparity32F.copyTo(fg_msg->image);
        disparity32F.copyTo(bg_msg->image);

        disparity32F.copyTo(disparity_fg);
        disparity32F.copyTo(disparity_bg);
        disparity32F.copyTo(disparity32F_bg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    fg_msg->header = msg_disp->header;
    fg_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    bg_msg->header = msg_disp->header;
    bg_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    //    bg_msg->image = bg_msg->image + INFINITY;
    //    fg_msg->image = fg_msg->image - INFINITY;
    if (1)  // make cloud
    {
        // Use correct principal point from calibration
        float center_x = cx_;
        float center_y = cy_;

        float constant_x = 1.0 / fx_;
        float constant_y = 1.0 / fy_;

        //        if ( di_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 )
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "IMG TYPE 32FC, GOOD");
            //            int row_step = dimage.step / sizeof ( float );
            //            const float* disparity_row = reinterpret_cast<const float*> (
            //            &dimage.data[0] );
            rclcpp::Time start = this->get_clock()->now();

            float padding = padding_;

            for (int v = (int)height - 2; (v >= 0); v -= 1) {
                for (int u = (int)width - 1; u >= 0; u -= 1) {
                    float disparity_value =
                        disparity32F.at<float>(v, u);  // disparity_row[v * row_step + u];// + 0.5;
                    if (!std::isnan(double(disparity_value * SCALE)) &&
                        ((int(disparity_value * SCALE) + 1) < lut_max_disparity_) &&
                        ((int(disparity_value * SCALE) + 1) > 0))  // expand
                    {
                        //                        disparity_value = disparity32F.at<float>(v,u)+0.5;
                        unsigned int u1 = table_u[int(disparity_value * SCALE) + 1][u].idx1;
                        unsigned int u2 = table_u[int(disparity_value * SCALE) + 1][u].idx2;

                        //                        unsigned int v1 =
                        //                        v;//table_v[int(disparity_value*SCALE)+1][v].idx1;
                        //                        unsigned int v2 =
                        //                        v+1;//table_v[int(disparity_value*SCALE)+1][v].idx2;

                        //                        if(v1 <0 || u1 <0 || v2 >=height || u2>=width ||
                        //                        (u2-u1)<=0 || (v2-v1)<=0)
                        //                        {
                        //                            ROS_ERROR("U Expansion out of img bounds with
                        //                            disparity: %f",disparity_value); ROS_ERROR("u1
                        //                            u2 v1 v2 %d %d \t %d %d",u1,u2,v1,v2);
                        //                            continue;
                        //                        }
                        if (disparity32F.empty()) {
                            RCLCPP_ERROR(this->get_logger(), "disparity32F matrix is empty.");
                            return;
                        }

                        // if (u1 < 0 || u1 >= disparity32F.cols || u2 < 0 ||
                        //     u2 >= disparity32F.cols || u1 >= u2) {
                        //     std::cerr << "Invalid ROI indices: u1=" << u1 << ", u2=" << u2
                        //               << std::endl;
                        //     return;

                        cv::Rect roi = cv::Rect(u1, v, (u2 - u1), 1);

                        //      if (roi.width <= 0 || roi.height <= 0 || roi.x < 0 || roi.y < 0 ||
                        //          roi.x + roi.width > disparity32F.cols ||
                        //          roi.y + roi.height > disparity32F.rows) {
                        //          RCLCPP_ERROR(this->get_logger(),
                        //                       "ROI is out of bounds: u1=%d, v=%d, u2=%d, "
                        //                       "disparity32F.cols=%d, disparity32F.rows=%d",
                        //                       u1, v, u2, disparity32F.cols, disparity32F.rows);
                        //          continue;
                        //      }

                        cv::Mat submat_t = disparity32F(roi).clone();

                        double min, max;
                        cv::Point p1, p2;
                        int min_idx, max_idx;
                        //                        cv::minMaxIdx(disparity32F(roi), &min,
                        //                        &max,&min_idx,&max_idx);
                        cv::minMaxLoc(disparity32F(roi), &min, &max, &p1, &p2);
                        min_idx = p1.x;
                        max_idx = p2.x;
                        float disp_new_fg = max;  // = table_d[int(max*SCALE)];
                        float disp_new_bg = min;  // = table_d[int(min*SCALE)];
                        float disp_to_depth = baseline * fx_ / max;

                        cv::Mat submat;
                        cv::divide(baseline * fx_, submat_t, submat);
                        submat = (submat - disp_to_depth);  /// robot_radius_;
                        //                        ROS_INFO("submat at : %f",disp_to_depth);
                        //                        std::cout<<submat<<std::endl;

                        if (padding < 0.0) {
                            float range = bg_multiplier_ * robot_radius_;
                            float max_depth = 0.0;
                            bool found = true;
                            int ctr = 1;
                            while (found) {
                                found = false;
                                for (int j = 0; j < submat.cols; j++) {
                                    float val = submat.at<float>(0, j);
                                    if (std::isfinite(val)) {
                                        if (val < ctr * range && val > max_depth) {
                                            found = true;
                                            max_depth = val;
                                        }
                                    }
                                }
                                ctr++;
                            }
                            disp_to_depth += max_depth;
                        } else
                            disp_to_depth += padding;  // baseline * fx_/min;

                        disp_new_bg =
                            baseline * fx_ / (disp_to_depth /*+ robot_radius_*/) /*- 0.5*/;
                        disparity_fg(roi).setTo(disp_new_fg);
                        disparity_bg(roi).setTo(/*min*/ disp_new_bg);

                        int u_temp = u1 + max_idx;
                        if (u_temp >= u)
                            u = u1;
                        else
                            u = u_temp + 1;
                    } else {
                        RCLCPP_ERROR_STREAM_THROTTLE(
                            this->get_logger(), *this->get_clock(), 1000,
                            "BAD disparity during expansion for u: "
                                << disparity_value << " lut_max_disparity_: " << lut_max_disparity_
                                << " SCALE: " << SCALE);
                    }
                }
            }

            //            fg_msg->image.copyTo(disparity32F);
            disparity_fg.copyTo(disparity32F);
            disparity_bg.copyTo(disparity32F_bg);

            for (int u = (int)width - 2; (u >= 0) && true; u -= 1) {
                for (int v = (int)height - 1; v >= 0; v -= 1) {
                    float disparity_value =
                        disparity32F.at<float>(v, u) +
                        pixel_error_;  // disparity_row[v * row_step + u];// + 0.5;
                    if (!std::isnan(double(disparity_value)) &&
                        ((int(disparity_value * SCALE) + 1) < lut_max_disparity_) &&
                        ((int(disparity_value * SCALE) + 1) > 0))  // expand
                    {
                        unsigned int v1 = table_v[int(disparity_value * SCALE) + 1][v].idx1;
                        unsigned int v2 = table_v[int(disparity_value * SCALE) + 1][v].idx2;

                        cv::Rect roi = cv::Rect(u, v1, 1, (v2 - v1));
                        cv::Mat submat_t = disparity32F_bg(roi).clone();
                        double min, max;
                        cv::Point p1, p2;
                        int min_idx, max_idx;
                        cv::minMaxLoc(disparity32F(roi), &min, &max, &p1, &p2);
                        min_idx = p1.y;
                        max_idx = p2.y;
                        float disp_new_fg;  // = max;// = table_d[int(max*SCALE)];
                        float disp_new_bg;  // = min;// = table_d[int(min*SCALE)];
                        float disp_to_depth = baseline * fx_ / max;
                        disp_new_fg =
                            baseline * fx_ / (disp_to_depth - robot_radius_) + pixel_error_;

                        cv::Mat submat;
                        cv::divide(baseline * fx_, submat_t, submat);
                        cv::minMaxLoc(disparity32F_bg(roi), &min, &max, &p1, &p2);
                        disp_to_depth = baseline * fx_ / max;
                        submat = (submat - disp_to_depth);  /// robot_radius_;
                        //                        ROS_INFO("submat at : %f",disp_to_depth);
                        //                        std::cout<<submat<<std::endl;

                        if (padding < 0.0) {
                            float range = bg_multiplier_ * robot_radius_;
                            float max_depth = 0.0;
                            bool found = true;
                            int ctr = 1;
                            while (found) {
                                found = false;
                                for (int j = 0; j < submat.rows; j++) {
                                    float val = submat.at<float>(j, 0);
                                    if (std::isfinite(val)) {
                                        if (val < ctr * range && val > max_depth) {
                                            found = true;
                                            max_depth = val;
                                        }
                                    }
                                }
                                ctr++;
                            }
                            disp_to_depth += max_depth;
                        } else
                            disp_to_depth += padding;  // baseline * fx_/min;

                        disp_new_bg =
                            baseline * fx_ / (disp_to_depth + robot_radius_) - pixel_error_;
                        disp_new_bg = disp_new_bg < 0.0 ? 0.0001 : disp_new_bg;

                        disparity_fg(roi).setTo(disp_new_fg);
                        disparity_bg(roi).setTo(disp_new_bg);

                        int v_temp = v1 + max_idx;
                        if (v_temp >= v)
                            v = v1;
                        else
                            v = v_temp + 1;
                    } else {
                        RCLCPP_ERROR_STREAM_THROTTLE(
                            this->get_logger(), *this->get_clock(), 1000,
                            "BAD disparity during expansion for v: "
                                << disparity_value << " lut_max_disparity_: " << lut_max_disparity_
                                << " SCALE: " << SCALE);
                    }
                }
            }

            fg_msg->image = disparity_fg;
            bg_msg->image = disparity_bg;

            // RCLCPP_INFO(this->get_logger(), "Time: %f \t%f",
            //             (this->get_clock()->now() - start).seconds(),
            //             1 / (this->get_clock()->now() - start).seconds());

            expanded_disparity_fg_pub->publish(*(fg_msg->toImageMsg()));
            expanded_disparity_bg_pub->publish(*(bg_msg->toImageMsg()));

            if (expansion_poly_pub->get_subscription_count() > 0)  // create expansion PCD
            {
                visualization_msgs::msg::MarkerArray marker_arr;
                visualization_msgs::msg::Marker marker;
                marker.header = msg_disp->header;
                marker.ns = "occ_space";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                //                marker.mesh_resource =
                //                "package://gazebo_robot_description/meshes/octorotor/riverine_octo_m.dae";
                marker.lifetime = rclcpp::Duration::from_seconds(0.5);
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = 0;  // xyz_centroid[0];
                marker.pose.position.y = 0;  // xyz_centroid[1];
                marker.pose.position.z = 0;  // xyz_centroid[2];

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, 0.0);
                marker.pose.orientation = tf2::toMsg(q);
                //                marker.pose.orientation.w = 1;
                float marker_scale = 0.51;
                marker.scale.x = marker_scale;
                marker.scale.y = marker_scale;
                marker.scale.z = marker_scale;
                marker.color.a = 0.3;  // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;

                geometry_msgs::msg::PolygonStamped poly;
                poly.header = msg_disp->header;
                int v = 120;
                float prev_depth = 0.0;
                for (int v = (int)0; v <= 239; v += 10) {
                    for (int u = (int)width - 1; u >= 0; u--) {
                        float depth_value = baseline * fx_ /
                                            fg_msg->image.at<float>(
                                                v, u);  // free_msg->image.at<cv::Vec4f>(v,u)[0];
                        float depth_diff = fabs(depth_value - prev_depth);
                        prev_depth = depth_value;
                        if (!std::isnan(depth_value) && !std::isinf(depth_value) &&
                            depth_diff < 0.5) {
                            marker.color.r = 1.0 * (fg_msg->image.at<float>(v, u) - pixel_error_) /
                                             fg_msg->image.at<float>(v, u);
                            marker.color.g = 1.0 * (pixel_error_) / fg_msg->image.at<float>(v, u);
                            geometry_msgs::msg::Point gm_p;
                            gm_p.x = (u - center_x) * depth_value * constant_x;
                            gm_p.y = (v - center_y) * depth_value * constant_y;
                            gm_p.z = depth_value;
                            marker.points.push_back(gm_p);

                            depth_value = baseline * fx_ / bg_msg->image.at<float>(v, u);
                            gm_p.x = (u - center_x) * depth_value * constant_x;
                            gm_p.y = (v - center_y) * depth_value * constant_y;
                            gm_p.z = depth_value;
                            marker.points.push_back(gm_p);

                            //                        poly.polygon.points.push_back(gm_p);
                        } else {
                            marker_arr.markers.push_back(marker);
                            marker.points.clear();
                            marker.id++;
                        }
                    }
                }
                marker_arr.markers.push_back(marker);
                expansion_poly_pub->publish(marker_arr);
            }

            if (expansion_cloud_pub->get_subscription_count() > 0)  // create expansion PCD
            {
                PointCloud::Ptr cloud(new PointCloud);
                cloud->header.frame_id = msg_disp->header.frame_id;
                cloud->height = 1;
                cloud->width = 1;
                cloud->is_dense = false;
                int point_counter = 0;
                pcl::PointXYZI pt_fg, pt_bg, pt_free1, pt_free2, pt_real;

                for (int v = (int)height - 1; v >= 0; v -= 4) {
                    for (int u = (int)width - 1; u >= 0; u -= 4) {
                        // Fill in XYZ
                        //                        float depth_value = baseline * fx_ /
                        //                        new_disparity_bridgePtr->image.at<float>(v,u);//fg_msg->image.at<float>(v,u);//free_msg->image.at<cv::Vec4f>(v,u)[0];
                        float depth_value = baseline * fx_ /
                                            fg_msg->image.at<float>(
                                                v, u);  // free_msg->image.at<cv::Vec4f>(v,u)[0];
                        pt_fg.x = (u - center_x) * depth_value * constant_x;
                        pt_fg.y = (v - center_y) * depth_value * constant_y;
                        pt_fg.z = depth_value;
                        pt_fg.intensity = 220;

                        depth_value = baseline * fx_ /
                                      bg_msg->image.at<float>(
                                          v, u);  // free_msg->image.at<cv::Vec4f>(v,u)[1];
                        pt_bg.x = (u - center_x) * depth_value * constant_x;
                        pt_bg.y = (v - center_y) * depth_value * constant_y;
                        pt_bg.z = depth_value;
                        pt_bg.intensity = 120;

                        // ACtual PCD
                        depth_value = baseline * fx_ /
                                      cv_ptrdisparity->image.at<float>(
                                          v, u);  // new_disparity_bridgePtr->image.at<float>(v,u);
                        pt_real.x = (u - center_x) * depth_value * constant_x;
                        pt_real.y = (v - center_y) * depth_value * constant_y;
                        pt_real.z = depth_value;
                        pt_real.intensity = 170;  //*disparity32F.at<float>(v,u)/200;

                        //                        depth_value = baseline * fx_ /
                        //                        free_msg->image.at<cv::Vec4f>(v,u)[2];
                        //                        pt_free1.x = ( u - center_x ) * depth_value *
                        //                        constant_x; pt_free1.y = ( v - center_y ) *
                        //                        depth_value * constant_y; pt_free1.z =
                        //                        depth_value; pt_free1.intensity = 170;
                        //                        depth_value = baseline * fx_ /
                        //                        free_msg->image.at<cv::Vec4f>(v,u)[3];
                        //                        pt_free2.x = ( u - center_x ) * depth_value *
                        //                        constant_x; pt_free2.y = ( v - center_y ) *
                        //                        depth_value * constant_y; pt_free2.z =
                        //                        depth_value; pt_free2.intensity = 170;

                        //                        if(fg_msg->image.at<float>(v,u) >= 0.1 &&
                        //                        bg_msg->image.at<float>(v,u) >= 0.1 )
                        //                        if(new_disparity_bridgePtr->image.at<float>(v,u)
                        //                        > 0.1)
                        {
                            //                            ROS_INFO("depth:
                            //                            %f",new_disparity_bridgePtr->image.at<float>(v,u));
                            point_counter++;
                            cloud->points.push_back(pt_fg);
                            point_counter++;
                            cloud->points.push_back(pt_bg);
                            point_counter++;
                            cloud->points.push_back(pt_real);

                            //                            point_counter++;
                            //                            cloud->points.push_back ( pt_free1 );
                            //                            point_counter++;
                            //                            cloud->points.push_back ( pt_free2 );
                        }
                    }
                }
                cloud->width = point_counter;
                // ########################################################################################################

                // Timestamp structure has changed cannot convert to uint64
                cloud->header.stamp = rclcpp::Time(msg_disp->header.stamp).nanoseconds();
                cloud->header.stamp = pcl_conversions::toPCL(rclcpp::Time(msg_disp->header.stamp));
                // cloud->header.stamp = msg_disp->header.stamp;
                // cloud->header.stamp = pcl_conversions::toPCL(msg_disp->header.stamp);

                sensor_msgs::msg::PointCloud2 cloud_PC2;
                pcl::toROSMsg(*cloud, cloud_PC2);
                expansion_cloud_pub->publish(cloud_PC2);
            }
        }
    }
    return;
}

DisparityExpansionNode::DisparityExpansionNode(const rclcpp::NodeOptions& options)
    : Node("DisparityExpansionNode", options) {
    expansion_cloud_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/expansion/cloud_fg", 10);
    expansion_poly_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/expansion/occ_marker", 10);
    expanded_disparity_fg_pub = this->create_publisher<sensor_msgs::msg::Image>(
        "/narrow_stereo/left/expanded_disparity_fg", 10);
    expanded_disparity_bg_pub = this->create_publisher<sensor_msgs::msg::Image>(
        "/narrow_stereo/left/expanded_disparity_bg", 10);

    RCLCPP_INFO(this->get_logger(), "into constr with node name %s", this->get_name());
    LUT_ready = false;
    got_cam_info = false;

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/robot1/perception/camera_info", 1,
        std::bind(&DisparityExpansionNode::getCamInfo, this, std::placeholders::_1));

    // disparity_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
    //     "/narrow_stereo/disparity", 1,
    //    std::bind(&DisparityExpansionNode::stereoDisparityCb, this, std::placeholders::_1));

    disparity_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
        "/disparity/image_raw", 1,
        std::bind(&DisparityExpansionNode::stereoDisparityCb, this, std::placeholders::_1));

    this->declare_parameter("robot_radius", 2.0);
    this->declare_parameter("lut_max_disparity", 164);
    this->declare_parameter("padding", 2.0);
    this->declare_parameter("bg_multiplier", 5.0);
    this->declare_parameter("sensor_pixel_error", 0.5);

    this->get_parameter("robot_radius", robot_radius_);
    this->get_parameter("lut_max_disparity", lut_max_disparity_);
    this->get_parameter("padding", padding_);
    this->get_parameter("bg_multiplier", bg_multiplier_);
    this->get_parameter("sensor_pixel_error", pixel_error_);

    // cx_ = 317.20617294311523;
    // cy_ = 233.2914752960205;
    // fx_ = fy_ = 307.4838344732113;
    // baseline = 0.5576007548439765;
    // width = 640;
    // height = 480;
    downsample_scale = 2.0;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisparityExpansionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
