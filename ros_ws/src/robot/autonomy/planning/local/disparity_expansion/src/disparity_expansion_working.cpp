/*
* Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

//Outputs an image in the form of a point cloud in 3d space.
#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <fstream>
#include <time.h>
#include <stdio.h>

#define SCALE 2.0 // num_units/pixel
#define LUT_MAX_DISPARITY   104 // units
#define ROB_RADIUS      1.0 // in meters
//for 2times Robsize = 0.0173 at 40m
//15 = 0.0677
#define DEPTH_ERROR_COEFF 0.0177

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class DisparityExpansionNode{
public:
    DisparityExpansionNode(ros::NodeHandle& nh);

    ros::Publisher expansion_cloud_pub;
    ros::Publisher expanded_disparity_fg_pub;
    ros::Publisher expanded_disparity_bg_pub;
    ros::Subscriber subs_;
    ros::Subscriber cam_info_sub_;
    ros::Subscriber disparity_sub_;

//    message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub;
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
//    message_filters::Subscriber<sensor_msgs::Image> image_sub;
//    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;
//    typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy2;
//    message_filters::Synchronizer<MySyncPolicy>* sync;
//    message_filters::Synchronizer<MySyncPolicy2>* sync2;
    image_geometry::PinholeCameraModel model_;
    sensor_msgs::Image dimage;
    float baseline;
    bool LUT_ready;
    bool got_cam_info;

    void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr &msg_info);
    void stereoDisparityCb(const stereo_msgs::DisparityImage::ConstPtr &msg_disp);
    void generateExpansionLUT();

    struct cell
    {
      unsigned int idx1;
      unsigned int idx2;
    };
    cell table_u[LUT_MAX_DISPARITY][640];//[640];
    cell table_v[LUT_MAX_DISPARITY][480];//[512];
    float table_d[LUT_MAX_DISPARITY];
};

void DisparityExpansionNode::generateExpansionLUT()
{
    bool debug = false;
    if(debug)
    {
        ROS_WARN("Expansion LUT Debug disabled creation");
        LUT_ready = true;
    }
    if(LUT_ready)
    {
        ROS_ERROR("LUT all ready");
//        sleep(1);
        return;
    }
    float center_x = model_.cx();
    float center_y = model_.cy();
    float constant_x = 1.0 / model_.fx();
    float constant_y = 1.0 / model_.fy();
    ROS_INFO("Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
    double r = ROB_RADIUS;//expansion radius in cm
    int u1,u2,v1,v2;
    double x,y,z;
    double disparity;


    for(unsigned int disp_idx = 1;disp_idx<= LUT_MAX_DISPARITY ;disp_idx++)
    {
//        z = depth*0.01;//cm to m
        disparity = disp_idx/SCALE;//1 cell = 0.5m, z is in meters
        r = ROB_RADIUS;// * exp(DEPTH_ERROR_COEFF*z);
        z = baseline * model_.fx() / disparity;

        float disp_new = baseline * model_.fx()/(z - ROB_RADIUS) + 0.5;
        table_d[disp_idx] = disp_new ;
        ROS_INFO("REAL EXPANDED: %f , %f",disparity,table_d[disp_idx]);


        for ( int v = ( int ) dimage.height - 1; v >= 0; --v )
        {
            y = ( v - center_y ) * z * constant_y;

            double beta = atan2(z,y);
            double beta1 = asin(r/sqrt(z*z + y*y));

            double r1 = z/tan(beta+beta1);
            double r2 = z/tan(beta-beta1);
            v1 = model_.fy()*r1/z + center_y;
            v2 = model_.fy()*r2/z + center_y;

            if((v2-v1)<0)
                ROS_ERROR("Something MESSED %d %d %d",v1,v2,disp_idx);

            if(v1 < 0) v1 = 0;
            if(v1 > (dimage.height-1)) v1 = dimage.height-1;

            if(v2 < 0) v2 = 0;
            if(v2 > (dimage.height-1)) v2 = dimage.height-1;

            table_v[disp_idx][v].idx1 = v1 ;
            table_v[disp_idx][v].idx2 = v2 ;
//            ROS_ERROR("d v1 v2 %d %d \t %d",z,v1,v2);
//            ROS_ERROR("d beta beta1 %f %f \t %f",z,beta*180/M_PI, beta1*180/M_PI);
        }

        for ( int u = ( int ) dimage.width - 1; u >= 0; --u )
        {
            x = ( u - center_x ) * z * constant_x;

            double alpha = atan2(z,x);
            double alpha1 = asin(r/sqrt(z*z + x*x));

            double r1 = z/tan(alpha+alpha1);
            double r2 = z/tan(alpha-alpha1);
            u1 = model_.fx()*r1/z + center_x;
            u2 = model_.fx()*r2/z + center_x;

            if((u2-u1)<0)
                ROS_ERROR("Something MESSED %d %d %d",u1,u2,disp_idx);

            if(u1 < 0) u1 = 0;
            if(u1 > (dimage.width-1)) u1 = dimage.width-1;

            if(u2 < 0) u2 = 0;
            if(u2 > (dimage.width-1)) u2 = dimage.width-1;

            table_u[disp_idx][u].idx1 = u1 ;
            table_u[disp_idx][u].idx2 = u2 ;
//            ROS_ERROR("d u1 u2 %d %d \t %d",u1,u2,depth);

        }
    }
    ROS_WARN("Expansion LUT created");
    for(int i =1;i<=LUT_MAX_DISPARITY;i++)
    {
        disparity = i/SCALE;//1 cell = 0.5m, z is in meters
        r = ROB_RADIUS;// * exp(DEPTH_ERROR_COEFF*z);
        z = baseline * model_.fx() / disparity;

        float disp_new = baseline * model_.fx()/(z - ROB_RADIUS) + 0.5;
        table_d[i] = disp_new ;
         ROS_INFO("expanded disparity: %f %d",table_d[i],table_u[i][1].idx1);
    }
    LUT_ready = true;
//    exit(0);
}


void DisparityExpansionNode::getCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg_info)
{
    model_.fromCameraInfo ( msg_info );
    ROS_INFO_ONCE("Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
    got_cam_info = true;
}

void DisparityExpansionNode::stereoDisparityCb(const stereo_msgs::DisparityImage::ConstPtr &msg_disp){
    ROS_INFO("Disparity CB");




    if(!got_cam_info)
        return;
    dimage = msg_disp->image;
    // Update camera model    
    baseline = msg_disp->T;

    if(!LUT_ready)
        generateExpansionLUT();

    cv_bridge::CvImagePtr fg_msg;
    cv_bridge::CvImagePtr bg_msg;
    cv_bridge::CvImagePtr free_msg(new cv_bridge::CvImage());
    cv_bridge::CvImagePtr temp_msg;
    cv_bridge::CvImagePtr di_msg;

    //DEBUG READ FROM FILE
    temp_msg = cv_bridge::toCvCopy(msg_disp->image, sensor_msgs::image_encodings::TYPE_32FC1);
    temp_msg->image = temp_msg->image *0;
    temp_msg->image(cv::Rect(0 ,0, temp_msg->image.cols/2, temp_msg->image.rows/2)) = baseline * model_.fx()/10.0 ;//TL QUAD
    temp_msg->image(cv::Rect(temp_msg->image.cols/2, 0,temp_msg->image.cols/2-1, temp_msg->image.rows/2-1)) = baseline * model_.fx()/15.0 ;//BR QUAD
    temp_msg->image(cv::Rect(temp_msg->image.cols/2, temp_msg->image.rows/2,temp_msg->image.cols/2-1, temp_msg->image.rows/2-1)) = baseline * model_.fx()/55.0 ;//BR QUAD
//    temp_msg->image(cv::Rect(dimage.width/2, dimage.height/2,dimage.width-1, dimage.height-1)) = baseline * model_.fx()/15.0 ;

//    temp_msg->toImageMsg(dimage);//UNCOMMENT TO DEBUG




//    const sensor_msgs::ImageConstPtr my_ptr = &dimage;
    cv::Mat dummy(msg_disp->image.height,msg_disp->image.width,CV_32FC4);
    dummy.copyTo(free_msg->image);
//    free_msg->image.zeros(msg_disp->image.height,msg_disp->image.width,CV_32FC4);
    free_msg->header = dimage.header;
    free_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
    try
    {
        fg_msg = cv_bridge::toCvCopy(dimage, sensor_msgs::image_encodings::TYPE_32FC1);
        bg_msg = cv_bridge::toCvCopy(dimage, sensor_msgs::image_encodings::TYPE_32FC1);
        di_msg = cv_bridge::toCvCopy(dimage, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }



//    cv_bridge::CvImage cv_image;
//    try{
//    cv_image.image = cv::imread("/home/aeroscout/gd_mav/src/ompl_planner3D/launch/img/DispImage.jpeg",CV_LOAD_IMAGE_GRAYSCALE);
//    }
//    catch (cv::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
////    cv_image.encoding = "bgr8";

//    cv::Mat test;
////    test = fg_msg->image;
////    cv::imshow("OPENCV_WINDOW", cv_image.image);
////    cv::waitKey(3);
//    test = cv::imread("/home/aeroscout/gd_mav/src/ompl_planner3D/launch/img/DispImage.jpeg", 0);


    fg_msg->header   = dimage.header;
    fg_msg->encoding = dimage.encoding;;
    bg_msg->header   = dimage.header;
    bg_msg->encoding = dimage.encoding;

    bg_msg->image = bg_msg->image + INFINITY;
    fg_msg->image = fg_msg->image - INFINITY;
    if(1)//make cloud
    {
        // Use correct principal point from calibration
        float center_x = model_.cx();
        float center_y = model_.cy();

        float constant_x = 1.0 / model_.fx();
        float constant_y = 1.0 / model_.fy();

        if ( dimage.encoding == sensor_msgs::image_encodings::TYPE_32FC1 )
        {
            ROS_INFO_ONCE("IMG TYPE 32FC, GOOD");
            int row_step = dimage.step / sizeof ( float );
            const float* disparity_row = reinterpret_cast<const float*> ( &dimage.data[0] );

//            cv::Mat cv_exp_disp_foregnd_mat(dimage.height,dimage.width,CV_32F,cv::Scalar(0));
//            cv::Mat cv_exp_disp_backgnd_mat(dimage.height,dimage.width,CV_32F,cv::Scalar(9999));
            for ( int v = ( int ) dimage.height - 1; (v >= 0) ; v-=10 )
            {
                for ( int u = ( int ) dimage.width - 1; u >= 0; u-=10 )
                {
                    float disparity_value = disparity_row[v * row_step + u] + 0.5;
                    if(!isnan(double(disparity_value)) && ((int(disparity_value*SCALE)+1) < LUT_MAX_DISPARITY) && ((int(disparity_value*SCALE)+1) > 0))//expand
                    {
                        unsigned int u1 = table_u[int(disparity_value*SCALE)+1][u].idx1;
                        unsigned int u2 = table_u[int(disparity_value*SCALE)+1][u].idx2;

                        unsigned int v1 = table_v[int(disparity_value*SCALE)+1][v].idx1;
                        unsigned int v2 = table_v[int(disparity_value*SCALE)+1][v].idx2;

                        if(v1 <0 || u1 <0 || v2 >=dimage.height || u2>=dimage.width || (u2-u1)<0 || (v2-v1)<0)
                        {
                            ROS_ERROR("Expansion out of img bounds with disparity: %f",disparity_value);
                            ROS_ERROR("u1 u2 v1 v2 %d %d \t %d %d",u1,u2,v1,v2);
                            continue;
                        }
                        cv::Rect roi = cv::Rect(u1,v1,(u2-u1),(v2-v1));
                        cv::Mat submat = di_msg->image(roi).clone();
                        double min, max;
                        cv::minMaxLoc(di_msg->image(roi), &min, &max);
                        float disp_new_fg = table_d[int(max*SCALE)];
                        float disp_new_bg = table_d[int(min*SCALE)];
                        float disp_to_depth = baseline * model_.fx()/max;
                        disp_new_fg = baseline * model_.fx()/(disp_to_depth - ROB_RADIUS) /*+ 0.5*/;
//                        disp_to_depth = baseline * model_.fx()/min;
                        disp_new_bg = baseline * model_.fx()/(disp_to_depth + ROB_RADIUS) /*- 0.5*/;
                        submat.setTo(disp_new_fg);
//                        submat = cv::max(fg_msg->image(roi),submat);
                        cv::Mat tMat(fg_msg->image(roi));
                        submat.copyTo(tMat);
                        tMat = (bg_msg->image(roi));
                        submat.setTo(disp_new_bg);
                        submat.copyTo(tMat);

//                        v -= abs(v2-v1)/4;
                        u -= abs(u2-u1)/2;
//                        u -= (u-u1)/(max-min)+1;

                        /*
                        for(int i = v1;i<=v2;i++)
                            for(int j = u1; j<=u2;j++)
                            {
                                float old_fg,old_bg;
                                old_fg = fg_msg->image.at<float>(i,j);
                                old_bg = bg_msg->image.at<float>(i,j);
                                float disp_to_depth = baseline * model_.fx()/disparity_value ;
                                float disp_new_fg = baseline * model_.fx()/(disp_to_depth - ROB_RADIUS) + 0.5;
                                if((disp_new_fg) > fg_msg->image.at<float>(i,j))
                                {
                                    fg_msg->image.at<float>(i,j) =  disp_new_fg;
                                    free_msg->image.at<cv::Vec4f>(i,j)[0] = disp_new_fg;
                                }

                                float disp_new_bg = baseline * model_.fx()/(disp_to_depth + ROB_RADIUS) - 0.5;
                                if((disp_new_bg) < bg_msg->image.at<float>(i,j))
                                {
                                    bg_msg->image.at<float>(i,j) =  disp_new_bg;
                                    free_msg->image.at<cv::Vec4f>(i,j)[1] = disp_new_bg;
                                }

                                if(disp_new_fg < old_bg && disparity_value != 0 && old_bg < 500)
                                {
                                    if(free_msg->image.at<cv::Vec4f>(i,j)[2] < old_bg)
                                        free_msg->image.at<cv::Vec4f>(i,j)[2] = old_bg-1;
                                    if(free_msg->image.at<cv::Vec4f>(i,j)[3] < disp_new_fg)
                                        free_msg->image.at<cv::Vec4f>(i,j)[3] = disp_new_fg+1;
                                }

                                if(disp_new_bg > old_fg && disparity_value != 0 && old_bg < 500)
                                {
                                    if(free_msg->image.at<cv::Vec4f>(i,j)[2] < disp_new_bg)
                                        free_msg->image.at<cv::Vec4f>(i,j)[2] = disp_new_bg-1;
                                    if(free_msg->image.at<cv::Vec4f>(i,j)[3] < old_fg)
                                        free_msg->image.at<cv::Vec4f>(i,j)[3] = old_fg+1;
                                }
                            }
                            */
                    }
                    else
                        ROS_ERROR_THROTTLE(1,"BAD disparity during expansion: %f",disparity_value);
                }
            }

//            double min, max;
//            cv::minMaxLoc(fg_msg->image, &min, &max);
//            ROS_INFO("FG minmax: \t%f \t%f",min,max);
//            cv::minMaxLoc(bg_msg->image, &min, &max);
//            ROS_INFO("BG minmax: \t%f \t%f",min,max);

            expanded_disparity_fg_pub.publish(fg_msg->toImageMsg());
            expanded_disparity_bg_pub.publish(bg_msg->toImageMsg());

            if(expansion_cloud_pub.getNumSubscribers() > 0 )//create expansion PCD
            {
                PointCloud::Ptr cloud (new PointCloud);
                cloud->header.frame_id = msg_disp->header.frame_id;
                cloud->height = 1;
                cloud->width =1;
                cloud->is_dense = false;
                int point_counter=0;
                pcl::PointXYZI pt_fg,pt_bg,pt_free1,pt_free2;
                for ( int v = ( int ) dimage.height - 1; v >= 0; v-=4 )
                {
                    for ( int u = ( int ) dimage.width - 1; u >= 0; u-=4 )
                    {

                        // Fill in XYZ
                        float depth_value = msg_disp->T * model_.fx() / fg_msg->image.at<float>(v,u);//free_msg->image.at<cv::Vec4f>(v,u)[0];
                        pt_fg.x = ( u - center_x ) * depth_value * constant_x;
                        pt_fg.y = ( v - center_y ) * depth_value * constant_y;
                        pt_fg.z = depth_value;
                        pt_fg.intensity = 220;

                        depth_value = msg_disp->T * model_.fx() / bg_msg->image.at<float>(v,u);//free_msg->image.at<cv::Vec4f>(v,u)[1];
                        pt_bg.x = ( u - center_x ) * depth_value * constant_x;
                        pt_bg.y = ( v - center_y ) * depth_value * constant_y;
                        pt_bg.z = depth_value;
                        pt_bg.intensity = 120;

                        depth_value = msg_disp->T * model_.fx() / free_msg->image.at<cv::Vec4f>(v,u)[2];
                        pt_free1.x = ( u - center_x ) * depth_value * constant_x;
                        pt_free1.y = ( v - center_y ) * depth_value * constant_y;
                        pt_free1.z = depth_value;
                        pt_free1.intensity = 170;
                        depth_value = msg_disp->T * model_.fx() / free_msg->image.at<cv::Vec4f>(v,u)[3];
                        pt_free2.x = ( u - center_x ) * depth_value * constant_x;
                        pt_free2.y = ( v - center_y ) * depth_value * constant_y;
                        pt_free2.z = depth_value;
                        pt_free2.intensity = 170;

                        if(fg_msg->image.at<float>(v,u) >= 0.1 && bg_msg->image.at<float>(v,u) >= 0.1 )
                        {
                            point_counter++;
                            cloud->points.push_back ( pt_fg );
                            point_counter++;
                            cloud->points.push_back ( pt_bg );

//                            point_counter++;
//                            cloud->points.push_back ( pt_free1 );
//                            point_counter++;
//                            cloud->points.push_back ( pt_free2 );

                        }
                    }
                }
                cloud->width  = point_counter;
                cloud->header.seq = msg_disp->header.seq;
                cloud->header.stamp = pcl_conversions::toPCL(msg_disp->header.stamp);

                sensor_msgs::PointCloud2 cloud_PC2;
                pcl::toROSMsg(*cloud,cloud_PC2);
                expansion_cloud_pub.publish(cloud_PC2);
            }
        }
    }
    return;
}

DisparityExpansionNode::DisparityExpansionNode(ros::NodeHandle& nh){
    //the depth image stream and color image stream have different framerates and therefore are not aligned. So we use a time synchronizer message filter (http://www.ros.org/wiki/message_filters)
//    depth_sub.subscribe(nh,"/camera/depth/image", 1);
//    image_sub.subscribe(nh, "/camera/rgb/image_rect_color", 1);
//    info_sub.subscribe(nh, "/camera/depth/camera_info", 1);

//    disp_sub.subscribe(nh, "/ueye_resized/disparity", 1);
//    image_sub.subscribe(nh, "/ueye_resized/left/image_rect_color", 1);
//    info_sub.subscribe(nh, "/ueye_resized/left/camera_info", 1);

//    disp_sub.subscribe(nh, "/ceye/disparity", 1);
//    image_sub.subscribe(nh, "/ceye/left/image_rect_color", 1);
//    info_sub.subscribe(nh, "/ceye/left/camera_info", 1);

//    sync = new message_filters::Synchronizer<MySyncPolicy>(2);
//    sync->connectInput(depth_sub,image_sub, info_sub);
//    sync->registerCallback(boost::bind(&depth_proc::callback,this, _1, _2,_3));

//    sync2 = new message_filters::Synchronizer<MySyncPolicy2>(2);
//    sync2->connectInput(disp_sub,image_sub, info_sub);
//    sync2->registerCallback(boost::bind(&DisparityExpansionNode::stereoDisparityCb,this, _1, _2,_3));
    expansion_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/expansion/cloud_fg", 10);
    expanded_disparity_fg_pub = nh.advertise<sensor_msgs::Image>("/ceye/left/expanded_disparity_fg", 10);
    expanded_disparity_bg_pub = nh.advertise<sensor_msgs::Image>("/ceye/left/expanded_disparity_bg", 10);
    ROS_INFO("into constr");
    LUT_ready = false;
    got_cam_info = false;


//    cam_info_sub_ = nh.subscribe("/ceye/left/camera_info", 1,&DisparityExpansionNode::getCamInfo,this);
//    disparity_sub_ = nh.subscribe("/ceye/disparity", 1,&DisparityExpansionNode::stereoDisparityCb,this);
    cam_info_sub_ = nh.subscribe("/camera/depth/camera_info", 1,&DisparityExpansionNode::getCamInfo,this);
    disparity_sub_ = nh.subscribe("/camera/disparity/image_raw", 1,&DisparityExpansionNode::stereoDisparityCb,this);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DisparityExpansionNode");
 //	cv::initModule_nonfree();//THIS LINE IS IMPORTANT for using surf and sift features of opencv
    ros::NodeHandle nh;
    DisparityExpansionNode d(nh);
    ros::spin();
    return 0;
}

