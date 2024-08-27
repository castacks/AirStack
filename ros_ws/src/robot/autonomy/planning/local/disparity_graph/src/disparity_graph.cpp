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
 * Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

#include "disparity_graph/disparity_graph.hpp"

#include <cmath>

using namespace nabla::disparity_graph;

disparity_graph::disparity_graph(const rclcpp::NodeOptions &options)
    : Node("disparity_graph", options) {
    // ros::NodeHandle priv_nh("~/disparity_graph");
    // ros::NodeHandle global_nh("/oa");
    // graph_size = 10;
    // angle_tol = /*cos*/(30.0);
    // displacement_tol = 01.5;
    this->declare_parameter("graph_size", 10);
    this->declare_parameter("angle_tolerance", 30.0);
    this->declare_parameter("displacement_tolerance", 1.5);

    graph_size = this->get_parameter("graph_size").as_int();
    angle_tol = this->get_parameter("angle_tolerance").as_double() * M_PI / 180.0;
    displacement_tol = this->get_parameter("displacement_tolerance").as_double();

    first = true;
    got_cam_info = false;
    fixed_frame = "world";
    stabilized_frame = "base_frame_stabilized";

    // disparity_graph_marker_pub =
    // priv_nh.advertise<visualization_msgs::MarkerArray>("disparity_marker", 10);
    disparity_graph_marker_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("disparity_marker", 10);

    marker.header.frame_id = fixed_frame;
    marker.header.stamp = this->now();
    marker.ns = "disparityGraph";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // downsample_scale =4.0;
    // baseline_ = 0.5576007548439765;

    // global_nh.param("baseline", baseline_, 0.10);//0.5576007548439765;
    // global_nh.param("downsample_scale", downsample_scale, 1.0);
    // global_nh.param("low_occ_thresh", thresh_, 0.9);
    this->declare_parameter("baseline", 0.10);
    this->declare_parameter("downsample_scale", 1.0);
    this->declare_parameter("low_occ_thresh", 0.9);

    // int igraph_size = graph_size;
    // priv_nh.param("graph_size", igraph_size, igraph_size);
    // graph_size = igraph_size;
    // priv_nh.param("displacement_tolerance", displacement_tol, displacement_tol);
    // priv_nh.param("angle_tolerance", angle_tol, angle_tol);

    // angle_tol = angle_tol * M_PI/180.0;

    // graph_size = this->get_parameter("graph_size").as_int();
    // angle_tol = this->get_parameter("angle_tolerance").as_double() * M_PI / 180.0;
    // displacement_tol = this->get_parameter("displacement_tolerance").as_double();
    baseline_ = this->get_parameter("baseline").as_double();
    downsample_scale = this->get_parameter("downsample_scale").as_double();
    thresh_ = this->get_parameter("low_occ_thresh").as_double();

    RCLCPP_ERROR(this->get_logger(), "disparity_graph NS: %s", this->get_namespace());

    //    right_info_sub_.subscribe(nh, right_info_topic, 1);
    disp_fg_sub_.subscribe(this, "/ceye/left/expanded_disparity_fg",
                           rclcpp::QoS(5).get_rmw_qos_profile());
    disp_bg_sub_.subscribe(this, "/ceye/left/expanded_disparity_bg",
                           rclcpp::QoS(5).get_rmw_qos_profile());

    exact_sync_ = std::make_shared<ExactSync>(ExactPolicy(5), disp_fg_sub_, disp_bg_sub_);
    exact_sync_->registerCallback(
        std::bind(&disparity_graph::disp_cb, this, std::placeholders::_1, std::placeholders::_2));

    // cam_info_sub_ = priv_nh.subscribe("/nerian_sp1/right/camera_info",
    // 1,&disparity_graph::getCamInfo,this);
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/nerian_sp1/right/camera_info", 10,
        std::bind(&disparity_graph::getCamInfo, this, std::placeholders::_1));
    // timer1 = priv_nh.createTimer(ros::Duration(0.20), &disparity_graph::pcd_test4,this);
    timer1 = this->create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&disparity_graph::pcd_test4, this));

    pcd_checked_states.header.frame_id = fixed_frame;
    pcd_checked_states.height = 1;
    pcd_checked_states.width = 0;
    pcd_checked_states.is_dense = false;
    pcdPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/graph_pcd", 10);
    expansion_poly_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/expansion/graph_occ_marker", 10);

}

void disparity_graph::pcd_test() {
    if (pcdPub->get_subscription_count() == 0) {
        return;
    }

    for (uint i = 0; i < disp_graph.size(); i++) {
        for (uint u = 0; u < width_; u += 1) {
            for (uint v = 0; v < height_; v += 1) {
                tf2::Vector3 pt_optical_fg, pt_optical_bg, pt_world_fg, pt_world_bg, curr_pt;
                tf2::Transform w2s_tf;
                tf2::Transform s2w_tf;
                double z_fg, z_bg;
                cv_bridge::CvImagePtr cv_depth_fg, cv_depth_bg;
                {
                    std::lock_guard<std::mutex> lock(io_mutex);
                    // cv_depth_fg =
                    // cv_bridge::toCvCopy(disp_graph.at(i).Im_fg,sensor_msgs::image_encodings::TYPE_32FC1);
                    // cv_depth_bg =
                    // cv_bridge::toCvCopy(disp_graph.at(i).Im_bg,sensor_msgs::image_encodings::TYPE_32FC1);
                    w2s_tf = disp_graph.at(i).w2s_tf;
                    s2w_tf = disp_graph.at(i).s2w_tf;

                    z_fg = (baseline_ * fx_ / disp_graph.at(i).Im_fg->image.at<float>(v, u));
                    z_bg = (baseline_ * fx_ / disp_graph.at(i).Im_bg->image.at<float>(v, u));
                }

                for (double z_sample = z_fg;
                     z_sample <= z_bg && std::isfinite(z_sample) /*&& z_sample>5.0*/;
                     z_sample += 0.5) {
                    pt_optical_fg.setZ(z_sample);
                    pt_optical_fg.setX((u - cx_) * pt_optical_fg.getZ() / fx_);
                    pt_optical_fg.setY((v - cy_) * pt_optical_fg.getZ() / fy_);

                    pt_world_fg = s2w_tf * pt_optical_fg;

                    pcl::PointXYZI pt;
                    // FG
                    pt.x = pt_world_fg.getX();
                    pt.y = pt_world_fg.getY();
                    pt.z = pt_world_fg.getZ();

                    double state_disparity = baseline_ * fx_ / z_sample;
                    double confidence = (state_disparity - 0.5) / state_disparity;

                    if (confidence >= thresh_)
                        pt.intensity = confidence;  //*200.0;//i*20+10;
                    else
                        pt.intensity = 0.0;
                    if (confidence >= thresh_) {
                        pcd_checked_states.points.push_back(pt);
                        pcd_checked_states.width++;
                    }
                    z_sample = ((z_sample + 0.5) > z_bg) ? z_bg : z_sample;
                }

            }
        }
    }

    // Create the filtering object
    pcd_checked_states.header.stamp = pcl_conversions::toPCL(this->now());
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(pcd_checked_states, *cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    double voxel_dim = 0.5;
    sor.setLeafSize(voxel_dim, voxel_dim, voxel_dim);
    sor.filter(*cloud_filtered);
    pcd_checked_states.clear();
    pcl::fromPCLPointCloud2(*cloud_filtered, pcd_checked_states);

    sensor_msgs::msg::PointCloud2 cloud_PC2;
    pcl::toROSMsg(pcd_checked_states, cloud_PC2);
    pcdPub->publish(cloud_PC2);
    pcd_checked_states.points.clear();
    pcd_checked_states.width = 0;
}

void disparity_graph::pcd_test4() {
    if (pcdPub->get_subscription_count() == 0) {
        return;
    }

    pcl::PointXYZI pt;

    rclcpp::Time stamp = this->now();
    tf_buffer_->canTransform(fixed_frame, stabilized_frame, stamp,
                             rclcpp::Duration::from_seconds(0.1));
    geometry_msgs::msg::TransformStamped transform;
    try {
        // listener.lookupTransform(fixed_frame, stabilized_frame, stamp, transform);
        transform = tf_buffer_->lookupTransform(fixed_frame, stabilized_frame, stamp);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "DG graph_pcd %s", ex.what());
        //         ros::Duration(1.0).sleep();
        //         continue;
        return;
    }
    //  checked_point_stmpd.header.stamp = stamp;//ros::Time::now();//(0);
    //  checked_point_stmpd.header.frame_id = fixed_frame;

    pcd_checked_states.header.frame_id = fixed_frame;

    geometry_msgs::msg::PoseStamped checked_pose;
    checked_pose.header.frame_id = pcd_checked_states.header.frame_id;
    checked_pose.header.stamp = stamp;
    float delta = .5;
    for (float i = -50; i <= 50; i += delta) {
        for (float u = -50; u <= 50; u += delta) {
            for (float v = -10; v <= 10; v += delta) {
                //          checked_point_stmpd.point.x = i;
                //          checked_point_stmpd.point.y = u;
                //          checked_point_stmpd.point.z = v;

                tf2::Vector3 local_point(i, u, v);
                //          tf::pointMsgToTF(checked_point_stmpd.point,local_point);
                // tf2::Vector3 world_point = tf2::transformToEigen(transform);
                // tf2::toMsg(world_point, checked_pose.pose.position);
                Eigen::Isometry3d transform_eigen;
                tf2::fromMsg(transform.transform, transform_eigen);
                Eigen::Vector3d eigen_world_point =
                    transform_eigen *
                    Eigen::Vector3d(local_point.x(), local_point.y(), local_point.z());
                tf2::Vector3 world_point(eigen_world_point.x(), eigen_world_point.y(),
                                         eigen_world_point.z());
                tf2::toMsg(world_point, checked_pose.pose.position);
                double occupancy = 0.0;
                bool valid = isStateValidDepth_pose(checked_pose, thresh_, occupancy);
                if (!valid) {
                    pt.x = world_point.getX();
                    pt.y = world_point.getY();
                    pt.z = world_point.getZ();
                    pt.intensity = occupancy;

                    if (occupancy >= thresh_) {
                        pcd_checked_states.points.push_back(pt);
                        pcd_checked_states.width++;
                    }
                }
            }
        }
    }

    // Create the filtering object
    pcd_checked_states.header.stamp = pcl_conversions::toPCL(stamp);
    /*pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(pcd_checked_states,*cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    double voxel_dim = 0.5;
    sor.setLeafSize (voxel_dim,voxel_dim,voxel_dim);
    sor.filter (*cloud_filtered);
    pcd_checked_states.clear();
    pcl::fromPCLPointCloud2(*cloud_filtered,pcd_checked_states);*/

    sensor_msgs::msg::PointCloud2 cloud_PC2;
    pcl::toROSMsg(pcd_checked_states, cloud_PC2);
    pcdPub->publish(cloud_PC2);
    pcd_checked_states.points.clear();
    pcd_checked_states.width = 0;
}

/*
void disparity_graph::pcd_test2(const ros::TimerEvent& event) {
  pcd_test(event);
  if(occPub_.getNumSubscribers() == 0)
    return;

  occ_map.info.map_load_time = ros::Time::now();
  occ_map.data.clear();
  occ_map.data.resize(occ_map.info.width*occ_map.info.height,-1);

  for(uint u=0; u<occ_map.info.width; u++) {
    for(uint v=0; v<occ_map.info.height; v++) {
      double x = (u + occ_map.info.origin.position.x);
      double y = (v + occ_map.info.origin.position.y);
      geometry_msgs::Point p;
      p.x = x;p.y = y;p.z = orig_z;
      geometry_msgs::Pose chk_pose;
      chk_pose.position = p;
      chk_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      uint64 index = occ_map.info.width*v/occ_map.info.resolution + u/occ_map.info.resolution;
      double cost =-1.0;
      if(index>=0 && index < occ_map.info.width*occ_map.info.height)
      {
        getStateCost(chk_pose,cost);
        occ_map.data[index] = uint64(cost);
      }

    }
  }
  occ_map.header.stamp = ros::Time::now();
  occPub_.publish(occ_map);
}
*/

void disparity_graph::pcd_test3() {
    /* create expansion PCD */
    if (expansion_poly_pub->get_subscription_count() > 0) {
        cv_bridge::CvImagePtr cv_depth_fg, cv_depth_bg;
        visualization_msgs::msg::MarkerArray marker_arr;
        visualization_msgs::msg::Marker marker;
        // marker.header = msg_disp->header;
        marker.ns = "occ_space";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;  // xyz_centroid[0];
        marker.pose.position.y = 0;  // xyz_centroid[1];
        marker.pose.position.z = 0;  // xyz_centroid[2];

        // tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0.0,0.0,0.0), marker.pose.orientation);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        tf2::convert(q, marker.pose.orientation);

        //                marker.pose.orientation.w = 1;
        float marker_scale = 0.1;
        marker.scale.x = marker_scale;
        marker.scale.y = marker_scale;
        marker.scale.z = marker_scale;
        marker.color.a = 1.0;  // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        int v = 60;
        for (uint i = 0; i < disp_graph.size(); i++) {
            float prev_depth = 0.0;
            marker.header = disp_graph.at(i).header;
            for (int u = (int)width_ - 1; u >= 0; u--) {
                float depth_value = baseline_ * fx_ /
                                    disp_graph.at(i).Im_fg->image.at<float>(
                                        v, u);  // free_msg->image.at<cv::Vec4f>(v,u)[0];
                float depth_diff = fabs(depth_value - prev_depth);
                prev_depth = depth_value;
                if (!std::isnan(depth_value) && !std::isinf(depth_value) && depth_diff < 0.5) {
                    geometry_msgs::msg::Point gm_p;
                    gm_p.x = (u - cx_) * depth_value / fx_;
                    gm_p.y = (v - cy_) * depth_value / fy_;
                    gm_p.z = depth_value;
                    marker.points.push_back(gm_p);

                    depth_value = baseline_ * fx_ / disp_graph.at(i).Im_bg->image.at<float>(v, u);
                    gm_p.x = (u - cx_) * depth_value / fx_;
                    gm_p.y = (v - cy_) * depth_value / fy_;
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
}

void disparity_graph::disp_cb(const sensor_msgs::msg::Image::ConstSharedPtr &disp_fg,
                              const sensor_msgs::msg::Image::ConstSharedPtr &disp_bg) {
    //    ROS_INFO("Recvd Disp,size %d",disp_graph.size());
    RCLCPP_INFO(this->get_logger(), "Recvd disp stamp: %lf",
                (this->now() - disp_fg->header.stamp).seconds());
    sensor_frame = disp_fg->header.frame_id;

    //  listener.waitForTransform(sensor_frame,
    //                            fixed_frame,
    //                            disp_fg->header.stamp, ros::Duration(0.10));
    //  tf::StampedTransform transform;

    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
        // listener.lookupTransform(sensor_frame,
        //                          fixed_frame,
        //                          disp_fg->header.stamp, transform);
        transform_stamped = tf_buffer_->lookupTransform(fixed_frame, sensor_frame,
                                                        tf2_ros::fromMsg(disp_fg->header.stamp),
                                                        tf2::durationFromSec(0.10));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "DG disp_cb %s", ex.what());
        //         ros::Duration(1.0).sleep();
        //         continue;
        return;
    }

    std::lock_guard<std::mutex> lock(io_mutex);
    if (first) {
        first = false;
        node n;
        try {
            n.Im_fg = cv_bridge::toCvCopy(disp_fg, sensor_msgs::image_encodings::TYPE_32FC1);
            n.Im_bg = cv_bridge::toCvCopy(disp_bg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "\n\n\n\n\n\t\t\tDEQUE ERR\n\n\n\n: %s", ex.what());
            return;
        }
        //        n.Im_fg     = *disp_fg;
        //        n.Im_bg     = *disp_bg;
        n.header = disp_bg->header;
        // n.w2s_tf = transform;
        tf2::fromMsg(transform_stamped.transform, n.w2s_tf);
        // n.w2s_tf= tf2::transformToEigen(transform_stamped.transform);
        n.s2w_tf = n.w2s_tf.inverse();
        disp_graph.push_front(n);
        disp_graph.push_back(n);
    } else {
        // Check for similar tf nodes
        // tf::Vector3 curr_a =transform.inverse().getRotation().normalize().getAxis().normalize();

        // tf::Vector3 curr_p =transform.inverse().getOrigin();
        // tf2::Transform transform;
        // tf2::Vector3 curr_p = transform_stamped.inverse().getOrigin();
        //  ###############################################################################

        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);

        // Now get the inverse of the tf2::Transform
        tf2::Transform inverse_transform = transform.inverse();

        // Convert the inverse tf2::Transform back to geometry_msgs::msg::Transform
        geometry_msgs::msg::Transform inverse_transform_msg = tf2::toMsg(inverse_transform);

        // Now, convert the geometry_msgs::msg::Transform to Eigen::Isometry3d
        Eigen::Isometry3d eigen_transform = tf2::transformToEigen(inverse_transform_msg);

        // Extract the translation as Eigen::Vector3d
        Eigen::Vector3d curr_p = eigen_transform.translation();

        // ##########################################################################################################
        std::deque<node>::iterator it, end;
        it = disp_graph.begin();
        end = disp_graph.end() - 1;

        // #############################################################################################################
        //     while(it!=end && disp_graph.size())
        //     {
        //  tf::Vector3 graph_a = it->s2w_tf.getRotation().normalize().getAxis().normalize();
        // tf2::Vector3 graph_p = it->s2w_tf.getOrigin();
        // Ensure it->s2w_tf is of type geometry_msgs::msg::Transform
        geometry_msgs::msg::Transform s2w_tf_msg =
            tf2::toMsg(it->s2w_tf);  // Assuming it->s2w_tf is a tf2::Transform

        // Convert to Eigen
        Eigen::Isometry3d graph_transform = tf2::transformToEigen(s2w_tf_msg);
        Eigen::Vector3d graph_p = graph_transform.translation();  // Get the translation

        tf2Scalar diff_a =
            tf2::angleShortestPath(transform.inverse().getRotation(), it->s2w_tf.getRotation());
        // tf2Scalar diff_p = curr_p.distance(graph_p);
        tf2Scalar diff_p = (curr_p - graph_p).norm();
        //        ROS_INFO_STREAM("diff_a ; diff_p:"<<diff_a<<"\t"<<diff_p);
        // ############################################################################################################
        node n;
        try {
            n.Im_fg = cv_bridge::toCvCopy(disp_fg, sensor_msgs::image_encodings::TYPE_32FC1);
            n.Im_bg = cv_bridge::toCvCopy(disp_bg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "\n\n\n\n\n\t\t\tDEQUE ERR\n\n\n\n: %s", ex.what());
            return;
        }
        //        n.Im_fg     = *disp_fg;
        //        n.Im_bg     = *disp_bg;
        n.header = disp_bg->header;
        n.w2s_tf = transform;
        n.s2w_tf = transform.inverse();
        // pose in graph is too close in angle to the current graph
        if (fabs(diff_a) >= angle_tol || fabs(diff_p) > displacement_tol) {
            RCLCPP_ERROR(this->get_logger(), "Adding new node size %d", (int)disp_graph.size());
            if (disp_graph.size() >= graph_size) {
                disp_graph.erase(end);
            }

            // add new data
            if (disp_graph.size() <= graph_size) {
                disp_graph.push_front(n);
            }
            //            it = disp_graph.erase(it);
        }
        disp_graph.pop_back();
        disp_graph.push_back(n);
        //        pcd_test();
    }

    visualization_msgs::msg::MarkerArray marker_arr;

    for (uint i = 0; i < disp_graph.size(); i++) {
        //        ROS_INFO_STREAM(disp_graph.at(i).tf.inverse().getOrigin().x());
        // tf2::pointTFToMsg(disp_graph.at(i).s2w_tf.getOrigin(), marker.pose.position);
        // tf2::quaternionTFToMsg(disp_graph.at(i).s2w_tf.getRotation(), marker.pose.orientation);
        // marker.pose.position = tf2::toMsg(disp_graph.at(i).s2w_tf.getOrigin());
        tf2::Vector3 origin = disp_graph.at(i).s2w_tf.getOrigin();
        marker.pose.position.x = origin.getX();
        marker.pose.position.y = origin.getY();
        marker.pose.position.z = origin.getZ();
        marker.pose.orientation = tf2::toMsg(disp_graph.at(i).s2w_tf.getRotation());
        marker.header.stamp = this->now();
        marker.id = i;
        marker_arr.markers.push_back(marker);
    }

    disparity_graph_marker_pub->publish(marker_arr);
    // marker_arr.markers.clear();
}

void disparity_graph::getCamInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg_info) {
    if (got_cam_info) {
        return;
    }
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(msg_info);
    RCLCPP_WARN(this->get_logger(), "Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f Baseline: %f",
                model_.fx(), model_.fy(), model_.cx(), model_.cy(), baseline_);
    cx_ = model_.cx() / downsample_scale;
    cy_ = model_.cy() / downsample_scale;
    fx_ = fy_ = model_.fx() / downsample_scale;
    width_ =
        msg_info->width / downsample_scale;  // this is already downsampled from, disp_expansion
    height_ = msg_info->height / downsample_scale;
    double baseline = -msg_info->p[3] / msg_info->p[0];
    if (baseline != 0.0) {
        baseline_ = baseline;
    }
    baseline_ *= downsample_scale;
    RCLCPP_WARN(this->get_logger(),
                "Transformed Cam Info Recvd Fx Fy Cx Cy: %f %f, %f %f Baseline: %f with "
                "downsamplescale: %f",
                model_.fx(), model_.fy(), model_.cx(), model_.cy(), baseline_, downsample_scale);

    RCLCPP_WARN(this->get_logger(),
                "Cam Info Constants - Fx Fy Cx Cy: %f %f, %f %f / width=%d - height=%d", fx_, fy_,
                cx_, cy_, width_, height_);

    got_cam_info = true;
}

bool disparity_graph::isStateValidDepth_pose(geometry_msgs::msg::PoseStamped checked_state,
                                             double thresh, double &occupancy) {
    // unsigned int invalid_cntr=0;
    occupancy = 0.0;
    geometry_msgs::msg::PointStamped checked_point_stamped;
    checked_point_stamped.point = checked_state.pose.position;
    checked_point_stamped.header = checked_state.header;

    ////check the pose using the footprint_cost check

    geometry_msgs::msg::PointStamped world_point_stamped;

    try {
        // listener_.transformPoint(fixed_frame, checked_point_stamped, world_point_stamped);
        world_point_stamped = tf_buffer_->transform(checked_point_stamped, fixed_frame);
    } catch (tf2::TransformException ex) {
        RCLCPP_ERROR(this->get_logger(), "TF to fixed_frame failed: %s", ex.what());
        return false;
    }

    tf2::Vector3 optical_point;
    tf2::fromMsg(world_point_stamped.point, optical_point);

    std::lock_guard<std::mutex> lock(io_mutex);
    bool seen = false;  // true;
    for (uint i = 0; i < disp_graph.size(); i++) {
        tf2::Vector3 local_point = disp_graph.at(i).w2s_tf * optical_point;

        float x, y, z;

        x = local_point.getX();
        y = local_point.getY();
        z = local_point.getZ();

        if (local_point.length() < 1.0) {
            seen = true;
            continue;
        }
        //    if((-0.20<z)&&(z<=0.50))
        //      z = 0.50;

        int u = x / z * fx_ + cx_;
        int v = y / z * fy_ + cy_;

        if (u >= 0 && u < width_ && v >= 0 && v < height_ && z > 0.0) {
            seen = true;
            double state_disparity = baseline_ * fx_ / z;
            if ((disp_graph.at(i).Im_fg->image.at<float>(v, u) > state_disparity) &&
                (disp_graph.at(i).Im_bg->image.at<float>(v, u) < state_disparity)) {
                occupancy += (state_disparity - 0.5) / state_disparity;
            } else {
                occupancy -= 0.5 * (state_disparity - 0.5) / state_disparity;
                occupancy = occupancy < 0.0 ? 0.0 : occupancy;
            }
        }
        if (/*invalid_cntr>=invalid_thresh*/ occupancy >= thresh) {
            return false;
        }
    }
    if (seen) {
        return true;
    } else {
        return false;
    }
}

bool disparity_graph::getStateCost(geometry_msgs::msg::Pose checked_state, double &cost) {
    //    uint invalid_thresh = 2;
    //    uint validity = 2;
    //  unsigned int invalid_cntr=0;
    geometry_msgs::msg::PointStamped checked_point_stamped;
    checked_point_stamped.header.frame_id = "world";
    checked_point_stamped.header.stamp = this->now() - rclcpp::Duration::from_seconds(0.1);
    checked_point_stamped.point = checked_state.position;

    geometry_msgs::msg::PointStamped world_point_stamped;
    //    listener.waitForTransform(fixed_frame,checked_point_stamped.header.frame_id,checked_point_stamped.header.stamp,ros::Duration(1.0));
    try {
        // listener_.transformPoint(fixed_frame, checked_point_stamped, world_point_stamped);
        world_point_stamped = tf_buffer_->transform(checked_point_stamped, fixed_frame);
    } catch (tf2::TransformException ex) {
        RCLCPP_ERROR(this->get_logger(), "TF to fixed_frame failed: %s", ex.what());
        return false;
    }

    tf2::Vector3 optical_point;
    tf2::fromMsg(world_point_stamped.point, optical_point);

    std::lock_guard<std::mutex> lock(io_mutex);
    for (uint i = 0; i < disp_graph.size(); i++) {
        tf2::Vector3 local_point = disp_graph.at(i).w2s_tf * optical_point;

        float x, y, z;
        x = local_point.getX();
        y = local_point.getY();
        z = local_point.getZ();

        // point behind camera
        if (z < 0) {
            continue;
        }

        int u = x / z * fx_ + cx_;
        int v = y / z * fy_ + cy_;

        if (u >= 0 && u < width_ && v >= 0 && v < height_) {
            double state_disparity = baseline_ * fx_ / z;
            if ((disp_graph.at(i).Im_fg->image.at<float>(v, u) > state_disparity) &&
                (disp_graph.at(i).Im_bg->image.at<float>(v, u) < state_disparity)) {
                //                cost += (fabs(bg_msg->image.at<float>(v,u) -
                //                state_disparity)/fabs(fg_msg->image.at<float>(v,u) -
                //                bg_msg->image.at<float>(v,u)) + 1) * 10;
                cost += state_disparity * 10.0;
                //        invalid_cntr++;
            }
            cost = cost < 0.0 ? 0.0 : cost;

            if (cost > 100.0) {
                cost = 100.0;
                return false;
            }

            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,  // Throttle for 1 second
                "CHECKING POINT at \t%f\t%f\t%f : cost = %f", checked_state.position.x,
                checked_state.position.y, checked_state.position.z, cost);
        }
    }

    return true;
}
