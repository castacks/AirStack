#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace disparity_graph {

class DisparityGraph {
    struct DisparityGraphNode {
        cv_bridge::CvImagePtr Im_fg;
        cv_bridge::CvImagePtr Im_bg;
        std_msgs::msg::Header header;
        tf2::Transform w2s_tf;
        tf2::Transform s2w_tf;
    };
    std::deque<DisparityGraphNode> disp_graph_;

    rclcpp::Node::SharedPtr node_ptr;

    size_t graph_size_;
    double thresh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
    std::string sensor_frame, fixed_frame, stabilized_frame;
    visualization_msgs::msg::Marker marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr disparity_graph_marker_pub_;
    double angle_tol, displacement_tol, assume_seen_within_radius;
    bool first;
    bool got_cam_info;
    double fx_, fy_, cx_, cy_, baseline_, downsample_scale;
    unsigned int width_, height_;

    std::mutex io_mutex;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> disp_fg_sub_, disp_bg_sub_;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>
        ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

    std::shared_ptr<ExactSync> exact_sync;

   public:
    DisparityGraph()
        : graph_size_(10),
          angle_tol(30.0),
          displacement_tol(1.5),
          first(true),
          got_cam_info(false) {
    }

    // we need this separate initialization function because we're taking a node_ptr. shared_from_this() only works after constructor exits.
    // ideally we switch to use nodelets so there's a proper setup/activate phase
    void initialize(const rclcpp::Node::SharedPtr &node_ptr,
                    const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) {
        RCLCPP_INFO(node_ptr->get_logger(), "DisparityGraph initialize called");
        this->node_ptr = node_ptr;
        this->tf_buffer_ptr = tf_buffer_ptr;
        disparity_graph_marker_pub_ =
            node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>("disparity_graph", 10);

        node_ptr->declare_parameter("baseline_fallback", 0.5);

        node_ptr->declare_parameter("fixed_frame", "map");
        node_ptr->get_parameter("fixed_frame", this->fixed_frame);
        node_ptr->declare_parameter("stabilized_frame", "base_link_stabilized");
        node_ptr->get_parameter("stabilized_frame", this->stabilized_frame);
        node_ptr->declare_parameter("assume_seen_within_radius", 1.0);
        node_ptr->get_parameter("assume_seen_within_radius", assume_seen_within_radius);
        node_ptr->declare_parameter("downsample_scale", 1.0);
        node_ptr->get_parameter("downsample_scale", downsample_scale);
        node_ptr->declare_parameter("low_occ_thresh", 0.9);
        node_ptr->get_parameter("low_occ_thresh", thresh_);
        node_ptr->declare_parameter("graph_size", 10);
        node_ptr->get_parameter("graph_size", graph_size_);
        node_ptr->declare_parameter("displacement_tolerance", 1.5);
        node_ptr->get_parameter("displacement_tolerance", displacement_tol);
        node_ptr->declare_parameter("angle_tolerance", 30.0);
        angle_tol = node_ptr->get_parameter("angle_tolerance").as_double() * M_PI / 180.0;

        node_ptr->declare_parameter("expanded_disparity_fg_topic", "expanded_disparity_fg");
        disp_fg_sub_.subscribe(node_ptr,
                               node_ptr->get_parameter("expanded_disparity_fg_topic").as_string());
        node_ptr->declare_parameter("expanded_disparity_bg_topic", "expanded_disparity_bg");
        disp_bg_sub_.subscribe(node_ptr,
                               node_ptr->get_parameter("expanded_disparity_bg_topic").as_string());

        exact_sync.reset(new ExactSync(ExactPolicy(50), disp_fg_sub_, disp_bg_sub_));
        exact_sync->registerCallback(std::bind(&DisparityGraph::disp_cb, this,
                                               std::placeholders::_1, std::placeholders::_2));

        node_ptr->declare_parameter("camera_info_topic", "camera_info");
        cam_info_sub_ = node_ptr->create_subscription<sensor_msgs::msg::CameraInfo>(
            node_ptr->get_parameter("camera_info_topic").as_string(), 1,
            std::bind(&DisparityGraph::get_cam_info, this, std::placeholders::_1));

        marker_.header.frame_id = this->fixed_frame;
        marker_.ns = "disparityGraph";
        marker_.id = 0;
        marker_.type = visualization_msgs::msg::Marker::ARROW;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.pose.position.x = 1;
        marker_.pose.position.y = 1;
        marker_.pose.position.z = 1;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;
        marker_.scale.x = 1;
        marker_.scale.y = 0.2;
        marker_.scale.z = 0.2;
        marker_.color.a = 1.0;  // Don't forget to set the alpha!
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 0.0;
        RCLCPP_INFO(node_ptr->get_logger(), "Disparity Graph Node Started");
    }

    void disp_cb(const sensor_msgs::msg::Image::ConstSharedPtr &disp_fg,
                 const sensor_msgs::msg::Image::ConstSharedPtr &disp_bg) {
        // RCLCPP_INFO(node_ptr->get_logger(), "Received disp stamp: %lf",
        //             (node_ptr->get_clock()->now() - disp_fg->header.stamp).seconds());
        this->sensor_frame = disp_fg->header.frame_id;

        tf2::Stamped<tf2::Transform> transform;
        try {
            auto tf_msg =
                tf_buffer_ptr->lookupTransform(this->sensor_frame, this->fixed_frame, disp_fg->header.stamp);
            tf2::fromMsg(tf_msg, transform);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node_ptr->get_logger(), "DG disp_cb %s", ex.what());
            return;
        }

        std::scoped_lock lock(io_mutex);

        DisparityGraphNode new_graph_node;

        new_graph_node.header = disp_bg->header;
        new_graph_node.w2s_tf = transform;
        new_graph_node.s2w_tf = transform.inverse();
        try {
            new_graph_node.Im_fg =
                cv_bridge::toCvCopy(disp_fg, sensor_msgs::image_encodings::TYPE_32FC1);
            new_graph_node.Im_bg =
                cv_bridge::toCvCopy(disp_bg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(node_ptr->get_logger(), "\n\n\n\n\n\t\t\tDEQUE ERR\n\n\n\n: %s",
                         ex.what());
            return;
        }

        if (first) {
            // if first, initialize the graph by adding the node to the front and back of the queue
            first = false;
            disp_graph_.push_front(new_graph_node);
            disp_graph_.push_back(new_graph_node);

        } else {
            tf2::Vector3 new_position = new_graph_node.s2w_tf.getOrigin();
            std::deque<DisparityGraphNode>::iterator it, end;
            it = disp_graph_.begin();
            end = disp_graph_.end() - 1;
            tf2::Vector3 graph_position = it->s2w_tf.getOrigin();
            tf2Scalar angle_difference = tf2::angleShortestPath(new_graph_node.s2w_tf.getRotation(),
                                                                it->s2w_tf.getRotation());
            tf2Scalar position_difference = new_position.distance(graph_position);

            if (fabs(angle_difference) >= angle_tol ||
                fabs(position_difference) > displacement_tol) {
                RCLCPP_INFO(node_ptr->get_logger(), "Adding new node, graph size now %d",
                            (int)disp_graph_.size());

                if (disp_graph_.size() >= graph_size_) {
                    disp_graph_.erase(end);
                }

                if (disp_graph_.size() <= graph_size_) {
                    disp_graph_.push_front(new_graph_node);
                }
            }
            disp_graph_.pop_back();
            disp_graph_.push_back(new_graph_node);
        }

        publish_disparity_graph_marker();
    }

    void publish_disparity_graph_marker() {
        // create marker
        visualization_msgs::msg::MarkerArray marker_arr;

        for (uint i = 0; i < disp_graph_.size(); i++) {
            auto position = disp_graph_.at(i).s2w_tf.getOrigin();
            tf2::toMsg(position, marker_.pose.position);
            auto rotation = disp_graph_.at(i).s2w_tf.getRotation();
            marker_.pose.orientation = tf2::toMsg(rotation);
            marker_.header.stamp = node_ptr->now();
            marker_.ns = "pose";
            marker_.id = i;
            marker_arr.markers.push_back(marker_);
        }

        disparity_graph_marker_pub_->publish(marker_arr);
    }

    void get_cam_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info_msg) {
        if (got_cam_info) {
            return;
        }
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(cam_info_msg);
        // print
        RCLCPP_INFO(node_ptr->get_logger(),
                    "Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f Baseline: %f", model.fx(),
                    model.fy(), model.cx(), model.cy(), baseline_);
        cx_ = model.cx() / downsample_scale;
        cy_ = model.cy() / downsample_scale;
        fx_ = fy_ = model.fx() / downsample_scale;
        width_ = cam_info_msg->width / downsample_scale;
        height_ = cam_info_msg->height / downsample_scale;
        double baseline = -cam_info_msg->p[3] / cam_info_msg->p[0];
        baseline_ = baseline;
        if (this->baseline_ == 0.0) {
            node_ptr->get_parameter("baseline_fallback", this->baseline_);
            RCLCPP_ERROR_STREAM_ONCE(
                node_ptr->get_logger(),
                "Baseline from camera_info was 0, setting to baseline_fallback:"
                    << this->baseline_);
        }
        baseline_ *= downsample_scale;
        RCLCPP_INFO(node_ptr->get_logger(),
                    "Transformed Cam Info Recvd Fx Fy Cx Cy: %f %f, %f %f Baseline: %f with "
                    "downsamplescale: %f",
                    model.fx(), model.fy(), model.cx(), model.cy(), baseline_, downsample_scale);

        RCLCPP_WARN(node_ptr->get_logger(),
                    "Cam Info Constants - Fx Fy Cx Cy: %f %f, %f %f / width=%d - height=%d", fx_,
                    fy_, cx_, cy_, width_, height_);

        got_cam_info = true;
    }

    /**
     * @brief checks if the pose is seen by any of the disparity images in the graph and if it is,
     * check if it is free
     *
     * @param pose_to_check the pose to check
     * @param thresh threshold for occupancy
     * @return std::tuple<bool, bool, double> is_seen, is_free, occupancy value [0, inf]
     */
    std::tuple<bool, bool, double> is_pose_seen_and_free(
        geometry_msgs::msg::PoseStamped pose_to_check, double thresh) {
        double occupancy = 0.0;  // init
        geometry_msgs::msg::PointStamped checked_point_stamped;
        checked_point_stamped.point = pose_to_check.pose.position;
        checked_point_stamped.header = pose_to_check.header;

        geometry_msgs::msg::PointStamped world_point_stamped;

        try {
            tf_buffer_ptr->transform(checked_point_stamped, world_point_stamped, this->fixed_frame);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node_ptr->get_logger(), "Could not transform %s to %s: %s",
                         this->fixed_frame.c_str(), pose_to_check.header.frame_id.c_str(), ex.what());
            return {false, false, 0.0};
        }

        tf2::Vector3 optical_point;
        tf2::fromMsg(world_point_stamped.point, optical_point);

        std::scoped_lock lock(io_mutex);

        bool is_free = true;   // whether the point is unoccupied. we will go through all disparity
                               // graph nodes to check
        bool is_seen = false;  // whether the point is is_seen by any of the disparity images

        if (disp_graph_.size() == 0) {
            RCLCPP_WARN_STREAM_ONCE(
                node_ptr->get_logger(),
                "No disparity images in graph, can't see anything, everything is invalid");
        }

        for (const auto &graph_node : disp_graph_) {
            tf2::Vector3 local_point = graph_node.w2s_tf * optical_point;

            float x = local_point.getX();
            float y = local_point.getY();
            float z = local_point.getZ();

            // RCLCPP_INFO_STREAM(node_ptr->get_logger(), "Local Point: " << x << " " << y << " " <<
            // z);

            if (local_point.length() < this->assume_seen_within_radius) {
                is_seen = true;  // assume the point is seen if it is close to the camera
                continue;
            }

            // project the 3D point to the image plane
            int u = x / z * fx_ + cx_;
            int v = y / z * fy_ + cy_;

            // check that the point is within the image bounds, and that it is in front of the
            // camera
            if (u >= 0 && u < width_ && v >= 0 && v < height_ && z > 0.0) {
                is_seen = true;

                double state_disparity = baseline_ * fx_ / z;
                // RCLCPP_INFO_STREAM(node_ptr->get_logger(), "State Disparity: " <<
                // state_disparity);

                // see Table 1 of Dubey et al. 2017 DROAN. it seems a little different
                bool is_state_between_fg_and_bg =
                    (state_disparity < graph_node.Im_fg->image.at<float>(v, u)) &&
                    (state_disparity > graph_node.Im_bg->image.at<float>(v, u));
                if (is_state_between_fg_and_bg) {
                    // if the disparity point lies between the disparity images, it is occupied and
                    // we add to the occupancy value.
                    // TODO: what does this value mean? is it the sensor model? It seems to be a
                    // heuristic based on distance. also shouldn't we clamp this at 1.0? or is it
                    // not really a probability?
                    occupancy += (state_disparity - 0.5) / state_disparity;
                } else {
                    // otherwise it's outside, we subtdroan.rvizract from the occupancy value
                    occupancy -= 0.5 * (state_disparity - 0.5) / state_disparity;
                    occupancy = std::clamp(occupancy, 0.0, 1.0);
                }
            }
            if (occupancy >= thresh) {
                is_free = false;
                break;
            }
        }
        // RCLCPP_INFO_STREAM(node_ptr->get_logger(),"occupancy:" << occupancy << " is_free: " <<
        // is_free << " is_seen: " << is_seen);

        return {is_seen, is_free, occupancy};
    }

    bool get_state_cost(geometry_msgs::msg::Pose checked_state, double &cost) {
        geometry_msgs::msg::PointStamped checked_point_stamped;
        checked_point_stamped.header.frame_id = "world";
        checked_point_stamped.header.stamp = node_ptr->now();  // - rclcpp::Duration(0.1);
        checked_point_stamped.point = checked_state.position;

        geometry_msgs::msg::PointStamped world_point_stamped;

        try {
            tf_buffer_ptr->transform(checked_point_stamped, world_point_stamped, this->fixed_frame);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node_ptr->get_logger(), "TF to fixed frame failed: %s", ex.what());
            return false;
        }

        tf2::Vector3 optical_point;
        tf2::fromMsg(world_point_stamped.point, optical_point);

        std::lock_guard<std::mutex> lock(io_mutex);
        for (uint i = 0; i < disp_graph_.size(); i++) {
            tf2::Vector3 local_point = disp_graph_.at(i).w2s_tf * optical_point;

            float x, y, z;

            x = local_point.getX();
            y = local_point.getY();
            z = local_point.getZ();

            if (z < 0.0) {
                return false;
            }

            int u = x / z * fx_ + cx_;
            int v = y / z * fy_ + cy_;

            if (u >= 0 && u < width_ && v >= 0 && v < height_ && z > 0.0) {
                double state_disparity = baseline_ * fx_ / z;
                if ((disp_graph_.at(i).Im_fg->image.at<float>(v, u) > state_disparity) &&
                    (disp_graph_.at(i).Im_bg->image.at<float>(v, u) < state_disparity)) {
                    cost += (state_disparity - 0.5) / state_disparity;
                }
                cost = cost < 0.0 ? 0.0 : cost;

                if (cost > 100.0) {
                    cost = 100.0;
                    return false;
                }
            }
        }
        return true;
    }

    void clear_graph(void) {
        std::lock_guard<std::mutex> lock(io_mutex);
        disp_graph_.clear();
        first = true;
        RCLCPP_INFO_STREAM(node_ptr->get_logger(), "<<< DISP GRAPH CLEARED >>>");
    };

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcdPub, occPub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr expansion_poly_pub;
    float orig_z;
};

}  // namespace disparity_graph
