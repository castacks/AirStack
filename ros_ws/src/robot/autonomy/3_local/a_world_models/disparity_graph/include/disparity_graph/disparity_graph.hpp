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

#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace disparity_graph {

class DisparityGraph : rclcpp::Node {
    struct DisparityGraphNode {
        cv_bridge::CvImagePtr Im_fg;
        cv_bridge::CvImagePtr Im_bg;
        std_msgs::msg::Header header;
        tf2::Transform w2s_tf;
        tf2::Transform s2w_tf;
    };
    std::deque<DisparityGraphNode> disp_graph_;

    size_t graph_size_;
    double thresh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string sensor_frame_, fixed_frame_, stabilized_frame_;
    visualization_msgs::msg::Marker marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr disparity_graph_marker_pub_;
    double angle_tol, displacement_tol;
    bool first;
    bool got_cam_info;
    double fx_, fy_, cx_, cy_, baseline_, downsample_scale;
    unsigned int width_, height_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    std::mutex io_mutex;

    message_filters::Subscriber<sensor_msgs::msg::Image> disp_fg_sub_, disp_bg_sub_;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>
        ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

    std::shared_ptr<ExactSync> exact_sync;

   public:
    DisparityGraph()
        : Node("disparity_graph"),
          graph_size_(10),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          fixed_frame_("world"),
          stabilized_frame_("base_frame_stabilized"),
          angle_tol(30.0),
          displacement_tol(1.5),
          first(true),
          got_cam_info(false) {
        disparity_graph_marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("disparity_marker", 10);
        marker_.header.frame_id = fixed_frame_;
        marker_.header.stamp = this->get_clock()->now();
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
        marker_.scale.y = 0.1;
        marker_.scale.z = 0.1;
        marker_.color.a = 1.0;  // Don't forget to set the alpha!
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 0.0;

        declare_parameter("baseline", 0.10);
        baseline_ = get_parameter("baseline").as_double();
        declare_parameter("downsample_scale", 1.0);
        downsample_scale = get_parameter("downsample_scale").as_double();
        declare_parameter("low_occ_thresh", 0.9);
        thresh_ = get_parameter("low_occ_thresh").as_double();
        declare_parameter("graph_size", 10);
        graph_size_ = get_parameter("graph_size").as_int();
        declare_parameter("displacement_tolerance", 1.5);
        displacement_tol = get_parameter("displacement_tolerance").as_double();
        declare_parameter("angle_tolerance", 30.0);
        angle_tol = get_parameter("angle_tolerance").as_double() * M_PI / 180.0;

        disp_fg_sub_.subscribe(this, "/ceye/left/expanded_disparity_fg");
        disp_bg_sub_.subscribe(this, "/ceye/left/expanded_disparity_bg");

        exact_sync.reset(new ExactSync(ExactPolicy(5), disp_fg_sub_, disp_bg_sub_));
        exact_sync->registerCallback(std::bind(&DisparityGraph::disp_cb, this,
                                               std::placeholders::_1, std::placeholders::_2));

        cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/nerian_sp1/right/camera_info", 1,
            std::bind(&DisparityGraph::get_cam_info, this, std::placeholders::_1));
    }

  //virtual ~DisparityGraph();

    void disp_cb(const sensor_msgs::msg::Image::ConstSharedPtr &disp_fg,
                 const sensor_msgs::msg::Image::ConstSharedPtr &disp_bg) {
        RCLCPP_INFO(this->get_logger(), "Recvd disp stamp: %lf",
                    (this->get_clock()->now() - disp_fg->header.stamp).seconds());
        sensor_frame_ = disp_fg->header.frame_id;

        tf2::Stamped<tf2::Transform> transform;
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_.lookupTransform(sensor_frame_, fixed_frame_, disp_fg->header.stamp);
            tf2::fromMsg(tf_msg, transform);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "DG disp_cb %s", ex.what());
            return;
        }

        std::scoped_lock lock(io_mutex);

        if (first) {
            first = false;
            DisparityGraphNode n;
            try {
                n.Im_fg = cv_bridge::toCvCopy(disp_fg, sensor_msgs::image_encodings::TYPE_32FC1);
                n.Im_bg = cv_bridge::toCvCopy(disp_bg, sensor_msgs::image_encodings::TYPE_32FC1);
            } catch (std::exception ex) {
                RCLCPP_ERROR(this->get_logger(), "\n\n\n\n\n\t\t\tDEQUE ERR\n\n\n\n: %s",
                             ex.what());
                return;
            }

            n.header = disp_bg->header;
            n.w2s_tf = transform;
            n.s2w_tf = transform.inverse();
            disp_graph_.push_front(n);
            disp_graph_.push_back(n);

        } else {
            tf2::Vector3 curr_p = transform.inverse().getOrigin();
            std::deque<DisparityGraphNode>::iterator it, end;
            it = disp_graph_.begin();
            end = disp_graph_.end() - 1;
            tf2::Vector3 graph_p = it->s2w_tf.getOrigin();
            tf2Scalar diff_a =
                tf2::angleShortestPath(transform.inverse().getRotation(), it->s2w_tf.getRotation());
            tf2Scalar diff_p = curr_p.distance(graph_p);

            DisparityGraphNode n;
            try {
                n.Im_fg = cv_bridge::toCvCopy(disp_fg, sensor_msgs::image_encodings::TYPE_32FC1);
                n.Im_bg = cv_bridge::toCvCopy(disp_bg, sensor_msgs::image_encodings::TYPE_32FC1);
            } catch (std::exception ex) {
                RCLCPP_ERROR(this->get_logger(), "\n\n\n\n\n\t\t\tDEQUE ERR\n\n\n\n: %s",
                             ex.what());
                return;
            }

            n.header = disp_bg->header;
            n.w2s_tf = transform;
            n.s2w_tf = transform.inverse();

            if (fabs(diff_a) >= angle_tol || fabs(diff_p) > displacement_tol) {
                RCLCPP_ERROR(this->get_logger(), "Adding new node size %d",
                             (int)disp_graph_.size());
                if (disp_graph_.size() >= graph_size_) {
                    disp_graph_.erase(end);
                }

                if (disp_graph_.size() <= graph_size_) {
                    disp_graph_.push_front(n);
                }
            }
            disp_graph_.pop_back();
            disp_graph_.push_back(n);
        }
    }

    void get_cam_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info_msg) {
        if (got_cam_info) {
            return;
        }
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(cam_info_msg);
        // print
        RCLCPP_INFO(this->get_logger(), "Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f Baseline: %f",
                    model.fx(), model.fy(), model.cx(), model.cy(), baseline_);
        cx_ = model.cx() / downsample_scale;
        cy_ = model.cy() / downsample_scale;
        fx_ = fy_ = model.fx() / downsample_scale;
        width_ = cam_info_msg->width / downsample_scale;
        height_ = cam_info_msg->height / downsample_scale;
        double baseline = -cam_info_msg->p[3] / cam_info_msg->p[0];
        if (baseline != 0.0) {
            baseline_ = baseline;
        }
        baseline_ *= downsample_scale;
        RCLCPP_INFO(this->get_logger(),
                    "Transformed Cam Info Recvd Fx Fy Cx Cy: %f %f, %f %f Baseline: %f with "
                    "downsamplescale: %f",
                    model.fx(), model.fy(), model.cx(), model.cy(), baseline_, downsample_scale);

        RCLCPP_WARN(this->get_logger(),
                    "Cam Info Constants - Fx Fy Cx Cy: %f %f, %f %f / width=%d - height=%d", fx_,
                    fy_, cx_, cy_, width_, height_);

        got_cam_info = true;
    }

    bool is_state_valid_depth_pose(geometry_msgs::msg::PoseStamped checked_state, double thresh,
                                   double &occupancy) {
        occupancy = 0.0;
        geometry_msgs::msg::PointStamped checked_point_stamped;
        checked_point_stamped.point = checked_state.pose.position;
        checked_point_stamped.header = checked_state.header;

        geometry_msgs::msg::PointStamped world_point_stamped;

        try {
            tf_buffer_.transform(checked_point_stamped, world_point_stamped, fixed_frame_);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                         fixed_frame_.c_str(), checked_state.header.frame_id.c_str(), ex.what());
            return false;
        }

        tf2::Vector3 optical_point;
        tf2::fromMsg(world_point_stamped.point, optical_point);

        std::scoped_lock lock(io_mutex);

        bool seen = false;
        for (uint i = 0; i < disp_graph_.size(); i++) {
            tf2::Vector3 local_point = disp_graph_.at(i).w2s_tf * optical_point;

            float x, y, z;

            x = local_point.getX();
            y = local_point.getY();
            z = local_point.getZ();

            if (local_point.length() < 1.0) {
                seen = true;
                continue;
            }

            int u = x / z * fx_ + cx_;
            int v = y / z * fy_ + cy_;

            if (u >= 0 && u < width_ && v >= 0 && v < height_ && z > 0.0) {
                seen = true;
                double state_disparity = baseline_ * fx_ / z;
                if ((disp_graph_.at(i).Im_fg->image.at<float>(v, u) > state_disparity) &&
                    (disp_graph_.at(i).Im_bg->image.at<float>(v, u) < state_disparity)) {
                    occupancy += (state_disparity - 0.5) / state_disparity;
                } else {
                    occupancy -= 0.5 * (state_disparity - 0.5) / state_disparity;
                    occupancy = occupancy < 0.0 ? 0.0 : occupancy;
                }
            }
            if (occupancy >= thresh) {
                return false;
            }
        }
        return seen;
    }

    bool get_state_cost(geometry_msgs::msg::Pose checked_state, double &cost) {
        geometry_msgs::msg::PointStamped checked_point_stamped;
        checked_point_stamped.header.frame_id = "world";
        checked_point_stamped.header.stamp = this->now();// - rclcpp::Duration(0.1);
        checked_point_stamped.point = checked_state.position;

        geometry_msgs::msg::PointStamped world_point_stamped;

        try {
            tf_buffer_.transform(checked_point_stamped, world_point_stamped, fixed_frame_);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF to fixed frame failed: %s", ex.what());
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
        RCLCPP_INFO_STREAM(this->get_logger(), "<<< DISP GRAPH CLEARED >>>");
    };

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcdPub, occPub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr expansion_poly_pub;
    float orig_z;
};

}  // namespace disparity_graph
