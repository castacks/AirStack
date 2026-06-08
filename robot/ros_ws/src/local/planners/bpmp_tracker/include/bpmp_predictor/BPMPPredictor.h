#ifndef BPMP_PREDICTOR_H   
#define BPMP_PREDICTOR_H
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "bpmp_tracker/srv/set_target_id.hpp"
#include "bpmp_tracker/srv/set_mode.hpp"
// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// message_filters for synchronized subscription
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// HumanFlow messages
#include "humanflow_msgs/msg/human_flow_array.hpp"
#include "humanflow_msgs/msg/re_id_groups.hpp"
// Eigen
#include <Eigen/Dense>
#include <map>

namespace bpmp_tracker{
    enum class PredictorMode {
        BBOX_3D_MODE   = 0,  // Mode 0: 3D Bbox
        BBOX_2D_MODE_1 = 1,  // Mode 1: 2D Bbox + Depth Image
        BBOX_2D_MODE_2 = 2   // Mode 2: 2D Bbox (HumanFlow) + Depth Image
    };
    using SyncPolicy2D = message_filters::sync_policies::ApproximateTime<
        vision_msgs::msg::Detection2DArray,
        sensor_msgs::msg::Image>;
    using SyncPolicyHF = message_filters::sync_policies::ApproximateTime<
        humanflow_msgs::msg::HumanFlowArray,
        sensor_msgs::msg::Image>;

    class BPMPPredictor : public rclcpp::Node{
        private:

        bool target_info_received_{false};
        int target_id_{0};
        int last_detection_count_{0};
        PredictorMode current_mode_{PredictorMode::BBOX_3D_MODE};

        // MODE 0: 3D bbox subscribers
        rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_array_sub_;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr vdb_map_sub_;

        // MODE 1: 2D bbox + depth image synchronized subscribers
        message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection2d_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy2D>> sync2d_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;

        // MODE 2: HumanFlow + depth image synchronized subscribers
        message_filters::Subscriber<humanflow_msgs::msg::HumanFlowArray> humanflow_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicyHF>> sync_hf_;
        rclcpp::Subscription<humanflow_msgs::msg::ReIDGroups>::SharedPtr reid_groups_sub_;
        std::map<int, int> reid_id_mapping_;  // YOLO tracking_id -> global group_id

        rclcpp::Service<bpmp_tracker::srv::SetTargetId>::SharedPtr set_target_id_srv_;
        rclcpp::Service<bpmp_tracker::srv::SetMode>::SharedPtr set_mode_srv_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_info_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filtered_vdb_map_publisher_;
        geometry_msgs::msg::PoseStamped target_info_msg_;

        // TF2
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string target_frame_{"map"};
        std::string robot_name_{"drone1"};

        Eigen::Affine3d getAffineTransform(const geometry_msgs::msg::TransformStamped& transform);

        void Run();
        void Detection3DArrayCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
        void Detection2DDepthCallback(
            const vision_msgs::msg::Detection2DArray::ConstSharedPtr det_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
        void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void SetTargetIdCallback(
            const bpmp_tracker::srv::SetTargetId::Request::SharedPtr req,
            bpmp_tracker::srv::SetTargetId::Response::SharedPtr res);
        void SetModeCallback(
            const bpmp_tracker::srv::SetMode::Request::SharedPtr req,
            bpmp_tracker::srv::SetMode::Response::SharedPtr res);
        void VDBMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
        void HumanFlowDepthCallback(
            const humanflow_msgs::msg::HumanFlowArray::ConstSharedPtr hf_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
        void ReIDGroupsCallback(const humanflow_msgs::msg::ReIDGroups::SharedPtr msg);
        public:
        BPMPPredictor();
        ~BPMPPredictor()=default;
        void initialize();
    };
}

#endif