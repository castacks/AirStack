#ifndef BPMP_PREDICTOR_H   
#define BPMP_PREDICTOR_H
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
// (Yunwoo) TF2 for frame transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// (Yunwoo) Eigen for matrix operations
#include <Eigen/Dense>

namespace bpmp_tracker{
    enum class PredictorMode {
        BBOX_3D_MODE = 0,  // Mode 0: USING 3D BBOX
        BBOX_2D_MODE_1 = 1,              // Mode 1: USING 2D BBOX WITH KNOWN TARGET HEIGHT
        BBOX_2D_MODE_2 = 2     // Mode 2: USING 2D BBOX WITH GROUND PLANES
    };
    class BPMPPredictor : public rclcpp::Node{
        private:

        bool target_info_received_{false};
        PredictorMode current_mode_{PredictorMode::BBOX_3D_MODE}; // Default mode is 3D BBOX
        rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_array_sub_;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr vdb_map_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_info_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filtered_vdb_map_publisher_;
        geometry_msgs::msg::PoseStamped target_info_msg_;
        

        
        // (Yunwoo) TF2 for frame transformation
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string target_frame_{"map"};  // Target frame to transform to
        
        // (Yunwoo) Helper function to get affine matrix from transform
        Eigen::Affine3d getAffineTransform(const geometry_msgs::msg::TransformStamped& transform);
        
        void Run();
        void Detection3DArrayCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
        void VDBMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
        public:
        BPMPPredictor();
        ~BPMPPredictor()=default;
        void initialize();
    };
}

#endif