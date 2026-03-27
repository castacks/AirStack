#include "bpmp_predictor/BPMPPredictor.h"

namespace bpmp_tracker{
    BPMPPredictor::BPMPPredictor() : Node("bpmp_predictor"){
        initialize();
    }

    void BPMPPredictor::initialize(){
        // Parameters
        this->declare_parameter<int>("mode", 0);
        this->declare_parameter<std::string>("target_frame", "map");
        current_mode_ = static_cast<PredictorMode>(this->get_parameter("mode").as_int());
        target_frame_ = this->get_parameter("target_frame").as_string();

        // (Yunwoo) Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        detection3d_array_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/robot_1/sensors/front_stereo/left/semantic_bbox", 5,
            std::bind(&BPMPPredictor::Detection3DArrayCallback, this, std::placeholders::_1));
        vdb_map_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/robot_1/vdb_mapping/vdb_map_visualization", 5,
            std::bind(&BPMPPredictor::VDBMapCallback, this, std::placeholders::_1));
        target_info_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_info", 5);
        filtered_vdb_map_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("filtered_vdb_map", 5);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&BPMPPredictor::Run, this));
    }

    void BPMPPredictor::Run(){
        if(current_mode_ == PredictorMode::BBOX_3D_MODE){
            // TODO: Implement 3D BBOX prediction
        }else if(current_mode_ == PredictorMode::BBOX_2D_MODE_1){
            // TODO: Implement 2D BBOX with known target height prediction
        }else if(current_mode_ == PredictorMode::BBOX_2D_MODE_2){
            // TODO: Implement 2D BBOX with ground planes prediction
        }
    }

    Eigen::Affine3d BPMPPredictor::getAffineTransform(const geometry_msgs::msg::TransformStamped& transform){
        // (Yunwoo) Convert geometry_msgs::TransformStamped to Eigen::Affine3d
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        
        // Extract translation
        affine.translation() << transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z;
        
        // Extract rotation (quaternion to rotation matrix)
        Eigen::Quaterniond q(transform.transform.rotation.w,
                             transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z);
        affine.linear() = q.toRotationMatrix();
        
        return affine;
    }
    void BPMPPredictor::VDBMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg){
        if(!target_info_received_){
            filtered_vdb_map_publisher_->publish(*msg);
            return;
        }
        visualization_msgs::msg::Marker filtered_vdb_map;
        filtered_vdb_map.header.frame_id = "map";
        filtered_vdb_map.header.stamp = this->now();
        filtered_vdb_map.id = 0;
        filtered_vdb_map.ns = "filtered_vdb_map";
        filtered_vdb_map.type = visualization_msgs::msg::Marker::CUBE_LIST;
        filtered_vdb_map.action = visualization_msgs::msg::Marker::ADD;
        filtered_vdb_map.pose = msg->pose;
        filtered_vdb_map.scale = msg->scale;
        // filtered_vdb_map.color = msg->color;
        filtered_vdb_map.points.clear();
        for(int i = 0; i < msg->points.size(); i++){
            const auto& point = msg->points[i];
            // Skip points within 1.0m box around target
            if(std::abs(point.x - target_info_msg_.pose.position.x) < 1.0 && 
               std::abs(point.y - target_info_msg_.pose.position.y) < 1.0 && 
               std::abs(point.z - target_info_msg_.pose.position.z) < 2.5)
                continue;
            filtered_vdb_map.points.push_back(point);
            filtered_vdb_map.colors.push_back(msg->colors[i]);
        }
        filtered_vdb_map_publisher_->publish(filtered_vdb_map);
    }
    void BPMPPredictor::Detection3DArrayCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){
        if (msg->detections.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No detections received");
            return;
        }
        target_info_received_ = true;
        // (Yunwoo) Extract position from detection (in world frame)
        const auto& bbox_center = msg->detections[0].bbox.center.position;
        Eigen::Vector3d position_world(bbox_center.x, bbox_center.y, bbox_center.z);

        try {
            // (Yunwoo) Get transform from world to map frame
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(target_frame_, "world", tf2::TimePointZero);
            
            // (Yunwoo) Get affine transformation matrix
            Eigen::Affine3d T_map_world = getAffineTransform(transform_stamped);
            
            // (Yunwoo) Transform position: p_map = T_map_world * p_world
            Eigen::Vector3d position_map = T_map_world * position_world;
            
            // (Yunwoo) Publish transformed position as PoseStamped with identity quaternion
            
            target_info_msg_.header.stamp = this->now();
            target_info_msg_.header.frame_id = target_frame_;
            target_info_msg_.pose.position.x = position_map.x();
            target_info_msg_.pose.position.y = position_map.y();
            target_info_msg_.pose.position.z = position_map.z();
            target_info_msg_.pose.orientation.x = 0.0;
            target_info_msg_.pose.orientation.y = 0.0;
            target_info_msg_.pose.orientation.z = 0.0;
            target_info_msg_.pose.orientation.w = 1.0;
            target_info_publisher_->publish(target_info_msg_);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF2 transform exception: %s", ex.what());
            return;
        }
    }
}