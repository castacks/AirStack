#include "bpmp_predictor/BPMPPredictor.h"

namespace bpmp_tracker{
    BPMPPredictor::BPMPPredictor() : Node("bpmp_predictor"){
        initialize();
    }

    void BPMPPredictor::initialize(){
        // Parameters
        this->declare_parameter<std::string>("target_frame", "map");
        target_frame_ = this->get_parameter("target_frame").as_string();

        const char* env_robot_name = std::getenv("ROBOT_NAME");
        if (env_robot_name) robot_name_ = env_robot_name;

        this->declare_parameter<int>("initial_mode", 0);
        this->declare_parameter<int>("initial_target_id", 0);
        int initial_mode = this->get_parameter("initial_mode").as_int();
        int initial_target_id = this->get_parameter("initial_target_id").as_int();
        if (initial_mode >= 0 && initial_mode <= 2)
            current_mode_ = static_cast<PredictorMode>(initial_mode);
        if (initial_target_id >= 0)
            target_id_ = initial_target_id;
        RCLCPP_INFO(this->get_logger(), "Initial mode: %d, initial target_id: %d", initial_mode, initial_target_id);

        // (Yunwoo) Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers — always created; current_mode_ gates which callback publishes
        detection3d_array_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            "semantic_bbox", 5,
            std::bind(&BPMPPredictor::Detection3DArrayCallback, this, std::placeholders::_1));
        detection2d_sub_.subscribe(this, "detection2d");
        depth_image_sub_.subscribe(this, "depth_image");
        sync2d_ = std::make_shared<message_filters::Synchronizer<SyncPolicy2D>>(
            SyncPolicy2D(10), detection2d_sub_, depth_image_sub_);
        sync2d_->registerCallback(
            std::bind(&BPMPPredictor::Detection2DDepthCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(5),
            std::bind(&BPMPPredictor::CameraInfoCallback, this, std::placeholders::_1));
        // Mode 2: HumanFlow + depth image (shares depth_image_sub_ with Mode 1)
        humanflow_sub_.subscribe(this, "humanflow");
        sync_hf_ = std::make_shared<message_filters::Synchronizer<SyncPolicyHF>>(
            SyncPolicyHF(10), humanflow_sub_, depth_image_sub_);
        sync_hf_->registerCallback(
            std::bind(&BPMPPredictor::HumanFlowDepthCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        reid_groups_sub_ = this->create_subscription<humanflow_msgs::msg::ReIDGroups>(
            "reid_groups", 10,
            std::bind(&BPMPPredictor::ReIDGroupsCallback, this, std::placeholders::_1));
        // Service servers
        set_mode_srv_ = this->create_service<bpmp_tracker::srv::SetMode>(
            "set_mode",
            std::bind(&BPMPPredictor::SetModeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        set_target_id_srv_ = this->create_service<bpmp_tracker::srv::SetTargetId>(
            "set_target_id",
            std::bind(&BPMPPredictor::SetTargetIdCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        vdb_map_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "vdb_map", 5,
            std::bind(&BPMPPredictor::VDBMapCallback, this, std::placeholders::_1));
        target_info_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_info", 5);
        filtered_vdb_map_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("filtered_vdb_map", 5);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&BPMPPredictor::Run, this));
    }

    void BPMPPredictor::Run(){
        if(current_mode_ == PredictorMode::BBOX_3D_MODE){
            // TODO: Implement Mode 0 — 3D Bbox prediction
        }else if(current_mode_ == PredictorMode::BBOX_2D_MODE_1){
            // TODO: Implement Mode 1 — 2D Bbox + Depth Image prediction
        }else if(current_mode_ == PredictorMode::BBOX_2D_MODE_2){
            // TODO: Implement Mode 2 — 2D Bbox (HumanFlow) + Depth Image prediction
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
    void BPMPPredictor::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
        camera_info_ = msg;
    }

    void BPMPPredictor::SetModeCallback(
        const bpmp_tracker::srv::SetMode::Request::SharedPtr req,
        bpmp_tracker::srv::SetMode::Response::SharedPtr res)
    {
        if (req->mode < 0 || req->mode > 2) {
            res->success = false;
            res->message = "Invalid mode " + std::to_string(req->mode) + " (valid: 0=3D_Bbox, 1=2D_Bbox+Depth, 2=2D_Bbox(HumanFlow)+Depth)";
            return;
        }
        current_mode_ = static_cast<PredictorMode>(req->mode);
        target_info_received_ = false;
        res->success = true;
        res->message = "Mode set to " + std::to_string(req->mode);
        RCLCPP_INFO(this->get_logger(), "Mode set to %d", req->mode);
    }

    void BPMPPredictor::Detection2DDepthCallback(
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr det_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
    {
        if (current_mode_ != PredictorMode::BBOX_2D_MODE_1) return;
        if (det_msg->detections.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No 2D detections received");
            return;
        }
        if (!camera_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for camera_info");
            return;
        }

        last_detection_count_ = static_cast<int>(det_msg->detections.size());
        const int idx = (target_id_ < last_detection_count_) ? target_id_ : 0;
        if (target_id_ >= last_detection_count_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "target_id %d out of range (%d detections), falling back to 0", target_id_, last_detection_count_);
        }

        // Pixel coordinates of bbox center
        const double u = det_msg->detections[idx].bbox.center.position.x;
        const double v = det_msg->detections[idx].bbox.center.position.y;
        const int iu = static_cast<int>(std::round(u));
        const int iv = static_cast<int>(std::round(v));

        if (iu < 0 || iu >= static_cast<int>(depth_msg->width) ||
            iv < 0 || iv >= static_cast<int>(depth_msg->height)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "bbox center (%d, %d) out of depth image bounds (%u x %u)", iu, iv,
                depth_msg->width, depth_msg->height);
            return;
        }

        // Sample depth at bbox center — support 32FC1 (m) and 16UC1 (mm)
        double depth = 0.0;
        if (depth_msg->encoding == "32FC1") {
            const float* row = reinterpret_cast<const float*>(
                &depth_msg->data[iv * depth_msg->step]);
            depth = static_cast<double>(row[iu]);
        } else if (depth_msg->encoding == "16UC1") {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(
                &depth_msg->data[iv * depth_msg->step]);
            depth = static_cast<double>(row[iu]) / 1000.0;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
            return;
        }

        if (!std::isfinite(depth) || depth <= 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Invalid depth value %.3f at bbox center", depth);
            return;
        }

        // Back-project to camera frame using pinhole model
        const double fx = camera_info_->k[0];
        const double fy = camera_info_->k[4];
        const double cx = camera_info_->k[2];
        const double cy = camera_info_->k[5];

        geometry_msgs::msg::PointStamped point_cam;
        point_cam.header = depth_msg->header;
        point_cam.point.x = (u - cx) * depth / fx;
        point_cam.point.y = (v - cy) * depth / fy;
        point_cam.point.z = depth;

        // Transform to map frame
        try {
            geometry_msgs::msg::PointStamped point_map;
            tf_buffer_->transform(point_cam, point_map, target_frame_, tf2::durationFromSec(0.1));

            target_info_received_ = true;
            target_info_msg_.header.stamp = this->now();
            target_info_msg_.header.frame_id = target_frame_;
            target_info_msg_.pose.position.x = point_map.point.x;
            target_info_msg_.pose.position.y = point_map.point.y;
            target_info_msg_.pose.position.z = point_map.point.z;
            target_info_msg_.pose.orientation.w = 1.0;
            target_info_msg_.pose.orientation.x = 0.0;
            target_info_msg_.pose.orientation.y = 0.0;
            target_info_msg_.pose.orientation.z = 0.0;
            target_info_publisher_->publish(target_info_msg_);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF2 transform exception: %s", ex.what());
        }
    }

    void BPMPPredictor::SetTargetIdCallback(
        const bpmp_tracker::srv::SetTargetId::Request::SharedPtr req,
        bpmp_tracker::srv::SetTargetId::Response::SharedPtr res)
    {
        if (req->target_id < 0) {
            res->success = false;
            res->message = "target_id must be >= 0";
            return;
        }
        if (last_detection_count_ > 0 && req->target_id >= last_detection_count_) {
            res->success = false;
            res->message = "target_id " + std::to_string(req->target_id) +
                           " is out of range (last detection count: " +
                           std::to_string(last_detection_count_) + ")";
            return;
        }
        target_id_ = req->target_id;
        res->success = true;
        res->message = "target_id set to " + std::to_string(target_id_);
        RCLCPP_INFO(this->get_logger(), "Target ID set to %d", target_id_);
    }

    void BPMPPredictor::ReIDGroupsCallback(const humanflow_msgs::msg::ReIDGroups::SharedPtr msg) {
        reid_id_mapping_.clear();
        for (const auto& group : msg->groups) {
            int global_id = 0;
            try { global_id = std::stoi(group.group_name); }
            catch (...) { continue; }
            for (const auto& track : group.tracks) {
                // track format: "{robot_name}_{yolo_tracking_id}"
                const std::string prefix = robot_name_ + "_";
                if (track.rfind(prefix, 0) != 0) continue;
                try {
                    int yolo_id = std::stoi(track.substr(prefix.size()));
                    reid_id_mapping_[yolo_id] = global_id;
                } catch (...) {}
            }
        }
    }

    void BPMPPredictor::HumanFlowDepthCallback(
        const humanflow_msgs::msg::HumanFlowArray::ConstSharedPtr hf_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
    {
        if (current_mode_ != PredictorMode::BBOX_2D_MODE_2) return;
        if (hf_msg->humanflows.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "No HumanFlow detections received");
            return;
        }
        if (!camera_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for camera_info");
            return;
        }

        // Find the humanflow entry whose global ReID group_id matches a given ID.
        auto find_human_by_global_id =
            [&](int global_id) -> const humanflow_msgs::msg::HumanFlow* {
                for (const auto& human : hf_msg->humanflows) {
                    auto it = reid_id_mapping_.find(human.tracking_id);
                    if (it != reid_id_mapping_.end() && it->second == global_id) {
                        return &human;
                    }
                }
                return nullptr;
            };

        // Follow the primary target (target_id_, 0 by default); if it is not
        // currently visible, fall back to global ReID ID 1.
        constexpr int kFallbackTargetId = 1;
        const humanflow_msgs::msg::HumanFlow* target_human =
            find_human_by_global_id(target_id_);
        if (target_human == nullptr && target_id_ != kFallbackTargetId) {
            target_human = find_human_by_global_id(kFallbackTargetId);
            if (target_human != nullptr) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Target global ID %d not visible; falling back to global ID %d",
                    target_id_, kFallbackTargetId);
            }
        }
        if (target_human == nullptr) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Neither target global ID %d nor fallback %d found in ReID "
                "mapping (mapping size: %zu)",
                target_id_, kFallbackTargetId, reid_id_mapping_.size());
            return;
        }

        const double u = target_human->bbox.center.position.x;
        const double v = target_human->bbox.center.position.y;
        const int iu = static_cast<int>(std::round(u));
        const int iv = static_cast<int>(std::round(v));

        if (iu < 0 || iu >= static_cast<int>(depth_msg->width) ||
            iv < 0 || iv >= static_cast<int>(depth_msg->height)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "bbox center (%d, %d) out of depth image bounds (%u x %u)",
                iu, iv, depth_msg->width, depth_msg->height);
            return;
        }

        double depth = 0.0;
        if (depth_msg->encoding == "32FC1") {
            const float* row = reinterpret_cast<const float*>(
                &depth_msg->data[iv * depth_msg->step]);
            depth = static_cast<double>(row[iu]);
        } else if (depth_msg->encoding == "16UC1") {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(
                &depth_msg->data[iv * depth_msg->step]);
            depth = static_cast<double>(row[iu]) / 1000.0;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
            return;
        }

        if (!std::isfinite(depth) || depth <= 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Invalid depth %.3f at HumanFlow bbox center", depth);
            return;
        }

        const double fx = camera_info_->k[0];
        const double fy = camera_info_->k[4];
        const double cx = camera_info_->k[2];
        const double cy = camera_info_->k[5];

        geometry_msgs::msg::PointStamped point_cam;
        point_cam.header = depth_msg->header;
        point_cam.point.x = (u - cx) * depth / fx;
        point_cam.point.y = (v - cy) * depth / fy;
        point_cam.point.z = depth;

        try {
            geometry_msgs::msg::PointStamped point_map;
            tf_buffer_->transform(point_cam, point_map, target_frame_, tf2::durationFromSec(0.1));

            target_info_received_ = true;
            target_info_msg_.header.stamp = this->now();
            target_info_msg_.header.frame_id = target_frame_;
            target_info_msg_.pose.position.x = point_map.point.x;
            target_info_msg_.pose.position.y = point_map.point.y;
            target_info_msg_.pose.position.z = point_map.point.z;
            target_info_msg_.pose.orientation.w = 1.0;
            target_info_msg_.pose.orientation.x = 0.0;
            target_info_msg_.pose.orientation.y = 0.0;
            target_info_msg_.pose.orientation.z = 0.0;
            target_info_publisher_->publish(target_info_msg_);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF2 transform exception: %s", ex.what());
        }
    }

    void BPMPPredictor::Detection3DArrayCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){
        if (current_mode_ != PredictorMode::BBOX_3D_MODE) return;
        if (msg->detections.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No detections received");
            return;
        }
        last_detection_count_ = static_cast<int>(msg->detections.size());
        const int idx = (target_id_ < last_detection_count_) ? target_id_ : 0;
        if (target_id_ >= last_detection_count_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "target_id %d out of range (%d detections), falling back to 0", target_id_, last_detection_count_);
        }
        target_info_received_ = true;
        // (Yunwoo) Extract position from detection (in world frame)
        const auto& bbox_center = msg->detections[idx].bbox.center.position;
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