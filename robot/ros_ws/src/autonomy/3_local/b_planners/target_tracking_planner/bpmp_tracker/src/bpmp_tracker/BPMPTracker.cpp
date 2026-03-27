#include "bpmp_tracker/BPMPTracker.h"

namespace bpmp_tracker{
    BPMPTracker::BPMPTracker() : Node("bpmp_tracker"){
        initialize();
    }
    
    BPMPTracker::~BPMPTracker(){
        // (Yunwoo) Destructor
    }
    void BPMPTracker::initialize(){
        // VISUALIZATION PARAMETERS
        UpdateVisualizationParams();
        // PLANNING PARAMETERS
        UpdatePlanningParams();


        // SUBSCRIBERS
        ego_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_1/odometry_conversion/odometry", 5, 
            std::bind(&BPMPTracker::EgoOdometryCallback, this, std::placeholders::_1));
        target_info_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_1/target_info", 5, 
            std::bind(&BPMPTracker::TargetInfoCallback, this, std::placeholders::_1));
        vdb_map_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/robot_1/bpmp/filtered_vdb_map", 5, 
            std::bind(&BPMPTracker::VDBMapCallback, this, std::placeholders::_1));

        // PUBLISHERS
        raw_primitives_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("raw_primitives", 5);
        feasible_primitives_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("feasible_primitives", 5);
        corridor_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("corridor", 5);
        best_primitive_publisher_ = this->create_publisher<nav_msgs::msg::Path>("best_primitive", 5);
        desired_traj_publisher_ = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("/robot_1/trajectory_controller/trajectory_override", 5);
        
        // (Yunwoo) SERVICE SERVER for target_tracking_toggle
        target_tracking_toggle_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "target_tracking_toggle",
            std::bind(&BPMPTracker::TargetTrackingToggleCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(4000), std::bind(&BPMPTracker::Run, this));
    }

    void BPMPTracker::UpdateVisualizationParams(){
        // (Yunwoo) DECLARE ALL PARAMETERS FIRST
        this->declare_parameter<std::string>("frame_id", "world");
        // Raw primitives parameters
        this->declare_parameter<bool>("raw_primitives.publish", false);
        this->declare_parameter<int>("raw_primitives.num_time_sample", 10);
        this->declare_parameter<double>("raw_primitives.proportion", 0.1);
        this->declare_parameter<double>("raw_primitives.line_scale", 0.01);
        this->declare_parameter<double>("raw_primitives.color_a", 0.1);
        this->declare_parameter<double>("raw_primitives.color_r", 0.0);
        this->declare_parameter<double>("raw_primitives.color_g", 0.0);
        this->declare_parameter<double>("raw_primitives.color_b", 0.0);
        // Feasible primitives parameters
        this->declare_parameter<bool>("feasible_primitives.publish", true);
        this->declare_parameter<int>("feasible_primitives.num_time_sample", 10);
        this->declare_parameter<double>("feasible_primitives.proportion", 0.2);
        this->declare_parameter<double>("feasible_primitives.line_scale", 0.02);
        this->declare_parameter<double>("feasible_primitives.color_a", 0.3);
        this->declare_parameter<double>("feasible_primitives.color_r", 0.0);
        this->declare_parameter<double>("feasible_primitives.color_g", 1.0);
        this->declare_parameter<double>("feasible_primitives.color_b", 1.0);
        // Best primitive parameters
        this->declare_parameter<bool>("best_primitive.publish", true);
        this->declare_parameter<int>("best_primitive.num_time_sample", 10);
        this->declare_parameter<double>("best_primitive.line_scale", 0.05);
        this->declare_parameter<double>("best_primitive.color_a", 1.0);
        this->declare_parameter<double>("best_primitive.color_r", 0.0);
        this->declare_parameter<double>("best_primitive.color_g", 0.0);
        this->declare_parameter<double>("best_primitive.color_b", 1.0);
        // Corridor parameters
        this->declare_parameter<bool>("corridor.publish", false);
        this->declare_parameter<double>("corridor.color_a", 0.5);
        this->declare_parameter<double>("corridor.color_r", 1.0);
        this->declare_parameter<double>("corridor.color_g", 0.0);
        this->declare_parameter<double>("corridor.color_b", 1.0);

        // (Yunwoo) GET PARAMETERS
        vis_param_.frame_id = this->get_parameter("frame_id").as_string();
        // Raw primitive (singular to match struct)
        vis_param_.raw_primitive.publish = this->get_parameter("raw_primitives.publish").as_bool();
        vis_param_.raw_primitive.num_time_sample = this->get_parameter("raw_primitives.num_time_sample").as_int();
        vis_param_.raw_primitive.proportion = this->get_parameter("raw_primitives.proportion").as_double();
        vis_param_.raw_primitive.line_scale = this->get_parameter("raw_primitives.line_scale").as_double();
        vis_param_.raw_primitive.color_a = this->get_parameter("raw_primitives.color_a").as_double();
        vis_param_.raw_primitive.color_r = this->get_parameter("raw_primitives.color_r").as_double();
        vis_param_.raw_primitive.color_g = this->get_parameter("raw_primitives.color_g").as_double();
        vis_param_.raw_primitive.color_b = this->get_parameter("raw_primitives.color_b").as_double();
        // Feasible primitive (singular to match struct)
        vis_param_.feasible_primitive.publish = this->get_parameter("feasible_primitives.publish").as_bool();
        vis_param_.feasible_primitive.num_time_sample = this->get_parameter("feasible_primitives.num_time_sample").as_int();
        vis_param_.feasible_primitive.proportion = this->get_parameter("feasible_primitives.proportion").as_double();
        vis_param_.feasible_primitive.line_scale = this->get_parameter("feasible_primitives.line_scale").as_double();
        vis_param_.feasible_primitive.color_a = this->get_parameter("feasible_primitives.color_a").as_double();
        vis_param_.feasible_primitive.color_r = this->get_parameter("feasible_primitives.color_r").as_double();
        vis_param_.feasible_primitive.color_g = this->get_parameter("feasible_primitives.color_g").as_double();
        vis_param_.feasible_primitive.color_b = this->get_parameter("feasible_primitives.color_b").as_double();
        // Best primitive
        vis_param_.best_primitive.publish = this->get_parameter("best_primitive.publish").as_bool();
        vis_param_.best_primitive.num_time_sample = this->get_parameter("best_primitive.num_time_sample").as_int();
        vis_param_.best_primitive.line_scale = this->get_parameter("best_primitive.line_scale").as_double();
        vis_param_.best_primitive.color_a = this->get_parameter("best_primitive.color_a").as_double();
        vis_param_.best_primitive.color_r = this->get_parameter("best_primitive.color_r").as_double();
        vis_param_.best_primitive.color_g = this->get_parameter("best_primitive.color_g").as_double();
        vis_param_.best_primitive.color_b = this->get_parameter("best_primitive.color_b").as_double();
        // Corridor
        vis_param_.corridor.publish = this->get_parameter("corridor.publish").as_bool();
        vis_param_.corridor.color_a = this->get_parameter("corridor.color_a").as_double();
        vis_param_.corridor.color_r = this->get_parameter("corridor.color_r").as_double();
        vis_param_.corridor.color_g = this->get_parameter("corridor.color_g").as_double();
        vis_param_.corridor.color_b = this->get_parameter("corridor.color_b").as_double();

        // Update visualizer with all parameters
        visualizer_.UpdateParams(vis_param_);
    }

    void BPMPTracker::UpdatePlanningParams(){
        // (Yunwoo) GET PARAMETERS
        // Use double to handle YAML type flexibility, then cast to int
        this->declare_parameter<double>("replanning_time", 2000.0);
        planning_params_.replanning_time = static_cast<int>(this->get_parameter("replanning_time").as_double());
    }

    void BPMPTracker::Run(){
        if(tracking_enabled_){ // TODO: tracking_enabled_
            bool success_flag = Replan();
            Publish(success_flag);
        }
    }

    bool BPMPTracker::Replan(){
        // TODO: Implement replanning
        if(current_mode_ == TrackerMode::WAITING_FOR_ODOMETRY || current_mode_ == TrackerMode::WAITING_FOR_TARGET){
            return false;
        }
        SampleEndPoint(); // Sample end points
        GeneratePrimitive(); // Generate primitives
        GetSafeIndex(); // Get safe index
        if(safe_index_.empty())
            return false;
        GetBestIndex(); // Get best index
        return true;
    }

    void BPMPTracker::GetBestIndex(){
        int num_chunk = safe_index_.size() / num_threads_;
        vector<thread> worker_threads;
        vector<std::pair<uint,double>> best_index_temp(num_threads_);
        for(int i = 0; i < num_threads_; i++){
            worker_threads.emplace_back(&BPMPTracker::GetBestIndexThread, this, num_chunk * i, num_chunk * (i + 1), std::ref(best_index_temp[i]));
        }
        for(int i =0;i<num_threads_;i++){
            worker_threads[i].join();
        }
        double min_score = 999999999999999999999.0;
        for(int i =0;i<num_threads_;i++){
            if(best_index_temp[i].second < min_score){
                min_score = best_index_temp[i].second;
                best_index_ = best_index_temp[i].first;
            }
        }
    }
    void BPMPTracker::GetBestIndexThread(const int &start_idx, const int &end_idx, std::pair<uint,double> &score_pair){
        double min_score = 999999999999999999999.0;
        double accumulated_score = 0.0;
        uint min_accumulated_idx = -1;
        for(int i = start_idx; i < end_idx; i++){
            accumulated_score = 0.0;
            for(int j = start_idx; j < end_idx; j++){
                accumulated_score += std::abs(primitives_[safe_index_[i]].ctrl_x[3] - primitives_[safe_index_[j]].ctrl_x[3]);
                accumulated_score += std::abs(primitives_[safe_index_[i]].ctrl_y[3] - primitives_[safe_index_[j]].ctrl_y[3]);
                accumulated_score += std::abs(primitives_[safe_index_[i]].ctrl_z[3] - primitives_[safe_index_[j]].ctrl_z[3]);
            }
            if(accumulated_score < min_score){
                min_score = accumulated_score;
                min_accumulated_idx = safe_index_[i];
            }
        }
        score_pair.first = min_accumulated_idx;
        score_pair.second = min_score;
    }
    void BPMPTracker::SampleEndPoint(){
        // TODO: Implement endpoint sampling
        end_points_.clear();
        int num_chunk = num_samples_ / num_threads_;
        vector<thread> worker_threads;
        vector<vector<bpmp_tracker::Point>> end_point_temp(num_threads_);
        for(int i = 0; i < num_threads_; i++){
            worker_threads.emplace_back(&BPMPTracker::SampleEndPointThread, this, num_chunk * i, num_chunk * (i + 1), std::ref(end_point_temp[i]));
        }
        for(int i =0;i<num_threads_;i++){
            worker_threads[i].join();
        }
        for(int i =0;i<num_threads_;i++){
            for(const auto& end_point : end_point_temp[i]){
                end_points_.push_back(end_point);
            }
        }
    }
    void BPMPTracker::SampleEndPointThread(const int & start_idx, const int & end_idx, vector<bpmp_tracker::Point>& endpoint_list_sub){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> r_dis(2.0, 3.0);
        double center_angle = std::atan2(current_odometry_.pose.pose.position.y-current_target_info_.pose.position.y, current_odometry_.pose.pose.position.x-current_target_info_.pose.position.x);
        double half_angle = M_PI/3.0;
        std::uniform_real_distribution<> theta_dis(center_angle - half_angle, center_angle + half_angle);
        std::uniform_real_distribution<> z_dis(current_target_info_.pose.position.z+0.5, current_target_info_.pose.position.z + 1.0);
        Point temp_point{0.0, 0.0, 0.0};
        double r, theta, z;
        for(int i = start_idx; i < end_idx; i++){
            r = r_dis(gen);
            theta = theta_dis(gen);
            z = z_dis(gen);
            temp_point.x = current_target_info_.pose.position.x + r * std::cos(theta);
            temp_point.y = current_target_info_.pose.position.y + r * std::sin(theta);
            temp_point.z = z;
            endpoint_list_sub.push_back(temp_point);
        }
    }
    void BPMPTracker::GetSafeIndex(){
        safe_index_.clear();
        GenerateCorridor();
        int num_chunk = num_samples_ / num_threads_;
        vector<thread> worker_threads;
        vector<vector<bpmp_tracker::uint>> safe_index_temp(num_threads_);
        for(int i = 0; i < num_threads_; i++){
            worker_threads.emplace_back(&BPMPTracker::GetSafeIndexThread, this, num_chunk * i, num_chunk * (i + 1), std::ref(safe_index_temp[i]));
        }
        for(int i =0;i<num_threads_;i++){
            worker_threads[i].join();
        }
        for(int i =0;i<num_threads_;i++){
            for(const auto& safe_index : safe_index_temp[i]){
                safe_index_.push_back(safe_index);
            }
        }
    }
    void BPMPTracker::GetSafeIndexThread(const int &start_idx, const int &end_idx, vector<bpmp_tracker::uint> &safe_idx_sub){
        Eigen::Vector3d A_comp_temp{0.0, 0.0, 0.0};
        double b_comp_temp(0.0);
        vector<Eigen::Vector3d> LinearConstraintA;
        vector<double> LinearConstraintb;
        int num_constraint = corridor_constraints_[0].A().rows();
        int num_var = corridor_constraints_[0].A().cols();
        double value_sfc;
        for (int i = 0; i < num_constraint; i++) {
            A_comp_temp[0] = corridor_constraints_[0].A().coeffRef(i, 0);
            A_comp_temp[1] = corridor_constraints_[0].A().coeffRef(i, 1);
            A_comp_temp[2] = corridor_constraints_[0].A().coeffRef(i, 2);
            b_comp_temp = corridor_constraints_[0].b().coeffRef(i, 0);
            LinearConstraintA.push_back(A_comp_temp);
            LinearConstraintb.push_back(b_comp_temp);
        }
        bool flag_store_in1 = true;
        for(int idx = start_idx; idx < end_idx; idx++){
            flag_store_in1 = true;
            for(int i = 0; i<LinearConstraintA.size(); i++){
                for (int j =0;j<4;j++){
                    value_sfc = LinearConstraintA[i][0] * (primitives_[idx].ctrl_x[j]) +
                            LinearConstraintA[i][1] * (primitives_[idx].ctrl_y[j]) +
                            LinearConstraintA[i][2] * (primitives_[idx].ctrl_z[j]) +
                            - LinearConstraintb[i];
                    if (value_sfc > 0.0) {
                        flag_store_in1 = false;
                        break;
                    }
                }
                if(not flag_store_in1){
                    break;
                }
            }
            if(flag_store_in1){
                safe_idx_sub.push_back(idx);
            }
        }
    }
    void BPMPTracker::GeneratePrimitive(){
        // TODO: Implement primitive generation
        primitives_.clear();
        int num_chunk = num_samples_ / num_threads_;
        vector<thread> worker_threads;
        vector<vector<bpmp_tracker::PrimitivePlanning>> primitive_temp(num_threads_);
        for(int i = 0; i < num_threads_; i++){
            worker_threads.emplace_back(&BPMPTracker::GeneratePrimitiveThread, this, num_chunk * i, num_chunk * (i + 1), std::ref(primitive_temp[i]));
        }
        for(int i =0;i<num_threads_;i++){
            worker_threads[i].join();
        }
        for(int i =0;i<num_threads_;i++){
            for(const auto& primitive : primitive_temp[i]){
                primitives_.push_back(primitive);
            }
        }
    }
    void BPMPTracker::GeneratePrimitiveThread(const int &start_idx, const int &end_idx, vector<bpmp_tracker::PrimitivePlanning> & primitive_list_sub){
        // TODO: Implement primitive generation
        bpmp_tracker::PrimitivePlanning temp_primitive;
        temp_primitive.t0 = 0.0;
        temp_primitive.tf = 1.0;       
        for(int i = start_idx; i < end_idx; i++){
            temp_primitive.ctrl_x[0] = current_odometry_.pose.pose.position.x;
            temp_primitive.ctrl_y[0] = current_odometry_.pose.pose.position.y;
            temp_primitive.ctrl_z[0] = current_odometry_.pose.pose.position.z;
            temp_primitive.ctrl_x[1] = 0.6666666666666666 * current_odometry_.pose.pose.position.x + 0.3333333333333333 * end_points_[i].x;
            temp_primitive.ctrl_y[1] = 0.6666666666666666 * current_odometry_.pose.pose.position.y + 0.3333333333333333 * end_points_[i].y;
            temp_primitive.ctrl_z[1] = 0.6666666666666666 * current_odometry_.pose.pose.position.z + 0.3333333333333333 * end_points_[i].z;
            temp_primitive.ctrl_x[2] = 0.3333333333333333 * current_odometry_.pose.pose.position.x + 0.6666666666666666 * end_points_[i].x;
            temp_primitive.ctrl_y[2] = 0.3333333333333333 * current_odometry_.pose.pose.position.y + 0.6666666666666666 * end_points_[i].y;
            temp_primitive.ctrl_z[2] = 0.3333333333333333 * current_odometry_.pose.pose.position.z + 0.6666666666666666 * end_points_[i].z;
            temp_primitive.ctrl_x[3] = end_points_[i].x;
            temp_primitive.ctrl_y[3] = end_points_[i].y;
            temp_primitive.ctrl_z[3] = end_points_[i].z;
            primitive_list_sub.push_back(temp_primitive);
        }
    }
    
    void BPMPTracker::GenerateCorridor(){
        polys_.clear();
        corridor_constraints_.clear();
        Eigen::Matrix<double,3,1> target_position;
        target_position << current_target_info_.pose.position.x, current_target_info_.pose.position.y, 1.0; //(TODO: Get target height from target info)
        Eigen::Matrix<double,3,1> robot_position;
        robot_position << current_odometry_.pose.pose.position.x, current_odometry_.pose.pose.position.y, current_odometry_.pose.pose.position.z;
        vec_Vec3f segment;
        segment.push_back(target_position);
        segment.push_back(robot_position);
        EllipsoidDecomp3D decomp_util;
        decomp_util.set_obs(point_cloud_3d_);
        decomp_util.set_local_bbox(Vec3f(5.0,5.0,2.0));
        decomp_util.dilate(segment);
        vec_E<Polyhedron3D> polys;
        auto poly_hedrons = decomp_util.get_polyhedrons();
        polys_.push_back(poly_hedrons[0]);
        LinearConstraint3D corridor_constraint(0.5 * (robot_position + target_position), poly_hedrons[0].hyperplanes());
        corridor_constraints_.push_back(corridor_constraint);

        // (Yunwoo) Convert LinearConstraint3D to vector<AffineCoeff3D> for visualization
        // LinearConstraint3D: Ax < b
        // AffineCoeff3D: ax + by + cz + d < 0, so d = -b
        vector<AffineCoeff3D> affine_constraints;
        for (int i = 0; i < corridor_constraint.A().rows(); ++i) {
            AffineCoeff3D affine;
            affine.head<3>() = corridor_constraint.A().row(i).transpose();
            affine(3) = -corridor_constraint.b()(i);
            affine_constraints.push_back(affine);
        }

        // (Yunwoo) Visualize and publish corridor
        auto corridor_marker = visualizer_.VisualizeCorridor(affine_constraints);
        corridor_publisher_->publish(corridor_marker);
    }

    void BPMPTracker::Publish(const bool &success_flag){
        // TODO: Implement publishing
        if(success_flag){
            // raw_primitives_publisher_->publish(visualizer_.VisualizeRawPrimitives(primitives_));
            feasible_primitives_publisher_->publish(visualizer_.VisualizeFeasiblePrimitives(primitives_, safe_index_));
            {// (Yunwoo) Publish best primitive
                nav_msgs::msg::Path best_primitive;
                best_primitive.header.stamp = this->now();
                best_primitive.header.frame_id = "map";
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = primitives_[best_index_].ctrl_x[3];
                pose.pose.position.y = primitives_[best_index_].ctrl_y[3];
                pose.pose.position.z = primitives_[best_index_].ctrl_z[3];
                best_primitive.poses.push_back(pose);   // (Yunwoo) Add end point
                pose.pose.position.x = primitives_[best_index_].ctrl_x[0];
                pose.pose.position.y = primitives_[best_index_].ctrl_y[0];
                pose.pose.position.z = primitives_[best_index_].ctrl_z[0];
                best_primitive.poses.push_back(pose);   // (Yunwoo) Add start point
                best_primitive_publisher_->publish(best_primitive);
                // (Yunwoo) Publish desired trajectory
                airstack_msgs::msg::TrajectoryXYZVYaw desired_traj;
                desired_traj.header.stamp = this->now();
                desired_traj.header.frame_id = "map";
                airstack_msgs::msg::WaypointXYZVYaw wp1, wp2;
                wp1.position.x = current_odometry_.pose.pose.position.x;
                wp1.position.y = current_odometry_.pose.pose.position.y;
                wp1.position.z = current_odometry_.pose.pose.position.z;
                tf2::Quaternion q1(current_odometry_.pose.pose.orientation.x,current_odometry_.pose.pose.orientation.y,
                    current_odometry_.pose.pose.orientation.z,current_odometry_.pose.pose.orientation.w);
                double roll1, pitch1, yaw1;
                tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
                wp1.velocity = 1.0;
                wp1.yaw = yaw1;

                wp2.position.x = primitives_[best_index_].ctrl_x[3];
                wp2.position.y = primitives_[best_index_].ctrl_y[3];
                wp2.position.z = primitives_[best_index_].ctrl_z[3];
                wp2.velocity = 0.5;
                wp2.yaw = std::atan2(current_target_info_.pose.position.y- primitives_[best_index_].ctrl_y[3],
                    current_target_info_.pose.position.x - primitives_[best_index_].ctrl_x[3]);
                wp2.yaw = std::fmod(wp2.yaw + M_PI, 2.0 * M_PI) - M_PI;
                desired_traj.waypoints.push_back(wp1);
                desired_traj.waypoints.push_back(wp2);
                desired_traj_publisher_->publish(desired_traj);
            }
            // best_primitive_publisher_->publish(visualizer_.VisualizeBestPrimitive(primitives_, best_index_));
        }else{
            RCLCPP_ERROR(this->get_logger(), "Replanning failed");
        }
    }

    // (Yunwoo) Odometry callback
    void BPMPTracker::EgoOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        current_odometry_ = *msg;
        odometry_received_ = true;
        UpdateMode();
    }

    // (Yunwoo) Target info callback
    void BPMPTracker::TargetInfoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        current_target_info_ = *msg;
        target_info_received_ = true;
        UpdateMode();
    }
    void BPMPTracker::VDBMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg){
        vdb_map_received_ = true;
        point_cloud_3d_.clear();
        for(const auto& point : msg->points){
            point_cloud_3d_.push_back(Vec3(point.x, point.y, point.z));
        }
        // UpdatePointCloud3D();
    }
    // (Yunwoo) Update state machine mode
    void BPMPTracker::UpdateMode(){
        TrackerMode new_mode;
        
        if (!odometry_received_) {
            // Mode 0: robot_odometry not received
            new_mode = TrackerMode::WAITING_FOR_ODOMETRY;
        } else if (odometry_received_ && target_info_received_) {
            // Mode 1: both target_info and robot_odometry received
            new_mode = TrackerMode::TRACKING;
        } else {
            // Mode 2: robot_odometry received, but target_info not received
            new_mode = TrackerMode::WAITING_FOR_TARGET;
        }

        // Log mode change
        if (new_mode != current_mode_) {
            current_mode_ = new_mode;
            RCLCPP_INFO(this->get_logger(), "Tracker mode changed to: %d", static_cast<int>(current_mode_));
        }
    }

    // (Yunwoo) Service callback for target_tracking_toggle
    void BPMPTracker::TargetTrackingToggleCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        tracking_enabled_ = !tracking_enabled_;
        
        if (tracking_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Target tracking ENABLED");
            response->success = true;
            response->message = "Target tracking enabled";
        } else {
            RCLCPP_INFO(this->get_logger(), "Target tracking DISABLED");
            response->success = true;
            response->message = "Target tracking disabled";
        }
    }

}