// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <behavior_executive/behavior_executive.hpp>

BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive"), modelPath("/home/robot/AirStack/robot/ros_ws/models/yolo11s.onnx"),
      labelsPath("/home/robot/AirStack/robot/ros_ws/models/coco.names"),
      isGPU(false),
      detector(modelPath, labelsPath, isGPU)
{
    // conditions
    auto_takeoff_commanded_condition = new bt::Condition("Auto Takeoff Commanded", this);
    takeoff_commanded_condition = new bt::Condition("Takeoff Commanded", this);
    armed_condition = new bt::Condition("Armed", this);
    offboard_mode_condition = new bt::Condition("Offboard Mode", this);
    stationary_condition = new bt::Condition("Stationary", this);
    land_commanded_condition = new bt::Condition("Land Commanded", this);
    pause_commanded_condition = new bt::Condition("Pause Commanded", this);
    rewind_commanded_condition = new bt::Condition("Rewind Commanded", this);
    fixed_trajectory_condition = new bt::Condition("Fixed Trajectory Commanded", this);
    global_plan_condition = new bt::Condition("Global Plan Commanded", this);
    offboard_commanded_condition = new bt::Condition("Offboard Commanded", this);
    arm_commanded_condition = new bt::Condition("Arm Commanded", this);
    disarm_commanded_condition = new bt::Condition("Disarm Commanded", this);
    takeoff_complete_condition = new bt::Condition("Takeoff Complete", this);
    landing_complete_condition = new bt::Condition("Landing Complete", this);
    in_air_condition = new bt::Condition("In Air", this);
    state_estimate_timed_out_condition = new bt::Condition("State Estimate Timed Out", this);
    stuck_condition = new bt::Condition("Stuck", this);
    autonomously_explore_condition = new bt::Condition("Autonomously Explore Commanded", this);
    image_based_visual_servo_condition = new bt::Condition("Image Based Visual Servoing", this);
    conditions.push_back(auto_takeoff_commanded_condition);
    conditions.push_back(takeoff_commanded_condition);
    conditions.push_back(armed_condition);
    conditions.push_back(offboard_mode_condition);
    conditions.push_back(stationary_condition);
    conditions.push_back(land_commanded_condition);
    conditions.push_back(pause_commanded_condition);
    conditions.push_back(rewind_commanded_condition);
    conditions.push_back(fixed_trajectory_condition);
    conditions.push_back(global_plan_condition);
    conditions.push_back(offboard_commanded_condition);
    conditions.push_back(arm_commanded_condition);
    conditions.push_back(disarm_commanded_condition);
    conditions.push_back(takeoff_complete_condition);
    conditions.push_back(landing_complete_condition);
    conditions.push_back(in_air_condition);
    conditions.push_back(state_estimate_timed_out_condition);
    conditions.push_back(stuck_condition);
    conditions.push_back(autonomously_explore_condition);
    conditions.push_back(image_based_visual_servo_condition);

    // actions
    arm_action = new bt::Action("Arm", this);
    takeoff_action = new bt::Action("Takeoff", this);
    land_action = new bt::Action("Land", this);
    pause_action = new bt::Action("Pause", this);
    rewind_action = new bt::Action("Rewind", this);
    follow_fixed_trajectory_action = new bt::Action("Follow Fixed Trajectory", this);
    global_plan_action = new bt::Action("Follow Global Plan", this);
    request_control_action = new bt::Action("Request Control", this);
    disarm_action = new bt::Action("Disarm", this);
    ibvs_action = new bt::Action("IBVS", this);
    actions.push_back(arm_action);
    actions.push_back(takeoff_action);
    actions.push_back(land_action);
    actions.push_back(pause_action);
    actions.push_back(rewind_action);
    actions.push_back(follow_fixed_trajectory_action);
    actions.push_back(global_plan_action);
    actions.push_back(request_control_action);
    actions.push_back(disarm_action);
    actions.push_back(ibvs_action);

    // subscribers
    behavior_tree_commands_sub =
        this->create_subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>(
            "behavior_tree_commands", 1,
            std::bind(&BehaviorExecutive::bt_commands_callback, this, std::placeholders::_1));
    is_armed_sub = this->create_subscription<std_msgs::msg::Bool>(
        "is_armed", 1,
        std::bind(&BehaviorExecutive::is_armed_callback, this, std::placeholders::_1));
    has_control_sub = this->create_subscription<std_msgs::msg::Bool>(
        "has_control", 1,
        std::bind(&BehaviorExecutive::has_control_callback, this, std::placeholders::_1));
    takeoff_state_sub = this->create_subscription<std_msgs::msg::String>("takeoff_state", 1,
									 std::bind(&BehaviorExecutive::takeoff_state_callback,
										   this, std::placeholders::_1));
    landing_state_sub = this->create_subscription<std_msgs::msg::String>("landing_state", 1,
									 std::bind(&BehaviorExecutive::landing_state_callback,
										   this, std::placeholders::_1));
    state_estimate_timed_out_sub =
      this->create_subscription<std_msgs::msg::Bool>("state_estimate_timed_out", 1,
						     std::bind(&BehaviorExecutive::state_estimate_timed_out_callback,
							       this, std::placeholders::_1));
    stuck_sub =
        this->create_subscription<std_msgs::msg::Bool>("stuck", 1,
                                                       std::bind(&BehaviorExecutive::stuck_callback,
                                                                 this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/robot_1/sensors/front_stereo/left/camera_info", 10,
            std::bind(&BehaviorExecutive::cameraInfoCallback, this, std::placeholders::_1));

    // Depth image subscriber
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot_1/sensors/front_stereo/left/depth_ground_truth", 10,
            std::bind(&BehaviorExecutive::depthCallback, this, std::placeholders::_1));

    // Image subscriber
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot_1/sensors/front_stereo/left/image_rect", 10,
            std::bind(&BehaviorExecutive::imageCallback, this, std::placeholders::_1));
    
    // publishers
    recording_pub = this->create_publisher<std_msgs::msg::Bool>("set_recording_status", 1);
    reset_stuck_pub = this->create_publisher<std_msgs::msg::Empty>("reset_stuck", 1);
    clear_map_pub = this->create_publisher<std_msgs::msg::Empty>("clear_map", 1);
    robot_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robot_1/interface/cmd_velocity", 10);

    // services
    service_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    robot_command_client = this->create_client<airstack_msgs::srv::RobotCommand>(
        "robot_command", rmw_qos_profile_services_default, service_callback_group);
    trajectory_mode_client = this->create_client<airstack_msgs::srv::TrajectoryMode>(
        "set_trajectory_mode", rmw_qos_profile_services_default, service_callback_group);
    takeoff_landing_command_client = this->create_client<airstack_msgs::srv::TakeoffLandingCommand>(
        "set_takeoff_landing_command", rmw_qos_profile_services_default, service_callback_group);
    global_planner_toggle_client = this->create_client<std_srvs::srv::Trigger>(
        "global_plan_toggle", rmw_qos_profile_services_default, service_callback_group);

    // timers
    timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1. / 20.),
                                 std::bind(&BehaviorExecutive::timer_callback, this));
}

void BehaviorExecutive::timer_callback() {
    if (request_control_action->is_active()) {
        if (request_control_action->active_has_changed()) {
            airstack_msgs::srv::RobotCommand::Request::SharedPtr request =
                std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
            request->command = airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL;

            auto result = robot_command_client->async_send_request(request);
            std::cout << "waiting be rc" << std::endl;
            result.wait();
            std::cout << "done be rc" << std::endl;
            if (result.get()->success)
                request_control_action->set_success();
            else
                request_control_action->set_failure();
        }
    }

    if (arm_action->is_active()) {
        if (arm_action->active_has_changed()) {
	    std_msgs::msg::Bool start_msg;
	    start_msg.data = true;
	    recording_pub->publish(start_msg);
	
            airstack_msgs::srv::RobotCommand::Request::SharedPtr request =
                std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
            request->command = airstack_msgs::srv::RobotCommand::Request::ARM;

            auto result = robot_command_client->async_send_request(request);
            std::cout << "waiting be arm" << std::endl;
            result.wait();
            std::cout << "done be arm" << std::endl;
            if (result.get()->success)
                arm_action->set_success();
            else
                arm_action->set_failure();
        }
    }

    if (disarm_action->is_active()) {
        if (disarm_action->active_has_changed()) {
	    std_msgs::msg::Bool stop_msg;
	    stop_msg.data = false;
	    recording_pub->publish(stop_msg);
	
	    in_air_condition->set(false);
            airstack_msgs::srv::RobotCommand::Request::SharedPtr request =
                std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
            request->command = airstack_msgs::srv::RobotCommand::Request::DISARM;

            auto result = robot_command_client->async_send_request(request);
            std::cout << "waiting be arm" << std::endl;
            result.wait();
            std::cout << "done be arm" << std::endl;
            if (result.get()->success)
                disarm_action->set_success();
            else
                disarm_action->set_failure();
        }
    }

    if (takeoff_action->is_active()) {
        // std::cout << "takeoff" << std::endl;
        takeoff_action->set_running();
	in_air_condition->set(true);
	reset_stuck_pub->publish(std_msgs::msg::Empty());
	
        if (takeoff_action->active_has_changed()) {
	    // clear the local planner's map
	    clear_map_pub->publish(std_msgs::msg::Empty());
	
            // put trajectory controller in track mode
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            std::cout << "mode 1" << std::endl;
            mode_result.wait();
            std::cout << "mode 2" << std::endl;

            if (mode_result.get()->success) {
                // send a takeoff command to ardupilot
                // TODO clean up variable names
                airstack_msgs::srv::RobotCommand::Request::SharedPtr request =
                    std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
                request->command = airstack_msgs::srv::RobotCommand::Request::TAKEOFF;

                auto result = robot_command_client->async_send_request(request);
                std::cout << "waiting robot command takeoff" << std::endl;
                result.wait();
                std::cout << "done robot command takeoff" << std::endl;

                // send the takeoff trajectory
                if (result.get()->success) {
                    airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr takeoff_request =
                        std::make_shared<airstack_msgs::srv::TakeoffLandingCommand::Request>();
                    takeoff_request->command =
                        airstack_msgs::srv::TakeoffLandingCommand::Request::TAKEOFF;
                    auto takeoff_result =
                        takeoff_landing_command_client->async_send_request(takeoff_request);
                    std::cout << "takeoff 1" << std::endl;
                    takeoff_result.wait();
                    std::cout << "takeoff 2" << std::endl;
                    if (!takeoff_result.get()->accepted)
                        takeoff_action->set_failure();
                } else
                    takeoff_action->set_failure();
            } else
                takeoff_action->set_failure();
        }

	if(takeoff_state == "COMPLETE"){
	  takeoff_complete_condition->set(true);
	  takeoff_action->set_success();
	}
    }

    if (land_action->is_active()) {
        // std::cout << "land" << std::endl;
        land_action->set_running();
        if (land_action->active_has_changed()) {
            // put trajectory controller in track mode
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            std::cout << "mode 1" << std::endl;
            mode_result.wait();
            std::cout << "mode 2" << std::endl;

            if (mode_result.get()->success) {
                airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr land_request =
                    std::make_shared<airstack_msgs::srv::TakeoffLandingCommand::Request>();
                land_request->command = airstack_msgs::srv::TakeoffLandingCommand::Request::LAND;
                auto land_result = takeoff_landing_command_client->async_send_request(land_request);
                std::cout << "land 1" << std::endl;
                land_result.wait();
                std::cout << "land 2" << std::endl;
                if (land_result.get()->accepted)
                    land_action->set_success();
                else
                    land_action->set_failure();
            } else
                land_action->set_failure();
        }
	

	if(landing_state == "COMPLETE"){
	  landing_complete_condition->set(true);
	  land_action->set_success();
	}
    }

    if (pause_action->is_active()) {
        pause_action->set_running();
        if (pause_action->active_has_changed()) {
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::PAUSE;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            mode_result.wait();
        }
    }

    if (rewind_action->is_active()) {
        rewind_action->set_running();
        if (rewind_action->active_has_changed()) {
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::REWIND;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            mode_result.wait();
        }
    }

    // follow fixed trajectory action
    if (follow_fixed_trajectory_action->is_active()) {
        follow_fixed_trajectory_action->set_running();

        if (follow_fixed_trajectory_action->active_has_changed()) {
            // put trajectory controller in track mode
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            std::cout << "mode 1" << std::endl;
            mode_result.wait();
            std::cout << "mode 2" << std::endl;
        }
    }

    if (global_plan_action->is_active()) {
        global_plan_action->set_running();
        if (global_plan_action->active_has_changed()) {
            // put trajectory controller in ADD_SEGMENT mode
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::ADD_SEGMENT;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            std::cout << "mode 1" << std::endl;
            mode_result.wait();
            std::cout << "mode 2" << std::endl;

	    if(global_planner_toggle_client->service_is_ready()){
	        std_srvs::srv::Trigger::Request::SharedPtr request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
	        auto result = global_planner_toggle_client->async_send_request(request);
	        result.wait();
	        if (result.get()->success)
                    global_plan_action->set_success();
	        else
                    global_plan_action->set_failure();
	    }
        };
    }

    if (ibvs_action->is_active()) {
        // std::cout << "land" << std::endl;
        ibvs_action->set_running();

        if (ibvs_action->active_has_changed()) {
            clear_map_pub->publish(std_msgs::msg::Empty());

            // put trajectory controller in track mode
            airstack_msgs::srv::TrajectoryMode::Request::SharedPtr mode_request =
                std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
            mode_request->mode = airstack_msgs::srv::TrajectoryMode::Request::TRACK_VELOCITY;
            auto mode_result = trajectory_mode_client->async_send_request(mode_request);
            std::cout << "mode 1" << std::endl;
            mode_result.wait();
            std::cout << "mode 2" << std::endl;

            if (mode_result.get()->success) {
                // send a takeoff command to ardupilot
                airstack_msgs::srv::RobotCommand::Request::SharedPtr request =
                    std::make_shared<airstack_msgs::srv::RobotCommand::Request>();
                request->command = airstack_msgs::srv::RobotCommand::Request::REQUEST_CONTROL;

                auto result = robot_command_client->async_send_request(request);
                std::cout << "waiting robot command takeoff" << std::endl;
                result.wait();
                std::cout << "done robot command takeoff" << std::endl;

                // send the takeoff trajectory
                if (result.get()->success) {
                    airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr takeoff_request =
                        std::make_shared<airstack_msgs::srv::TakeoffLandingCommand::Request>();
                    takeoff_request->command =
                        airstack_msgs::srv::TakeoffLandingCommand::Request::NONE;
                    auto takeoff_result =
                        takeoff_landing_command_client->async_send_request(takeoff_request);
                    std::cout << "takeoff 1" << std::endl;
                    takeoff_result.wait();
                    std::cout << "takeoff 2" << std::endl;
                    if (!takeoff_result.get()->accepted)
                        ibvs_action->set_failure();
                    else {
                            ibvs_running_ = true;

                            ibvs_timer_ = this->create_wall_timer(
                                std::chrono::milliseconds(50),  // 20 Hz
                                [this]() {
                                    if (!ibvs_running_) return;
                                    ibvs_control(det_x_, det_y_, det_z_);
                                }
                            );
                    }

                }
                else
                    ibvs_action->set_failure();
            }
            else
                ibvs_action->set_failure();
        }
    }

    for (bt::Condition *condition : conditions)
        condition->publish();
    for (bt::Action *action : actions)
        action->publish();
}

// callbacks

void BehaviorExecutive::bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg) {
    for (size_t i = 0; i < msg.commands.size(); i++) {
        std::string condition_name = msg.commands[i].condition_name;
        int status = msg.commands[i].status;

        for (size_t j = 0; j < conditions.size(); j++) {
            bt::Condition* condition = conditions[j];
            if (condition_name == condition->get_label()) {
                if (status == behavior_tree_msgs::msg::Status::SUCCESS)
                    condition->set(true);
                else if (status == behavior_tree_msgs::msg::Status::FAILURE)
                    condition->set(false);
            }
        }
    }
}

void BehaviorExecutive::is_armed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    armed_condition->set(msg->data);
}

void BehaviorExecutive::has_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    offboard_mode_condition->set(msg->data);
}


void BehaviorExecutive::takeoff_state_callback(const std_msgs::msg::String::SharedPtr msg){
  takeoff_state = msg->data;
}

void BehaviorExecutive::landing_state_callback(const std_msgs::msg::String::SharedPtr msg){
  landing_state = msg->data;
}

void BehaviorExecutive::state_estimate_timed_out_callback(const std_msgs::msg::Bool::SharedPtr msg){
  state_estimate_timed_out_condition->set(msg->data);
}

void BehaviorExecutive::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
}

void BehaviorExecutive::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat depth_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_depth_ = depth_image;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void BehaviorExecutive::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    cv::Mat image;
    cv::Mat image_raw;

    // Convert ROS2 image → cv::Mat
    try
    {
        image_raw = cv_bridge::toCvShare(msg, msg->encoding)->image.clone();
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // convert to BGR, from all possible formats
    if (msg->encoding == "rgb8")
    {
        cv::cvtColor(image_raw, image, cv::COLOR_RGB2BGR);
    }
    else if (msg->encoding == "rgba8")
    {
        cv::cvtColor(image_raw, image, cv::COLOR_RGBA2BGR);
    }
    else if (msg->encoding == "bgra8")
    {
        cv::cvtColor(image_raw, image, cv::COLOR_BGRA2BGR);
    }
    else
    {
        image = image_raw;
    }

    double center_x = -1;
    double center_y = -1;

    std::vector<Detection> detections = detector.detect(image);

    if (!detections.empty()) {
        const Detection &d = detections[0];

        center_x = d.box.x + d.box.width * 0.5;
        center_y = d.box.y + d.box.height * 0.5;
    }
    

    float detected_z = std::numeric_limits<float>::quiet_NaN();

    {
        std::lock_guard<std::mutex> lock(depth_mutex_);

        if (!latest_depth_.empty() &&
            center_x >= 0 && center_y >= 0 &&
            center_x < latest_depth_.cols &&
            center_y < latest_depth_.rows)
        {
            // depth conversion
            if (latest_depth_.type() == CV_32FC1) {
                detected_z = latest_depth_.at<float>((int)center_y, (int)center_x);
            } else if (latest_depth_.type() == CV_16UC1) {
                detected_z = latest_depth_.at<uint16_t>((int)center_y, (int)center_x)/1000.0f;
            }
        }
    }

    det_x_ = std::nan("");
    det_y_ = std::nan("");
    det_z_ = std::nan("");

    // Safety check
    if (detected_z >= 0.0 && detected_z <= 15.0) {
        det_x_ = center_x;
        det_y_ = center_y;
        det_z_ = detected_z;
    }
}

void BehaviorExecutive::stuck_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    stuck_condition->set(msg->data);
}

void BehaviorExecutive::publish_velocity_ibvs(double vx, double vy, double vz)
  {
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.frame_id = "/base_link_stabilized";
    vel_msg.header.stamp = this->now();

    vel_msg.twist.linear.x = vx;
    vel_msg.twist.linear.y = vy;
    vel_msg.twist.linear.z = vz;

    vel_msg.twist.angular.x = 0.0;
    vel_msg.twist.angular.y = 0.0;
    vel_msg.twist.angular.z = 0.0;

    std_msgs::msg::Bool mute_msg;
    mute_msg.data = true;

    velocity_pub_->publish(vel_msg);
  }

Eigen::Vector3d BehaviorExecutive::camera_to_body(double cx, double cy, double cz)
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI,   Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0,    Eigen::Vector3d::UnitX());

    Eigen::Vector3d t(tx_, ty_, tz_); // camera location 

    Eigen::Vector4d Pc(cx, cy, cz, 1.0);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>() = R;
    T.topRightCorner<3,1>() = t;

    Eigen::Vector4d Pr = T * Pc;
    return Pr.head<3>();
}

void BehaviorExecutive::ibvs_control(double x, double y, double z)
{
     Eigen::Vector3d error_vs(x - cx_, y - cy_, z);
   
    if (z == 0) z = 1e-6;

    Eigen::Matrix<double,3,3> L;
    L << -fx_/z, 0,     x/z,
          0,    -fy_/z, y/z,
          0, 0, -100;

    Eigen::Matrix<double, 3, 3> L_pseudo = L.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::Vector3d v_c = -0.3 * (L_pseudo * error_vs);

    Eigen::Vector3d v_b = camera_to_body(v_c.x(), v_c.y(), v_c.z());

    double vel_z = std::max(std::abs(v_b.z()), 0.09 ); // Not absolutely required, but more of a safety check

    publish_velocity_ibvs(v_b.x(), v_b.y(), -vel_z);
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<BehaviorExecutive>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
