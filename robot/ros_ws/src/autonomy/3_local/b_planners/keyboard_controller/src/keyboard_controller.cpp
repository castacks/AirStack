#include <keyboard_controller/keyboard_controller.h>
#include <iostream>
int kfd = 0;
struct termios cooked;
struct termios raw;
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    _exit(0);
}

KeyboardController::KeyboardController(): rclcpp::Node("keyboard_controller"){
    des_pose_vis_.header.frame_id = "map";
    des_pose_.header.frame_id = "map"; // (TODO)
    des_pose_.child_frame_id="map";
    
    des_pose_.pose.position.x = 0.0;
    des_pose_.pose.position.y = 0.0;
    des_pose_.pose.position.z = 0.0;
    des_pose_.pose.orientation.w = 1.0;
    des_pose_.pose.orientation.x = 0.0;
    des_pose_.pose.orientation.y = 0.0;
    des_pose_.pose.orientation.z = 0.0;
    des_pose_vis_.pose = des_pose_.pose;
    
    des_pose_publisher_ = this->create_publisher<airstack_msgs::msg::Odometry>("tracking_point",1);
    des_pose_vis_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tracking_point_vis",1);
    des_traj_publisher_ = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_override",1);
    drone_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry",1,std::bind(&KeyboardController::OdomCallback,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardController::timer_callback, this));

    // (Yunwoo) Topic-based keyboard input for GUI integration
    keyboard_input_sub_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input", 10,
        std::bind(&KeyboardController::KeyboardInputCallback, this, std::placeholders::_1));
    keyboard_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "keyboard_control_enable", 10,
        std::bind(&KeyboardController::KeyboardEnableCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Keyboard controller initialized. Listening on 'keyboard_input' topic.");
}

void KeyboardController::timer_callback(){
	
}

void KeyboardController::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    if (not is_drone_pose_received_){
        des_pose_.header = msg->header;
        des_pose_.pose = msg->pose.pose;
        des_pose_.child_frame_id=msg->child_frame_id;
    }
    is_drone_pose_received_ = true;
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose;
}

Isometry3d KeyboardController::GetEigenFromTf2(const tf2::Transform &t){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const tf2::Matrix3x3 &R_tf = t.getBasis();
    Eigen::Matrix3d R_eigen;
    R_eigen << R_tf[0][0], R_tf[0][1], R_tf[0][2],
               R_tf[1][0], R_tf[1][1], R_tf[1][2],
               R_tf[2][0], R_tf[2][1], R_tf[2][2];
    T.linear() = R_eigen;

    const tf2::Vector3 &p = t.getOrigin();
    T.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());

    return T;
}

bool KeyboardController::MoveMAV(const double &dx, const double &dy, const double &dz, const double &dyaw){
    if (not is_drone_pose_received_){
      return false;
    }
    if (not is_keyboard_received_){
    	des_pose_.pose = current_pose_.pose.pose;
    	is_keyboard_received_ = true;
    }
    Vector3d dpose(dx,dy,dz);
    tf2::Quaternion q_cur_des;
    q_cur_des.setX(des_pose_.pose.orientation.x);
    q_cur_des.setY(des_pose_.pose.orientation.y);
    q_cur_des.setZ(des_pose_.pose.orientation.z);
    q_cur_des.setW(des_pose_.pose.orientation.w);
    tf2::Vector3 t_cur_des(des_pose_.pose.position.x,des_pose_.pose.position.y,des_pose_.pose.position.z);
    tf2::Transform transform_cur;
    transform_cur.setIdentity();
    transform_cur.setRotation(q_cur_des);
    //transform matrix
    Eigen::Isometry3d Twb = GetEigenFromTf2(transform_cur);
    Vector3d dpose_w = Twb*dpose;
    //
    des_pose_.pose.position.x += dpose_w[0];
    des_pose_.pose.position.y += dpose_w[1];
    des_pose_.pose.position.z += dpose_w[2];
    //
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_cur_des).getEulerYPR(yaw,pitch,roll);
    yaw += dyaw;
    tf2::Quaternion q_des = tf2::Quaternion();
    q_des.setRPY(roll,pitch,yaw);
    des_pose_.pose.orientation.x = q_des.getX();
    des_pose_.pose.orientation.y = q_des.getY();
    des_pose_.pose.orientation.z = q_des.getZ();
    des_pose_.pose.orientation.w = q_des.getW();
    
    des_pose_vis_.pose = des_pose_.pose;

    des_pose_vis_publisher_->publish(des_pose_vis_);
    
    // Generate Trajectory
    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    traj.header.frame_id = des_pose_.header.frame_id;
    traj.header.stamp = des_pose_.header.stamp;

    airstack_msgs::msg::WaypointXYZVYaw wp1, wp2, wp3;
    // wp1
    wp1.position.x = current_pose_.pose.pose.position.x;
    wp1.position.y = current_pose_.pose.pose.position.y;
    wp1.position.z = current_pose_.pose.pose.position.z;
    tf2::Quaternion q1(current_pose_.pose.pose.orientation.x,current_pose_.pose.pose.orientation.y,
        current_pose_.pose.pose.orientation.z,current_pose_.pose.pose.orientation.w);
    tf2::Matrix3x3(q1).getRPY(roll, pitch, yaw);
    wp1.yaw = yaw;
    wp1.velocity = 1.0;
    // wp3
    wp3.position.x = des_pose_.pose.position.x;
    wp3.position.y = des_pose_.pose.position.y;
    wp3.position.z = des_pose_.pose.position.z;
    tf2::Quaternion q3(des_pose_.pose.orientation.x,des_pose_.pose.orientation.y,des_pose_.pose.orientation.z,des_pose_.pose.orientation.w);
    tf2::Matrix3x3(q3).getRPY(roll, pitch, yaw);
    wp3.yaw = yaw;
    wp3.velocity = 0.01;
    // wp2
    wp2.position.x = 0.5*(wp1.position.x+des_pose_.pose.position.x);
    wp2.position.y = 0.5*(wp1.position.y+des_pose_.pose.position.y);
    wp2.position.z = 0.5*(wp1.position.z+des_pose_.pose.position.z);
    tf2::Quaternion q_2 = q1.slerp(q3, 0.5);
    tf2::Matrix3x3(q_2).getRPY(roll, pitch, yaw);
    wp2.yaw = yaw;
    wp2.velocity = 1.0;
    traj.waypoints.push_back(wp1);
    traj.waypoints.push_back(wp2);
    traj.waypoints.push_back(wp3);
    des_traj_publisher_->publish(traj);
    // des_pose_publisher_->publish(des_pose_); // DO NOT PUBLISH THIS TOPIC
    return true;
}

void KeyboardController::KeyLoop(){
    char c;
    bool dirty = false;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the drone.");

    for(;;){
        //get the next event from the keyboard
        if(read(kfd,&c,1)<0){
            perror("read():");
            exit(-1);
        }
        Eigen::Vector4d move_pose; move_pose.setZero();
        switch (c){
            case KEYBOARD_O :{
                    increment_step_xyz_ -= 0.02;
                    // ROS_INFO("Decreased linear step. current: %f",increment_step_xyz_);
                    RCLCPP_INFO(this->get_logger(),"Decreased linear step. current: %f", increment_step_xyz_);
                    break;
            }
            case KEYBOARD_P :{
                    increment_step_xyz_ += 0.02;
                    // ROS_INFO("Increased linear step. current: %f",increment_step_xyz_);
                    RCLCPP_INFO(this->get_logger(),"Increased linear step. current: %f", increment_step_xyz_);
                    break;
            }
            case KEYBOARD_K : {
                    increment_step_yaw_ -= 3.141592/12;
                    // ROS_INFO("Decreased yaw step. current: %f", increment_step_yaw_);
                    RCLCPP_INFO(this->get_logger(),"Decreased yaw step. current: %f", increment_step_yaw_);
                    break;
            }
            case KEYBOARD_L : {
                    increment_step_yaw_ += 3.141592/12;
                    // ROS_INFO("Increased yaw step. current: %f", increment_step_yaw_);
                    RCLCPP_INFO(this->get_logger(),"Increased yaw step. current: %f", increment_step_yaw_);
                    break;
            }
            case KEYBOARD_W: {move_pose(0) = increment_step_xyz_; break;}
            case KEYBOARD_S: {move_pose(0) = -increment_step_xyz_; break;}
            case KEYBOARD_A: {move_pose(1) = increment_step_xyz_; break;}
            case KEYBOARD_D: {move_pose(1) = -increment_step_xyz_; break;}
            case KEYBOARD_Z: {move_pose(2) = -increment_step_xyz_; break;}
            case KEYBOARD_C: {move_pose(2) = increment_step_xyz_; break;}
            case KEYBOARD_Q: {move_pose(3) = increment_step_yaw_; break;}
            case KEYBOARD_E: {move_pose(3) = -increment_step_yaw_; break;}
        }
        bool flag_temp;
        flag_temp = MoveMAV(move_pose(0),move_pose(1),move_pose(2),move_pose(3));
    }
}

// (Yunwoo) Process a single key character - shared logic for terminal and topic input
void KeyboardController::ProcessKey(char c) {
    Eigen::Vector4d move_pose; 
    move_pose.setZero();
    
    switch (c) {
        case KEYBOARD_O: {
            increment_step_xyz_ -= 0.02;
            RCLCPP_INFO(this->get_logger(), "Decreased linear step. current: %f", increment_step_xyz_);
            break;
        }
        case KEYBOARD_P: {
            increment_step_xyz_ += 0.02;
            RCLCPP_INFO(this->get_logger(), "Increased linear step. current: %f", increment_step_xyz_);
            break;
        }
        case KEYBOARD_K: {
            increment_step_yaw_ -= 3.141592/12;
            RCLCPP_INFO(this->get_logger(), "Decreased yaw step. current: %f", increment_step_yaw_);
            break;
        }
        case KEYBOARD_L: {
            increment_step_yaw_ += 3.141592/12;
            RCLCPP_INFO(this->get_logger(), "Increased yaw step. current: %f", increment_step_yaw_);
            break;
        }
        case KEYBOARD_W: { move_pose(0) = increment_step_xyz_; break; }
        case KEYBOARD_S: { move_pose(0) = -increment_step_xyz_; break; }
        case KEYBOARD_A: { move_pose(1) = increment_step_xyz_; break; }
        case KEYBOARD_D: { move_pose(1) = -increment_step_xyz_; break; }
        case KEYBOARD_Z: { move_pose(2) = -increment_step_xyz_; break; }
        case KEYBOARD_C: { move_pose(2) = increment_step_xyz_; break; }
        case KEYBOARD_Q: { move_pose(3) = increment_step_yaw_; break; }
        case KEYBOARD_E: { move_pose(3) = -increment_step_yaw_; break; }
    }
    
    MoveMAV(move_pose(0), move_pose(1), move_pose(2), move_pose(3));
}

// (Yunwoo) Callback for topic-based keyboard input from GUI
void KeyboardController::KeyboardInputCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_enabled_) {
        RCLCPP_DEBUG(this->get_logger(), "Keyboard control is disabled. Ignoring input.");
        return;
    }
    
    if (!msg->data.empty()) {
        char c = msg->data[0];
        RCLCPP_DEBUG(this->get_logger(), "Received keyboard input via topic: %c", c);
        ProcessKey(c);
    }
}

// (Yunwoo) Callback to enable/disable keyboard control from GUI
void KeyboardController::KeyboardEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_enabled_ = msg->data;
    if (is_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Keyboard control ENABLED via GUI");
    } else {
        RCLCPP_INFO(this->get_logger(), "Keyboard control DISABLED via GUI");
    }
}
