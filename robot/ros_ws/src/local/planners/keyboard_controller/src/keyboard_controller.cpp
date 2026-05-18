#include <keyboard_controller/keyboard_controller.h>

int kfd = 0;
struct termios cooked;
struct termios raw;

void quit(int sig) {
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    _exit(0);
}

KeyboardController::KeyboardController() : rclcpp::Node("keyboard_controller") {
    des_pose_.header.frame_id = "map";
    des_pose_.child_frame_id = "map";
    des_pose_.pose.position.x = 0.0;
    des_pose_.pose.position.y = 0.0;
    des_pose_.pose.position.z = 0.0;
    des_pose_.pose.orientation.w = 1.0;
    des_pose_vis_.header.frame_id = "map";
    des_pose_vis_.pose = des_pose_.pose;

    des_traj_publisher_ = this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(
        "trajectory_override", 1);
    des_pose_vis_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "tracking_point_vis", 1);
    drone_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 1,
        std::bind(&KeyboardController::OdomCallback, this, std::placeholders::_1));

    // Topic-based input for Foxglove/GUI integration
    keyboard_input_sub_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input", 10,
        std::bind(&KeyboardController::KeyboardInputCallback, this, std::placeholders::_1));
    keyboard_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "keyboard_control_enable", 10,
        std::bind(&KeyboardController::KeyboardEnableCallback, this, std::placeholders::_1));

    traj_mode_client_ = this->create_client<airstack_msgs::srv::TrajectoryMode>(
        "set_trajectory_mode");

    RCLCPP_INFO(this->get_logger(),
        "Keyboard controller ready. Terminal: KeyLoop thread. GUI: publish to 'keyboard_input'.");
}

void KeyboardController::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!is_drone_pose_received_) {
        des_pose_.header = msg->header;
        des_pose_.pose = msg->pose.pose;
        des_pose_.child_frame_id = msg->child_frame_id;
    }
    is_drone_pose_received_ = true;
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose;
}

Eigen::Isometry3d KeyboardController::GetEigenFromTf2(const tf2::Transform& t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const tf2::Matrix3x3& R_tf = t.getBasis();
    Eigen::Matrix3d R_eigen;
    R_eigen << R_tf[0][0], R_tf[0][1], R_tf[0][2],
               R_tf[1][0], R_tf[1][1], R_tf[1][2],
               R_tf[2][0], R_tf[2][1], R_tf[2][2];
    T.linear() = R_eigen;
    const tf2::Vector3& p = t.getOrigin();
    T.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());
    return T;
}

bool KeyboardController::MoveMAV(const double& dx, const double& dy,
                                  const double& dz, const double& dyaw) {
    if (!is_drone_pose_received_) return false;

    if (!is_keyboard_received_) {
        des_pose_.pose = current_pose_.pose.pose;
        is_keyboard_received_ = true;
    }

    // Rotate displacement from body frame to world frame
    tf2::Quaternion q_des;
    q_des.setX(des_pose_.pose.orientation.x);
    q_des.setY(des_pose_.pose.orientation.y);
    q_des.setZ(des_pose_.pose.orientation.z);
    q_des.setW(des_pose_.pose.orientation.w);
    tf2::Transform tf_des;
    tf_des.setIdentity();
    tf_des.setRotation(q_des);
    Eigen::Isometry3d Twb = GetEigenFromTf2(tf_des);
    Eigen::Vector3d dpose_w = Twb * Eigen::Vector3d(dx, dy, dz);

    des_pose_.pose.position.x += dpose_w[0];
    des_pose_.pose.position.y += dpose_w[1];
    des_pose_.pose.position.z += dpose_w[2];

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_des).getEulerYPR(yaw, pitch, roll);
    yaw += dyaw;
    tf2::Quaternion q_new;
    q_new.setRPY(roll, pitch, yaw);
    des_pose_.pose.orientation.x = q_new.getX();
    des_pose_.pose.orientation.y = q_new.getY();
    des_pose_.pose.orientation.z = q_new.getZ();
    des_pose_.pose.orientation.w = q_new.getW();

    des_pose_vis_.pose = des_pose_.pose;
    des_pose_vis_publisher_->publish(des_pose_vis_);

    // Build a 3-waypoint trajectory: current → midpoint → target
    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    traj.header.frame_id = des_pose_.header.frame_id;
    traj.header.stamp = this->now();

    airstack_msgs::msg::WaypointXYZVYaw wp1, wp2, wp3;

    double r1, p1, y1;
    tf2::Quaternion q1(current_pose_.pose.pose.orientation.x,
                       current_pose_.pose.pose.orientation.y,
                       current_pose_.pose.pose.orientation.z,
                       current_pose_.pose.pose.orientation.w);
    tf2::Matrix3x3(q1).getRPY(r1, p1, y1);

    double r3, p3, y3;
    tf2::Quaternion q3(des_pose_.pose.orientation.x, des_pose_.pose.orientation.y,
                       des_pose_.pose.orientation.z, des_pose_.pose.orientation.w);
    tf2::Matrix3x3(q3).getRPY(r3, p3, y3);

    wp1.position.x = current_pose_.pose.pose.position.x;
    wp1.position.y = current_pose_.pose.pose.position.y;
    wp1.position.z = current_pose_.pose.pose.position.z;
    wp1.yaw = y1;
    wp1.velocity = 1.0;

    wp3.position.x = des_pose_.pose.position.x;
    wp3.position.y = des_pose_.pose.position.y;
    wp3.position.z = des_pose_.pose.position.z;
    wp3.yaw = y3;
    wp3.velocity = 0.01;

    wp2.position.x = 0.5 * (wp1.position.x + wp3.position.x);
    wp2.position.y = 0.5 * (wp1.position.y + wp3.position.y);
    wp2.position.z = 0.5 * (wp1.position.z + wp3.position.z);
    tf2::Quaternion q2 = q1.slerp(q3, 0.5);
    double r2, p2, y2;
    tf2::Matrix3x3(q2).getRPY(r2, p2, y2);
    wp2.yaw = y2;
    wp2.velocity = 1.0;

    traj.waypoints.push_back(wp1);
    traj.waypoints.push_back(wp2);
    traj.waypoints.push_back(wp3);
    des_traj_publisher_->publish(traj);
    return true;
}

void KeyboardController::ProcessKey(char c) {
    Eigen::Vector4d move; move.setZero();
    switch (c) {
        case KEYBOARD_O: increment_step_xyz_ -= 0.1;
            RCLCPP_INFO(get_logger(), "Step xyz: %.2f", increment_step_xyz_); return;
        case KEYBOARD_P: increment_step_xyz_ += 0.1;
            RCLCPP_INFO(get_logger(), "Step xyz: %.2f", increment_step_xyz_); return;
        case KEYBOARD_K: increment_step_yaw_ -= M_PI / 12.0;
            RCLCPP_INFO(get_logger(), "Step yaw: %.2f", increment_step_yaw_); return;
        case KEYBOARD_L: increment_step_yaw_ += M_PI / 12.0;
            RCLCPP_INFO(get_logger(), "Step yaw: %.2f", increment_step_yaw_); return;
        case KEYBOARD_W: move(0) =  increment_step_xyz_; break;
        case KEYBOARD_S: move(0) = -increment_step_xyz_; break;
        case KEYBOARD_A: move(1) =  increment_step_xyz_; break;
        case KEYBOARD_D: move(1) = -increment_step_xyz_; break;
        case KEYBOARD_C: move(2) =  increment_step_xyz_; break;
        case KEYBOARD_Z: move(2) = -increment_step_xyz_; break;
        case KEYBOARD_Q: move(3) =  increment_step_yaw_; break;
        case KEYBOARD_E: move(3) = -increment_step_yaw_; break;
        default: return;
    }
    MoveMAV(move(0), move(1), move(2), move(3));
}

void KeyboardController::KeyboardInputCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_enabled_) return;
    if (!msg->data.empty()) {
        ProcessKey(static_cast<char>(msg->data[0]));
    }
}

bool KeyboardController::set_trajectory_mode(int32_t mode) {
    // Uses fire-and-forget async_send_request — safe to call from subscription
    // callbacks (inside rclcpp::spin) and from the KeyLoop thread alike.
    // spin_until_future_complete would deadlock when called from a callback.
    if (!traj_mode_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "set_trajectory_mode service not ready");
        return false;
    }
    auto req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    req->mode = mode;
    traj_mode_client_->async_send_request(req);
    return true;
}

void KeyboardController::KeyboardEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_enabled_ = msg->data;
    if (is_enabled_) {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK);
        RCLCPP_INFO(get_logger(), "Keyboard control ENABLED — trajectory mode set to TRACK");
    } else {
        set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
        RCLCPP_INFO(get_logger(), "Keyboard control DISABLED — trajectory mode set to ROBOT_POSE");
    }
}

void KeyboardController::KeyLoop() {
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    RCLCPP_INFO(get_logger(), "Terminal keyboard active: W/S=fwd/back A/D=left/right C/Z=up/down Q/E=yaw O/P=step K/L=yaw-step");
    set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::TRACK);

    char c;
    for (;;) {
        if (read(kfd, &c, 1) < 0) {
            perror("read()");
            set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
            return;
        }
        ProcessKey(c);
    }
}
