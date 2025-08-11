#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

// Messages
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rosgraph_msgs/msg/clock.hpp>

// Services
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

#include "sim.h"

class MavrosMockNode : public rclcpp::Node {
public:
  MavrosMockNode()
    : Node("mavros_mock_node") {
    // Declare and get parameters
    this->declare_parameter<double>("image_fov", 60.);
    this->declare_parameter<int>("image_width", 800);
    this->declare_parameter<int>("image_height", 600);
    this->declare_parameter<double>("baseline", 0.12);
    this->declare_parameter<std::string>("model_filename", "/data/fire_academy/fire_academy_no_box.fbx");
    this->declare_parameter<double>("model_scale", 0.01);
    this->declare_parameter<double>("model_x_offset", 0.0);
    this->declare_parameter<double>("model_y_offset", 0.0);
    this->declare_parameter<double>("model_z_offset", 0.0);
    this->declare_parameter<int>("screen_width", 0);
    this->declare_parameter<int>("screen_height", 0);

    image_fov = this->get_parameter("image_fov").as_double();
    image_width = this->get_parameter("image_width").as_int();
    image_height = this->get_parameter("image_height").as_int();
    baseline = this->get_parameter("baseline").as_double();
    model_filename = this->get_parameter("model_filename").as_string();
    model_scale = this->get_parameter("model_scale").as_double();
    model_x_offset = this->get_parameter("model_x_offset").as_double();
    model_y_offset = this->get_parameter("model_y_offset").as_double();
    model_z_offset = this->get_parameter("model_z_offset").as_double();
    int screen_width  = this->get_parameter("screen_width").as_int();
    int screen_height  = this->get_parameter("screen_height").as_int();
    

    sim = new Sim(image_fov, image_width, image_height, baseline, model_filename,
		  model_scale, model_x_offset, model_y_offset, model_z_offset, screen_width, screen_height);
    state.connected = true;
    state.armed = false;

    do_fast_mode = false;
    fast_mode_start = -1000.f;

    //RCLCPP_INFO(this->get_logger(), "Loaded parameters: width=%d, height=%d, model_dir=%s, scale=%.2f",
    //		image_width, image_height, model_filename.c_str(), model_scale);

    // Service servers
    set_mode_srv = this->create_service<mavros_msgs::srv::SetMode>("/robot_1/interface/mavros/set_mode",
								   std::bind(&MavrosMockNode::handle_set_mode, this,
									     std::placeholders::_1, std::placeholders::_2));

    arming_srv = this->create_service<mavros_msgs::srv::CommandBool>("/robot_1/interface/mavros/cmd/arming",
								     std::bind(&MavrosMockNode::handle_arming, this,
									       std::placeholders::_1, std::placeholders::_2));

    takeoff_srv = this->create_service<mavros_msgs::srv::CommandTOL>("/robot_1/interface/mavros/cmd/takeoff",
								     std::bind(&MavrosMockNode::handle_takeoff, this,
									       std::placeholders::_1, std::placeholders::_2));

    // Subscriber
    attitude_sub = this->create_subscription<mavros_msgs::msg::AttitudeTarget>("/robot_1/interface/mavros/setpoint_raw/attitude", 10,
									       std::bind(&MavrosMockNode::attitude_callback,
											 this, std::placeholders::_1));

    // Publishers
    clock_pub = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    state_pub = this->create_publisher<mavros_msgs::msg::State>("/robot_1/interface/mavros/state", 10);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/robot_1/interface/mavros/local_position/odom", 10);
    left_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/robot_1/sensors/front_stereo/left/image_rect", rclcpp::SensorDataQoS());
    right_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/robot_1/sensors/front_stereo/right/image_rect", rclcpp::SensorDataQoS());
    left_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/robot_1/sensors/front_stereo/left/camera_info", 10);
    right_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/robot_1/sensors/front_stereo/right/camera_info", 10);

	
    // Timer
    timer_period = 1./60.;
    timer = this->create_wall_timer(std::chrono::duration<double>(timer_period),
				    std::bind(&MavrosMockNode::timer_callback, this));
    frame_count = 0;
    frame_pub = 6;
  }

private:
  // Service callbacks
  void handle_set_mode(const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
		       std::shared_ptr<mavros_msgs::srv::SetMode::Response> res) {
    res->mode_sent = true;

    if(req->custom_mode == "GUIDED"){
      sim->drone.offboard = true;
    }
    else
      sim->drone.offboard = false;
    
    RCLCPP_INFO(this->get_logger(), "Set mode to: %s %d", req->custom_mode.c_str(), sim->drone.offboard);
  }

  void handle_arming(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
		     std::shared_ptr<mavros_msgs::srv::CommandBool::Response> res) {
    //RCLCPP_INFO(this->get_logger(), "Arming request: %s", req->value ? "true" : "false");
    res->success = true;
    
    //state.armed = req->value;
    sim->drone.armed = req->value;
  }

  void handle_takeoff(const std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> req,
		      std::shared_ptr<mavros_msgs::srv::CommandTOL::Response> res) {
    //RCLCPP_INFO(this->get_logger(), "Takeoff requested: altitude = %.2f", req->altitude);
    res->success = true;

    do_fast_mode = true;
  }

  void attitude_callback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg) {
    //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //			 "Received attitude setpoint with thrust: %.2f", msg->thrust);


    if(state.mode == "GUIDED"){
      tf2::Quaternion tf_q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

      //RCLCPP_INFO(this->get_logger(), "roll pitch yaw: %.2f %.2f %.2f", roll, pitch, yaw);
      
      //sim->drone.set_command(roll, pitch, yaw, -(msg->thrust*(15.0 - 0.0) + 0.0));
      //sim->drone.set_command(-yaw, -roll, pitch, -(msg->thrust*(15.0 - 0.0) + 0.0));
      sim->drone.set_command(-roll, -pitch, yaw, -(msg->thrust*(15.0 - 0.0) + 0.0));
      //RCLCPP_INFO(this->get_logger(), "drone: %.2f %.2f %.2f", sim->drone.roll, sim->drone.pitch, sim->drone.yaw);
    }
  }

  // Timer callback
  void timer_callback(){
    
    float sim_time_seconds = sim->step(left_image_data, right_image_data);

    // fast forwarding past autonomy's 10 second wait period after takeoff for sending commands
    //RCLCPP_INFO(this->get_logger(), "fast mode start: %.2f %.2f", fast_mode_start, sim_time_seconds);
    if(do_fast_mode){
      do_fast_mode = false;
      fast_mode_start = sim_time_seconds;
      sim->set_fast_mode(true);

      timer = this->create_wall_timer(std::chrono::duration<double>(1./1000.),
				      std::bind(&MavrosMockNode::timer_callback, this));
    }
    if((fast_mode_start > 0.f) && ((sim_time_seconds - fast_mode_start) > 9.5f)){
      fast_mode_start = -1000.f;
      sim->set_fast_mode(false);

      timer = this->create_wall_timer(std::chrono::duration<double>(timer_period),
				      std::bind(&MavrosMockNode::timer_callback, this));
    }
    
    
    if((frame_count++) % frame_pub != 0)
      return;
    rclcpp::Time sim_time(static_cast<uint64_t>(sim_time_seconds * 1e9));
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = sim_time;
    clock_pub->publish(clock_msg);

    auto now = clock_msg.clock;
    sensor_msgs::msg::Image left_image_msg;
    left_image_msg.header.stamp = now;
    left_image_msg.header.frame_id = "front_stereo_left_camera_optical_frame";
    left_image_msg.height = image_height;
    left_image_msg.width = image_width;
    left_image_msg.encoding = "rgb8";
    left_image_msg.is_bigendian = 0;
    left_image_msg.step = image_width * 3;
    left_image_msg.data = left_image_data;
    left_image_pub->publish(left_image_msg);

    sensor_msgs::msg::Image right_image_msg;
    right_image_msg.header.stamp = now;
    right_image_msg.header.frame_id = "front_stereo_right_camera_optical_frame";
    right_image_msg.height = image_height;
    right_image_msg.width = image_width;
    right_image_msg.encoding = "rgb8";
    right_image_msg.is_bigendian = 0;
    right_image_msg.step = image_width * 3;
    right_image_msg.data = right_image_data;
    right_image_pub->publish(right_image_msg);

    float fx = sim->get_fx();
    float fy = sim->get_fy();
    float cx = sim->get_cx();
    float cy = sim->get_cy();
    
    sensor_msgs::msg::CameraInfo left_cam_info;
    left_cam_info.header.stamp = now;
    left_cam_info.header.frame_id = "front_stereo_left_camera_optical_frame";
    left_cam_info.width = image_width;
    left_cam_info.height = image_height;
    left_cam_info.k = {
        fx,  0, cx,
         0, fy, cy,
         0,  0,  1
    };
    left_cam_info.r = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    left_cam_info.p = {
        fx,  0, cx, 0,
         0, fy, cy, 0,
         0,  0,  1, 0
    };
    left_cam_info.d.resize(5, 0.0);
    left_cam_info.distortion_model = "plumb_bob";
    left_info_pub->publish(left_cam_info);

    sensor_msgs::msg::CameraInfo right_cam_info;
    right_cam_info.header.stamp = now;
    right_cam_info.header.frame_id = "front_stereo_right_camera_optical_frame";
    right_cam_info.width = image_width;
    right_cam_info.height = image_height;
    right_cam_info.k = {
        fx,  0, cx,
         0, fy, cy,
         0,  0,  1
    };
    right_cam_info.r = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    right_cam_info.p = {
        fx,  0, cx, -fx*baseline,
         0, fy, cy, 0,
         0,  0,  1, 0
    };
    right_cam_info.d.resize(5, 0.0);
    right_cam_info.distortion_model = "plumb_bob";
    right_info_pub->publish(right_cam_info);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = -sim->drone.position.z;
    odom.pose.pose.position.y = -sim->drone.position.x;
    odom.pose.pose.position.z = sim->drone.position.y;

    tf2::Quaternion tf_q(sim->drone.orientation.x, sim->drone.orientation.y, sim->drone.orientation.z, sim->drone.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    tf2::Quaternion new_q;
    //new_q.setRPY(-yaw, -roll, pitch);
    new_q.setRPY(-glm::radians(sim->drone.roll), -glm::radians(sim->drone.pitch), glm::radians(sim->drone.yaw));

    //RCLCPP_INFO_STREAM(get_logger(), "rpy: " << roll << " " << pitch << " " << yaw);
    /*    
    odom.pose.pose.orientation.x = dq.x;
    odom.pose.pose.orientation.y = dq.y;
    odom.pose.pose.orientation.z = dq.z;
    odom.pose.pose.orientation.w = dq.w;
    */
    //*
    odom.pose.pose.orientation.x = new_q.x();
    odom.pose.pose.orientation.y = new_q.y();
    odom.pose.pose.orientation.z = new_q.z();
    odom.pose.pose.orientation.w = new_q.w();
    //*/
    glm::vec3 body_vel = glm::inverse(sim->drone.orientation) * sim->drone.velocity;
    odom.twist.twist.linear.x = -body_vel.z;
    odom.twist.twist.linear.y = -body_vel.x;
    odom.twist.twist.linear.z = body_vel.y;
    odom_pub->publish(odom);

    //glm::vec3 body_vel1 = sim->drone.orientation * sim->drone.velocity;
    //glm::vec3 body_vel2 = glm::inverse(sim->drone.orientation) * sim->drone.velocity;
    //RCLCPP_INFO_STREAM(get_logger(), "v1: " << body_vel1.x << " " << body_vel1.y << " " << body_vel1.z);
    //RCLCPP_INFO_STREAM(get_logger(), "v2: " << body_vel2.x << " " << body_vel2.y << " " << body_vel2.z);


    if(sim->drone.offboard)
      state.mode = "GUIDED";
    else
      state.mode = "LOITER";
    state.armed = sim->drone.armed;
    state_pub->publish(state);
  }

  int frame_count, frame_pub;
  
  Sim* sim;  
  mavros_msgs::msg::State state;
  std::vector<unsigned char> left_image_data, right_image_data;
  bool do_fast_mode;
  float fast_mode_start;
  double timer_period;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer;
  
  // Parameters
  double image_fov;
  int image_width, image_height;
  double baseline;
  std::string model_filename;
  double model_scale, model_x_offset, model_y_offset, model_z_offset;

  // Service servers
  rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_srv;
  rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_srv;
  rclcpp::Service<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_srv;

  // Subscriber
  rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_sub;

  // Publishers
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavrosMockNode>());
  rclcpp::shutdown();
  return 0;
}
