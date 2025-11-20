#include <rclcpp/rclcpp.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <airstack_common/vislib.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <airstack_common/tflib.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <EGL/egl.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <mutex>

#include <droan_gl/gl_interface.hpp>

class DisparityExpanderNode : public rclcpp::Node {
private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fg_pub_, bg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_debug_pub_;

  rclcpp::TimerBase::SharedPtr timer;
  
  std::string target_frame;
  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::vector<TrajectoryPoint> trajectory_points;
  vis::MarkerArray traj_markers;
  bool visualize;

  GLInterface* gl_interface;

public:
  DisparityExpanderNode()
    : Node("disparity_expander_node"){
    disp_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>("/robot_1/sensors/front_stereo/disparity", 10,
								      std::bind(&DisparityExpanderNode::onDisparity,
										this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/robot_1/sensors/front_stereo/right/camera_info", 10,
								     std::bind(&DisparityExpanderNode::onCameraInfo,
									       this, std::placeholders::_1));
    look_ahead_sub = create_subscription<airstack_msgs::msg::Odometry>("/robot_1/trajectory_controller/look_ahead", 10,
								       std::bind(&DisparityExpanderNode::look_ahead_callback,
										 this, std::placeholders::_1));
    global_plan_sub = this->create_subscription<nav_msgs::msg::Path>("global_plan", 10,
								     std::bind(&DisparityExpanderNode::global_plan_callback,
									       this, std::placeholders::_1));
	    
    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    fg_pub_ = create_publisher<sensor_msgs::msg::Image>("foreground_expanded", 10);
    bg_pub_ = create_publisher<sensor_msgs::msg::Image>("background_expanded", 10);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 10);
    traj_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("traj_debug", 1);
    
    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    visualize = airstack::get_param(this, "visualize", true);
    
    look_ahead_valid = false;

    gl_interface = new GLInterface(this, tf_buffer);

    timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1./5.),
    				 std::bind(&DisparityExpanderNode::timer_callback, this));
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    gl_interface->handle_camera_info(msg);
  }

  void onDisparity(const stereo_msgs::msg::DisparityImage::SharedPtr msg) {
    gl_interface->handle_disparity(msg);
    if(visualize)
      gl_interface->publish_viz(msg->header, fg_pub_, bg_pub_, fg_bg_cloud_pub_);
  }
  
  void timer_callback(){
    gl_interface->evaluate_trajectories(look_ahead, trajectory_points);
    if(trajectory_points.empty())
      return;
    
    traj_markers.overwrite();
    vis::Marker& free_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    free_markers.set_namespace("free");
    free_markers.set_color(0., 1., 0.);
    free_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker& collision_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    collision_markers.set_namespace("collision");
    collision_markers.set_color(1., 0., 0.);
    collision_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker& unseen_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    unseen_markers.set_namespace("unseen");
    unseen_markers.set_color(0., 0., 1.);
    unseen_markers.set_scale(0.1, 0.1, 0.1);
    
    for(int i = 0; i < trajectory_points.size(); i++){
      TrajectoryPoint& state = trajectory_points[i];

      int seen, unseen, collision;
      get_counts(state.w(), &seen, &unseen, &collision);

      if(collision > 0 && collision > seen)
	collision_markers.add_point(state.x(), state.y(), state.z());
      else if(seen > 3)
	free_markers.add_point(state.x(), state.y(), state.z());
      else
       	unseen_markers.add_point(state.x(), state.y(), state.z());
    }
    
    traj_debug_pub_->publish(traj_markers.get_marker_array());
  }

  void get_counts(float w, int* seen, int* unseen, int* collision){
    int i = w;
    //RCLCPP_INFO_STREAM(get_logger(), "i: " << i);
    *collision = i % 1000;
    i -= *collision;
    //RCLCPP_INFO_STREAM(get_logger(), "i: " << i << " collision: " << *collision);
    *unseen = (i % 1000000)/1000;
    //RCLCPP_INFO_STREAM(get_logger(), "i: " << i << " unseen: " << *unseen);
    i -= *unseen * 1000;
    *seen = i / 1000000;
    //RCLCPP_INFO_STREAM(get_logger(), "i: " << i << " seen: " << *seen);
  }

  void look_ahead_callback(const airstack_msgs::msg::Odometry::SharedPtr msg) {
    look_ahead = *msg;
    look_ahead_valid = true;
  }

  void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    //global_plan_msg = *msg;
    //global_plan_id++;
    //global_plan_trajectory_distance = 0;
  }
  


};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparityExpanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
