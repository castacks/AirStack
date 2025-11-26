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
#include <droan_gl/global_plan.hpp>

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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_plan_vis_pub;
  rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
  
  rclcpp::TimerBase::SharedPtr timer;
  
  std::string target_frame, look_ahead_frame;
  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::vector<TrajectoryPoint> trajectory_points;
  vis::MarkerArray traj_markers;
  bool visualize;

  GLInterface* gl_interface;
  GlobalPlan* global_plan;

public:
  DisparityExpanderNode()
    : Node("disparity_expander_node"){
    disp_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>("disparity", 10,
								      std::bind(&DisparityExpanderNode::onDisparity,
										this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10,
								     std::bind(&DisparityExpanderNode::onCameraInfo,
									       this, std::placeholders::_1));
    look_ahead_sub = create_subscription<airstack_msgs::msg::Odometry>("look_ahead", 10,
								       std::bind(&DisparityExpanderNode::look_ahead_callback,
										 this, std::placeholders::_1));
    global_plan_sub = create_subscription<nav_msgs::msg::Path>("global_plan", 1,
							       std::bind(&DisparityExpanderNode::global_plan_callback,
									 this, std::placeholders::_1));
	    
    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    fg_pub_ = create_publisher<sensor_msgs::msg::Image>("foreground_expanded", 1);
    bg_pub_ = create_publisher<sensor_msgs::msg::Image>("background_expanded", 1);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 1);
    traj_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("traj_debug", 1);
    global_plan_vis_pub = create_publisher<visualization_msgs::msg::MarkerArray>("local_planner_global_plan_vis", 1);
    traj_pub = create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_segment_to_add", 1);
    
    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    look_ahead_frame = airstack::get_param(this, "look_ahead_frame", std::string("look_ahead_point_stabilized"));
    visualize = airstack::get_param(this, "visualize", true);
    
    look_ahead_valid = false;

    gl_interface = new GLInterface(this, tf_buffer);
    global_plan = new GlobalPlan(this, tf_buffer);

    timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(2.*1./5.),
    				 std::bind(&DisparityExpanderNode::timer_callback, this));
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
    gl_interface->handle_camera_info(msg);
  }

  void onDisparity(const stereo_msgs::msg::DisparityImage::SharedPtr msg){
    gl_interface->handle_disparity(msg);
    if(visualize)
      gl_interface->publish_viz(msg->header, fg_pub_, bg_pub_, fg_bg_cloud_pub_);
  }
  
  void timer_callback(){
    if(!look_ahead_valid)
      return;
    
    tf2::Transform look_ahead_to_target_tf;
    gl_interface->evaluate_trajectories(look_ahead, trajectory_points, look_ahead_to_target_tf);
    if(trajectory_points.empty())
      return;

    global_plan->trim(look_ahead);
    
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

    int best_traj_index = -1;
    float best_traj_cost = std::numeric_limits<float>::infinity();
    bool is_traj_safe = true;
    
    for(int i = 0; i < trajectory_points.size(); i++){
      TrajectoryPoint& state = trajectory_points[i];
      int traj_index = i/gl_interface->get_traj_size();

      //int seen, unseen, collision;
      //get_counts(state.w(), &seen, &unseen, &collision);
      int seen = state.get_seen();
      int unseen = state.get_unseen();
      int collision = state.get_collision();

      if(collision > 0 && collision > seen){
	is_traj_safe = false;
	collision_markers.add_point(state.x(), state.y(), state.z());
      }
      else if(seen > 3)
	free_markers.add_point(state.x(), state.y(), state.z());
      else{
	is_traj_safe = false;
       	unseen_markers.add_point(state.x(), state.y(), state.z());
      }
      
      // if last waypoint in trajectory
      if((i % gl_interface->get_traj_size()) == (gl_interface->get_traj_size() - 1)){
	if(!is_traj_safe){
	  is_traj_safe = true;
	  continue;
	}
	auto [deviation, path_distance] = global_plan->get_distance(state.x(), state.y(), state.z());
	if(deviation > 0 && path_distance > 0){
	  // TODO add weights as ros parameters
	  float cost = deviation - path_distance;
	  if(cost < best_traj_cost){
	    best_traj_cost = cost;
	    best_traj_index = traj_index;
	  }
	}
      }
    }
    
    traj_debug_pub_->publish(traj_markers.get_marker_array());
    global_plan->publish_vis(global_plan_vis_pub);
    
    if(best_traj_index < 0)
      return;
    
    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    for(int i = 0; i < gl_interface->get_traj_size(); i++){
      airstack_msgs::msg::WaypointXYZVYaw wp;

      TrajectoryPoint& state = trajectory_points[best_traj_index*gl_interface->get_traj_size() + i];
      tf2::Vector3 p(state.x(), state.y(), state.z());
      p = look_ahead_to_target_tf*p;

      wp.position.x = p.x();
      wp.position.y = p.y();
      wp.position.z = p.z();
      wp.velocity = state.get_vel();

      traj.waypoints.push_back(wp);
    }

    traj.header.stamp = look_ahead.header.stamp;
    traj.header.frame_id = look_ahead_frame;
    apply_smooth_yaw(traj);
    traj_pub->publish(traj);
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
    global_plan->set_global_plan(msg);
  }
  
  void apply_smooth_yaw(airstack_msgs::msg::TrajectoryXYZVYaw &best_traj_msg){
    bool found_initial_heading = false;
    double initial_heading = 0;
    try{
      tf2::Stamped<tf2::Transform> transform;
      tf_buffer->canTransform(best_traj_msg.header.frame_id, look_ahead.header.frame_id,
			      look_ahead.header.stamp, rclcpp::Duration::from_seconds(0.1));
      auto transform_msg = tf_buffer->lookupTransform(best_traj_msg.header.frame_id,
						      look_ahead.header.frame_id,
						      look_ahead.header.stamp);
      tf2::fromMsg(transform_msg, transform);

      transform.setOrigin(tf2::Vector3(0, 0, 0)); // only care about rotation
      initial_heading =
	tf2::getYaw(transform * tflib::to_tf(look_ahead.pose.orientation));

      found_initial_heading = true;
    }
    catch (tf2::TransformException &ex){
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
    }

    if (found_initial_heading){
      best_traj_msg.waypoints[0].yaw = initial_heading;
      double alpha = 0.1;
      double sin_yaw_prev = sin(best_traj_msg.waypoints[0].yaw);
      double cos_yaw_prev = cos(best_traj_msg.waypoints[0].yaw);

      for (size_t i = 1; i < best_traj_msg.waypoints.size(); i++){
	airstack_msgs::msg::WaypointXYZVYaw wp_prev = best_traj_msg.waypoints[i - 1];
	airstack_msgs::msg::WaypointXYZVYaw &wp_curr = best_traj_msg.waypoints[i];

	double yaw = atan2(wp_curr.position.y - wp_prev.position.y,
			   wp_curr.position.x - wp_prev.position.x);
	double cos_yaw = alpha * cos(yaw) + (1 - alpha) * cos_yaw_prev;
	double sin_yaw = alpha * sin(yaw) + (1 - alpha) * sin_yaw_prev;
	yaw = atan2(sin_yaw, cos_yaw);

	sin_yaw_prev = sin_yaw;
	cos_yaw_prev = cos_yaw;

	wp_curr.yaw = yaw;
      }
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparityExpanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
