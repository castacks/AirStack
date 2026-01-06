#include <rclcpp/rclcpp.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
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
#include <droan_gl/rewind_monitor.hpp>

class DisparityExpanderNode : public rclcpp::Node {
private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_stuck_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_map_sub;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fg_pub_, bg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_plan_vis_pub;
  rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rewind_info_pub;
  
  rclcpp::TimerBase::SharedPtr timer;
  
  std::string target_frame, look_ahead_frame, rewind_info_frame;
  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::vector<TrajectoryPoint> trajectory_points;
  vis::MarkerArray traj_markers;
  bool visualize;

  GLInterface* gl_interface;
  GlobalPlan* global_plan;
  RewindMonitor* rewind_monitor;

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
    tracking_point_sub = create_subscription<airstack_msgs::msg::Odometry>("tracking_point", 10,
									   std::bind(&DisparityExpanderNode::tracking_point_callback,
										     this, std::placeholders::_1));
    global_plan_sub = create_subscription<nav_msgs::msg::Path>("global_plan", 1,
							       std::bind(&DisparityExpanderNode::global_plan_callback,
									 this, std::placeholders::_1));
    reset_stuck_sub = this->create_subscription<std_msgs::msg::Empty>("reset_stuck", 1,
								      std::bind(&DisparityExpanderNode::reset_stuck_callback,
										this, std::placeholders::_1));
    clear_map_sub = this->create_subscription<std_msgs::msg::Empty>("clear_map", 1,
								    std::bind(&DisparityExpanderNode::clear_map_callback,
									      this, std::placeholders::_1));
	    
    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    fg_pub_ = create_publisher<sensor_msgs::msg::Image>("foreground_expanded", 1);
    bg_pub_ = create_publisher<sensor_msgs::msg::Image>("background_expanded", 1);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 1);
    traj_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("traj_debug", 1);
    graph_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("graph_vis", 1);
    global_plan_vis_pub = create_publisher<visualization_msgs::msg::MarkerArray>("local_planner_global_plan_vis", 1);
    traj_pub = create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("trajectory_segment_to_add", 1);
    stuck_pub = create_publisher<std_msgs::msg::Bool>("stuck", 1);
    rewind_info_pub = create_publisher<visualization_msgs::msg::MarkerArray>("rewind_info", 1);
    
    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    look_ahead_frame = airstack::get_param(this, "look_ahead_frame", std::string("look_ahead_point_stabilized"));
    rewind_info_frame = airstack::get_param(this, "rewind_info_frame", std::string("base_link_stabilized"));
    visualize = airstack::get_param(this, "visualize", true);
    
    look_ahead_valid = false;

    gl_interface = new GLInterface(this, tf_buffer);
    global_plan = new GlobalPlan(this, tf_buffer);
    rewind_monitor = new RewindMonitor(this);

    // TODO make this time a parameter
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
      gl_interface->publish_viz(msg->header, fg_pub_, bg_pub_, fg_bg_cloud_pub_, graph_vis_pub_);
  }
  
  void timer_callback(){
    if(!look_ahead_valid)
      return;

    std_msgs::msg::Bool stuck_msg;
    stuck_msg.data = rewind_monitor->should_rewind();
    stuck_pub->publish(stuck_msg);
    rewind_monitor->publish_vis(rewind_info_pub, rewind_info_frame);
    
    tf2::Transform look_ahead_to_target_tf;
    gl_interface->evaluate_trajectories(look_ahead, trajectory_points, look_ahead_to_target_tf);
    if(trajectory_points.empty())
      return;

    global_plan->trim(look_ahead);
    
    traj_markers.overwrite();
    vis::Marker& free_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    free_markers.set_namespace("free_points");
    free_markers.set_color(0., 1., 0.);
    free_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker& free_traj_markers = traj_markers.add_line_list(target_frame, look_ahead.header.stamp,
								0., 1., 0., 0.8,
								0.1, 0);
    free_traj_markers.set_namespace("free_trajectories");
								
    vis::Marker& collision_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    collision_markers.set_namespace("collision_points");
    collision_markers.set_color(1., 0., 0.);
    collision_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker& collision_traj_markers = traj_markers.add_line_list(target_frame, look_ahead.header.stamp,
								1., 0., 0., 0.8,
								0.1, 0);
    collision_traj_markers.set_namespace("collision_trajectories");
								
    vis::Marker& unseen_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
    unseen_markers.set_namespace("unseen_points");
    unseen_markers.set_color(0.7, 0.7, 0.7, 0.3);
    unseen_markers.set_scale(0.1, 0.1, 0.1);
    vis::Marker& unseen_traj_markers = traj_markers.add_line_list(target_frame, look_ahead.header.stamp,
								0.7, 0.7, 0.7, 0.3,
								0.1, 0);
    unseen_traj_markers.set_namespace("unseen_trajectories");

    int best_traj_index = -1;
    float best_traj_cost = std::numeric_limits<float>::infinity();
    bool is_traj_safe = true;
    int SEEN = 0;
    int UNSEEN = 1;
    int COLLISION = 2;
    int traj_status = SEEN;
    std::vector<tf2::Vector3> traj_points(gl_interface->get_traj_size());
    
    for(int i = 0; i < trajectory_points.size(); i++){
      TrajectoryPoint& state = trajectory_points[i];
      int traj_index = i/gl_interface->get_traj_size();
      int point_index = i % gl_interface->get_traj_size();

      //int seen, unseen, collision;
      //get_counts(state.w(), &seen, &unseen, &collision);
      int seen = state.get_seen();
      int unseen = state.get_unseen();
      int collision = state.get_collision();

      traj_points[point_index] = tf2::Vector3(state.x(), state.y(), state.z());

      if(collision > 0 && collision > seen){
	is_traj_safe = false;
	collision_markers.add_point(state.x(), state.y(), state.z());
	traj_status = COLLISION;
      }
      else if(seen > 1)
	free_markers.add_point(state.x(), state.y(), state.z());
      else{
	is_traj_safe = false;
       	unseen_markers.add_point(state.x(), state.y(), state.z());
	if(traj_status == SEEN)
	  traj_status = UNSEEN;
      }
      
      // if last waypoint in trajectory
      if(point_index == (gl_interface->get_traj_size() - 1)){
	vis::Marker* tm = &free_traj_markers;
	if(traj_status == UNSEEN)
	  tm = &unseen_traj_markers;
	else if(traj_status == COLLISION)
	  tm = &collision_traj_markers;
	
	for(int j = 1; j < traj_points.size(); j++){
	  tf2::Vector3& curr = traj_points[j];
	  tf2::Vector3& prev = traj_points[j-1];
	  tm->add_point(prev.x(), prev.y(), prev.z());
	  tm->add_point(curr.x(), curr.y(), curr.z());
	}

	int traj_status_log = traj_status;
	
	traj_status = SEEN;
	if(!is_traj_safe){
	  is_traj_safe = true;
	  continue;
	}
	
	auto [deviation, path_distance] = global_plan->get_distance(state.x(), state.y(), state.z());
	//RCLCPP_INFO_STREAM(get_logger(), i << " " << traj_status_log << " " << deviation << " " <<  path_distance);
	if(deviation >= 0 && path_distance >= 0){
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

    if(best_traj_index < 0){
      rewind_monitor->found_trajectory(false);
      return;
    }
    rewind_monitor->found_trajectory(true);
    
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
    global_plan->apply_smooth_yaw(traj, look_ahead);
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
  
  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg) {
    rewind_monitor->update_odom(msg);
  }

  void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    global_plan->set_global_plan(msg);
  }

  void reset_stuck_callback(const std_msgs::msg::Empty::SharedPtr msg){
    rewind_monitor->clear_history();
  }

  void clear_map_callback(const std_msgs::msg::Empty::SharedPtr msg){
    rewind_monitor->clear_history();
    // TODO gl_interface clear map
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparityExpanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
