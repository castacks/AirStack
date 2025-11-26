#pragma once

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


struct GraphNode {
  int fg_index;
  int bg_index;
  tf2::Stamped<tf2::Transform> tf;
};

struct alignas(16) Vec3 {
  float x, y, z;
};

struct alignas(16) Vec4 {
  float x, y, z, w;
};

struct alignas(16) State {
  Vec3 pos;
  Vec3 vel;
  Vec3 acc;
  Vec3 jerk;
  Vec3 collision;

  State(){}
  State(airstack_msgs::msg::Odometry odom){
    pos.x = odom.pose.position.x;
    pos.y = odom.pose.position.y;
    pos.z = odom.pose.position.z;

    vel.x = odom.twist.linear.x;
    vel.y = odom.twist.linear.y;
    vel.z = odom.twist.linear.z;

    acc.x = odom.acceleration.x;
    acc.y = odom.acceleration.y;
    acc.z = odom.acceleration.z;

    jerk.x = odom.jerk.x;
    jerk.y = odom.jerk.y;
    jerk.z = odom.jerk.z;
  }
};

struct TrajectoryParams {
  float vel_desired[3];
  float vel_max;
};

struct alignas(16) CommonInit {
  State initial_state;
  //int traj_count;
  //int traj_size;
  //float dt;
};

struct mat4 {
  float data[16];

  mat4(){
    std::memset(&data[0], 0, sizeof(data));
  }
  
  mat4(const tf2::Stamped<tf2::Transform>& stamped_tf){
    //const tf2::Transform& tf = stamped_tf.getTransform();
    const tf2::Vector3& t = stamped_tf.getOrigin();
    tf2::Matrix3x3 R = stamped_tf.getBasis();

    data[0]  = R[0][0];
    data[1]  = R[1][0];
    data[2]  = R[2][0];
    data[3]  = 0.0f;

    data[4]  = R[0][1];
    data[5]  = R[1][1];
    data[6]  = R[2][1];
    data[7]  = 0.0f;

    data[8]  = R[0][2];
    data[9]  = R[1][2];
    data[10] = R[2][2];
    data[11] = 0.0f;

    data[12] = t.x();
    data[13] = t.y();
    data[14] = t.z();
    data[15] = 1.0f;
  }

    mat4(const tf2::Transform& stamped_tf){
    const tf2::Vector3& t = stamped_tf.getOrigin();
    tf2::Matrix3x3 R = stamped_tf.getBasis();

    data[0]  = R[0][0];
    data[1]  = R[1][0];
    data[2]  = R[2][0];
    data[3]  = 0.0f;

    data[4]  = R[0][1];
    data[5]  = R[1][1];
    data[6]  = R[2][1];
    data[7]  = 0.0f;

    data[8]  = R[0][2];
    data[9]  = R[1][2];
    data[10] = R[2][2];
    data[11] = 0.0f;

    data[12] = t.x();
    data[13] = t.y();
    data[14] = t.z();
    data[15] = 1.0f;
  }
};

struct alignas(16) CollisionInfo {
  mat4 state_tf;
  float fx, fy, cx, cy;
  float baseline;
  int width, height;
  int limit;
  float scale;
  float expansion_radius;
  int graph_nodes;
};

struct alignas(16) TrajectoryPoint {
  Vec4 v1, v2;
  
  float x(){return v1.x;}
  float y(){return v1.y;}
  float z(){return v1.z;}
  float w(){return v1.w;}

  float get_vel(){return v2.w;}

  int get_collision(){return v2.x;}
  int get_unseen(){return v2.y;}
  int get_seen(){return v2.z;}
};

class GLInterface {
private:
  rclcpp::Node* node;
  tf2_ros::Buffer* tf_buffer;

  bool gl_inited;

  GLFWwindow *window_;
  GLuint horizProg_, vertProg_;
  float fx, fy, cx, cy, baseline;
  int image_width, image_height;
  int downsample_scale;
  float scale;
  float expansion_radius;
  float seen_radius;

  int current_node, graph_nodes, total_layers;
  
  GLuint texIn, fgHoriz, bgHoriz, fgFinal, bgFinal;

  std::deque<GraphNode> graph;
  std::string target_frame;

  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::string look_ahead_frame;
  std::vector<TrajectoryParams> traj_params;
  float dt, ht;
  GLuint traj_shader, collision_shader, traj_collision_shader;
  GLuint common_ubo, collision_info_ubo;
  GLuint params_ssbo, traj_ssbo, transform_ssbo;
  
  GLuint elapsed_query;
  
public:
  GLInterface(rclcpp::Node* node, tf2_ros::Buffer* tf_buffer);
  void handle_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void handle_disparity(const stereo_msgs::msg::DisparityImage::SharedPtr msg);
  void publish_viz(const std_msgs::msg::Header &hdr,
		   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fg_pub,
		   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bg_pub,
		   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub);
  void evaluate_trajectories(const airstack_msgs::msg::Odometry& look_ahead, std::vector<TrajectoryPoint>& trajectory_points, tf2::Transform& look_ahead_to_target_tf);

  int get_traj_size();
  void check_gl_error();
  void gl_tic();
  float gl_toc();
  
  void initGL(int original_width, int original_height, int downsampled_width, int downsampled_height);
  GLuint createComputeShader(const std::string &file);

};
