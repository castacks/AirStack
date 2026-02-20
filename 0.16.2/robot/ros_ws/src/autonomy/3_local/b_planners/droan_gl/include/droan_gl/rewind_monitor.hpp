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

struct Rewind
{
  bool valid;
  float start_time;
  tf2::Vector3 start_position;

  float duration;
  float distance;

  Rewind()
  {
    valid = false;
  }
};

class RewindMonitor
{
private:
  rclcpp::Node *node;

  float all_in_collision_duration_threshold;
  float all_in_collision_rewind_duration;
  float all_in_collision_start_time;

  float stationary_distance_threshold;
  float stationary_history_duration;
  float stationary_rewind_distance, stationary_rewind_duration;
  bool should_do_stationary_check;
  float min_dt;

  Rewind rewind;

  std::deque<std::pair<tf2::Vector3, float>> positions;

  visualization_msgs::msg::Marker clear, rewind_info_marker;

public:
  RewindMonitor(rclcpp::Node *node);

  void update_odom(const airstack_msgs::msg::Odometry::SharedPtr odom);
  void do_stationary_check(bool b);
  void found_trajectory(bool b);
  void trigger_rewind(float duration, float distance = -1.f);
  bool should_rewind();
  void clear_history();
  void publish_vis(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, std::string frame_id);
};
