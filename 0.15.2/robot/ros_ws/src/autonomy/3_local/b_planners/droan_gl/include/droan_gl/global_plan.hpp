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

#include <vector>
#include <mutex>

#include <droan_gl/gl_interface.hpp>
#include <trajectory_library/trajectory_library.hpp>

class GlobalPlan
{
private:
  rclcpp::Node *node;
  tf2_ros::Buffer *tf_buffer;

  std::string target_frame;
  int current_global_plan_id, next_global_plan_id;
  nav_msgs::msg::Path path;
  Trajectory global_plan;

  bool update_global_plan();

public:
  GlobalPlan(rclcpp::Node *node, tf2_ros::Buffer *tf_buffer);
  void set_global_plan(const nav_msgs::msg::Path::SharedPtr msg);
  void trim(const airstack_msgs::msg::Odometry &msg);
  std::tuple<float, float> get_distance(float x, float y, float z);
  void publish_vis(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
  void apply_smooth_yaw(airstack_msgs::msg::TrajectoryXYZVYaw &best_traj_msg, const airstack_msgs::msg::Odometry look_ahead);
};
