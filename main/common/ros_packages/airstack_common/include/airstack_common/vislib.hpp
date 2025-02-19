#ifndef _VISLIB_H_
#define _VISLIB_H_

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <stdarg.h>

namespace vis {

  struct PointColor {
    float x, y, z;
    float r, g, b, a;
  };
  
  class Marker {
  private:
    visualization_msgs::msg::Marker marker;
    
  public:
    Marker(std::string frame_id, rclcpp::Time stamp, std::string ns, int id, int type);
    Marker& set_position(double x, double y, double z);
    Marker& set_scale(double x, double y, double z);
    Marker& set_color(float r, float g, float b, float a=1.f);
    Marker& set_text(std::string text);
    Marker& add_point(float x, float y, float z);
    Marker& add_color(float r, float g, float b, float a);
    Marker& set_pose(geometry_msgs::msg::Pose pose);
      
    Marker& set_action(uint8_t action);
    
    int get_id();
    
    visualization_msgs::msg::Marker get_marker();
  };
  
  class MarkerArray {
  private:
    std::string ns;
    int id;

    std::vector<Marker> markers;
    
  public:
    MarkerArray(std::string ns="default");
    
    Marker& add_text(std::string frame_id, rclcpp::Time stamp, std::string text, double x, double y, double z);
    Marker& add_line_list(std::string frame_id, rclcpp::Time stamp, float r, float g, float b, float a, float width, int lines, ...);
    Marker& add_line_strip(std::string frame_id, rclcpp::Time stamp, std::vector<PointColor> points, float width=0.1f);
    Marker& add_arrow(std::string frame_id, rclcpp::Time stamp, geometry_msgs::msg::Pose pose,
		      float length=0.5f, float width=0.05f, float height=0.05f);
    Marker& add_sphere(std::string frame_id, rclcpp::Time stamp, float x, float y, float z, float radius=0.1f);
    
    void overwrite();
    visualization_msgs::msg::MarkerArray get_marker_array();
  };
  
}

#endif
