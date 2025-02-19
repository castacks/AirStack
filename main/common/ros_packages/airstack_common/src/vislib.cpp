#include <rclcpp/rclcpp.hpp>
#include <airstack_common/vislib.hpp>

namespace vis{

  Marker::Marker(std::string frame_id, rclcpp::Time stamp, std::string ns, int id, int type){
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.frame_locked = false;
  }
  
  Marker& Marker::set_position(double x, double y, double z){
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    return *this;
  }

  Marker& Marker::set_pose(geometry_msgs::msg::Pose pose){
    marker.pose = pose;
    return *this;
  }

  Marker& Marker::set_scale(double x, double y, double z){
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
    return *this;
  }
  
  Marker& Marker::set_color(float r, float g, float b, float a){
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return *this;
  }
  
  Marker& Marker::set_text(std::string text){
    marker.text = text;
    return *this;
  }

  Marker& Marker::add_point(float x, float y, float z){
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    marker.points.push_back(point);
    
    return *this;
  }

  Marker& Marker::add_color(float r, float g, float b, float a){
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    marker.colors.push_back(color);
    
    return *this;
  }
  
  Marker& Marker::set_action(uint8_t action){
    marker.action = action;
    return *this;
  }

  int Marker::get_id(){
    return marker.id;
  }
  
  visualization_msgs::msg::Marker Marker::get_marker(){
    return marker;
  }
  
  MarkerArray::MarkerArray(std::string ns){
    this->ns = ns;
    id = 0;
  }
  
  Marker& MarkerArray::add_text(std::string frame_id, rclcpp::Time stamp, std::string text, double x, double y, double z){
    Marker marker(frame_id, stamp, ns, id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    marker.set_text(text);
    marker.set_position(x, y, z);
    
    if(marker.get_id() >= markers.size())
      markers.push_back(marker);
    else
      markers[marker.get_id()] = marker;

    return markers[marker.get_id()];
  }

  Marker& MarkerArray::add_line_list(std::string frame_id, rclcpp::Time stamp, float r, float g, float b, float a , float width, int lines, ...){
    Marker marker(frame_id, stamp, ns, id++, visualization_msgs::msg::Marker::LINE_LIST);
    marker.set_scale(width, 0, 0);
    marker.set_color(r, g, b, a);
    
    va_list list;
    va_start(list, lines);

    for(size_t i = 0; i < lines; i++){
      float x1 = va_arg(list, double);
      float y1 = va_arg(list, double);
      float z1 = va_arg(list, double);
      float x2 = va_arg(list, double);
      float y2 = va_arg(list, double);
      float z2 = va_arg(list, double);

      marker.add_point(x1, y1, z1);
      marker.add_point(x2, y2, z2);
    }

    if(marker.get_id() >= markers.size())
      markers.push_back(marker);
    else
      markers[marker.get_id()] = marker;

    return markers[marker.get_id()];
  }

  Marker& MarkerArray::add_line_strip(std::string frame_id, rclcpp::Time stamp, std::vector<PointColor> points, float width){
    Marker marker(frame_id, stamp, ns, id++, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.set_scale(width, 0, 0);
    
    for(size_t i = 0; i < points.size(); i++){
      PointColor& pc = points[i];
      marker.add_point(pc.x, pc.y, pc.z);
      marker.add_color(pc.r, pc.g, pc.b, pc.a);
    }
    
    if(marker.get_id() >= markers.size())
      markers.push_back(marker);
    else
      markers[marker.get_id()] = marker;

    return markers[marker.get_id()];
  }

  Marker& MarkerArray::add_arrow(std::string frame_id, rclcpp::Time stamp, geometry_msgs::msg::Pose pose,
				 float length, float width, float height){
    Marker marker(frame_id, stamp, ns, id++, visualization_msgs::msg::Marker::ARROW);
    marker.set_scale(length, width, height);
    marker.set_pose(pose);

    if(marker.get_id() >= markers.size())
      markers.push_back(marker);
    else
      markers[marker.get_id()] = marker;

    return markers[marker.get_id()];
  }

  void MarkerArray::overwrite(){
    for(size_t i = 0; i < markers.size(); i++)
      markers[i].set_action(visualization_msgs::msg::Marker::DELETE);
    id = 0;
  }

  Marker& MarkerArray::add_sphere(std::string frame_id, rclcpp::Time stamp, float x, float y, float z, float radius){
    Marker marker(frame_id, stamp, ns, id++, visualization_msgs::msg::Marker::SPHERE);
    marker.set_scale(2*radius, 2*radius, 2*radius);
    marker.set_position(x, y, z);

    if(marker.get_id() >= markers.size())
      markers.push_back(marker);
    else
      markers[marker.get_id()] = marker;

    return markers[marker.get_id()];
  }
  
  visualization_msgs::msg::MarkerArray MarkerArray::get_marker_array(){
    visualization_msgs::msg::MarkerArray marker_array;
    
    for(size_t i = 0; i < markers.size(); i++){
      marker_array.markers.push_back(markers[i].get_marker());
    }
    
    return marker_array;
  }

}
