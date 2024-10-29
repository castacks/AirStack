#ifndef _DISPARITY_MAP_REPRESENTATION_
#define _DISPARITY_MAP_REPRESENTATION_
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

#include <disparity_graph/disparity_graph.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_representation_interface/map_representation.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace disparity_map_representation {
class DisparityMapRepresentation : public map_representation_interface::MapRepresentation {
   private:
    disparity_graph::DisparityGraph disp_graph;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker points_marker;

    int obstacle_check_num_points;
    double obstacle_check_radius;

   public:
    DisparityMapRepresentation();
    virtual double distance_to_obstacle(geometry_msgs::msg::PoseStamped pose,
                                        tf2::Vector3 direction);

    virtual const visualization_msgs::msg::MarkerArray& get_debug_markerarray() const override;

    virtual std::vector<std::vector<double> > get_values(
        std::vector<std::vector<geometry_msgs::msg::PointStamped> > trajectories) override;

    virtual void initialize(const rclcpp::Node::SharedPtr& node_ptr, const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) override;
};
}  // namespace disparity_map_representation

#endif
