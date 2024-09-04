#ifndef _DISPARITY_MAP_REPRESENTATION_
#define _DISPARITY_MAP_REPRESENTATION_
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <disparity_graph/disparity_graph.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_representation_interface/map_representation.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace disparity_map_representation {
class DisparityMapRepresentation : public map_representation_interface::MapRepresentation {
   private:
    std::unique_ptr<disparity_graph::DisparityGraph> disp_graph;

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker points;

    // tf::TransformListener* listener;
    std::shared_ptr<tf2_ros::TransformListener> listener;

    // ros::Publisher debug_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub;

    int obstacle_check_points;
    double obstacle_check_radius;

   public:
    DisparityMapRepresentation();
    virtual double distance_to_obstacle(geometry_msgs::msg::PoseStamped pose,
                                        tf2::Vector3 direction);
    virtual void publish_debug();

    // virtual std::vector< std::vector<double> >
    // get_values(std::vector<airstack_msgs::TrajectoryXYZVYaw> trajectories);
    virtual std::vector<std::vector<double> > get_values(
        std::vector<std::vector<geometry_msgs::msg::PointStamped> > trajectories);
};
}  // namespace disparity_map_representation

#endif
