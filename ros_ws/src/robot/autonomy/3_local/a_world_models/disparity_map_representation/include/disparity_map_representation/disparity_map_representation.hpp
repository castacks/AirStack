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
    std_msgs::msg::ColorRGBA green;
    std_msgs::msg::ColorRGBA red;

    int obstacle_check_num_points;
    double obstacle_check_radius;
    tf2::Quaternion Q_UP, Q_DOWN, Q_LEFT, Q_RIGHT;
    std::set<tf2::Quaternion> Q_DIRECTIONS;

    tf2::Vector3 quaternion_to_unit_vector(tf2::Quaternion q) {
        tf2::Vector3 unit(1, 0, 0);
        return tf2::Transform(q) * unit;
    }

    auto create_side_and_up_vectors(tf2::Vector3 direction) {
        // Choose an arbitrary up vector
        tf2::Vector3 forward = direction.normalized();
        tf2::Vector3 arbitrary_up(0.0, 0.0, 1.0);

        // If forward is parallel to arbitrary_up, choose a different up vector
        if (fabs(forward.dot(arbitrary_up)) > 0.999) {
            arbitrary_up = tf2::Vector3(0.0, 1.0, 0.0);
        }

        // Side vector (perpendicular to both forward and arbitrary up)
        tf2::Vector3 side = forward.cross(arbitrary_up).normalized();

        // True up vector (perpendicular to both forward and side)
        tf2::Vector3 up = side.cross(forward).normalized();

        return std::make_pair(side, up);
    }
    void check_pose_and_add_marker(const Trajectory& trajectory, const Waypoint& waypoint,
                                   double dist, const tf2::Vector3 direction,
                                   double& closest_obstacle_distance);
tf2::Vector3 determine_waypoint_direction(const Trajectory& trajectory,
                                                                      const Waypoint& waypoint,
                                                                      size_t waypoint_index) ;

   public:
    DisparityMapRepresentation();
    virtual double distance_to_obstacle(geometry_msgs::msg::PoseStamped pose,
                                        tf2::Vector3 direction);

    virtual const visualization_msgs::msg::MarkerArray& get_debug_markerarray() const override;

    virtual std::vector<std::vector<double> > get_values(
        std::vector<Trajectory> trajectories) override;

    virtual void initialize(const rclcpp::Node::SharedPtr& node_ptr,
                            const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) override;
};
}  // namespace disparity_map_representation

#endif
