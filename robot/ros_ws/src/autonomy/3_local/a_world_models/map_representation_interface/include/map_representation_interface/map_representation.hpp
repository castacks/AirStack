#ifndef _CORE_MAP_REPRESENTATION_H_
#define _CORE_MAP_REPRESENTATION_H_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_library/trajectory_library.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace map_representation_interface {

class CostMap {
   protected:
    rclcpp::Node::SharedPtr node_ptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
    CostMap() {}

   public:
    /**
       Takes in a list of trajectories and outputs a value for each waypoint in each trajectory.
       @return A vector of vectors containing values for each waypoint. There is a vector of vectors
       for each trajectory. There is a vector of doubles for each waypoint within a trajectory.
    */

    virtual std::vector<std::vector<double> > get_cost_per_waypoint(
        const std::vector<Trajectory>& trajectories) = 0;
    /**
       Clears the map.
    */
    virtual void clear() {
        RCLCPP_ERROR(node_ptr->get_logger(), "clear CALLED BUT NOT IMPLEMENTED");
    }

    /**
       Use this function to publish visualizations of the map that might be helpful for debugging.
    */
    virtual const visualization_msgs::msg::MarkerArray& get_debug_markerarray() const {}

    virtual ~CostMap() {}

    virtual void initialize(const rclcpp::Node::SharedPtr& node_ptr,
                            const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) {
        RCLCPP_INFO(node_ptr->get_logger(), "MapRepresentation initialize called");
        this->node_ptr = node_ptr;
        this->tf_buffer_ptr = tf_buffer_ptr;
    }
};

}  // namespace map_representation_interface

#endif
