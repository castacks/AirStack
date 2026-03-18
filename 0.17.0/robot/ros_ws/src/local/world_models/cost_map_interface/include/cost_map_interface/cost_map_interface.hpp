// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_library/trajectory_library.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace cost_map_interface {

class CostMapInterface {
   protected:
    rclcpp::Node::SharedPtr node_ptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
    CostMapInterface() {}

   public:
    /**
       Takes in a list of trajectories and outputs a value for each waypoint in each trajectory.
       @return A vector of vectors containing values for each waypoint. There is a vector of vectors
       for each trajectory. There is a vector of doubles for each waypoint within a trajectory.
    */

    virtual std::vector<std::vector<double> > get_trajectory_costs_per_waypoint(
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

    virtual ~CostMapInterface() {}

    virtual void initialize(const rclcpp::Node::SharedPtr& node_ptr,
                            const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) {
        RCLCPP_INFO(node_ptr->get_logger(), "CostMapInterface initialize called");
        this->node_ptr = node_ptr;
        this->tf_buffer_ptr = tf_buffer_ptr;
    }
};

}  // namespace cost_map_interface 
