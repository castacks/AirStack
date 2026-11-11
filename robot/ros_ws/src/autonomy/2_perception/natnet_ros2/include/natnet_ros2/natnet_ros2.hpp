// Copyright 2024 Laboratoire des signaux et systèmes
//
// This program is free software: you can redistribute it and/or 
// modify it under the terms of the GNU General Public License as 
// published by the Free Software Foundation, either version 3 of 
// the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, 
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License 
// along with this program. If not, see <https://www.gnu.org/licenses/>. 
//
// Author: Aarsh Thakker <aarsh.thakker@centralesupelec.fr>


#ifndef NATNET_ROS2_HPP
#define NATNET_ROS2_HPP

#include <iostream>
#include <sstream>
#include <map>
#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "natnet_ros2/object_data.hpp"

#include <NatNetCAPI.h>
#include <NatNetClient.h>

class NatNetNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    NatNetNode(rclcpp::NodeOptions& node_options);
    ~NatNetNode();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state);

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state);

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);


    bool disconnect();
    bool connect();
    void get_conn_params();
    void get_node_params();
    void set_conn_params();

    void process_frame(sFrameOfMocapData* data);

    void get_info();
    void del_info();
    std::chrono::nanoseconds get_latency_info(sFrameOfMocapData * data);

    void process_rigid_body(sRigidBodyData &data);
    void process_rigid_body_marker(sMarker &data);
    void process_individual_marker(sMarker &data);
    void process_pointcloud(sMarker &data);

protected:
    // Your existing member variables and methods go here

    NatNetClient* g_pClient;
    sNatNetClientConnectParams g_connectParams;
    ConnectionType g_ConnectionType = ConnectionType_Multicast;
    sServerDescription g_serverDescription;

private:

    bool log_internals=false; // To log internal parameters and mesurements from the motion capture system
    bool log_frames=false; // To log frames 
    bool log_latencies=false; // To log the latencies of the system
    bool pub_rigid_body = false; // To enable publishing the rigidbody as ros msg
    bool pub_rigid_body_marker = false; // To enable publishing individual markers of rigidbodies
    bool pub_individual_marker = false; // To publish the position of individual markers
    std::string immt;
    bool pub_pointcloud = false; // To publish all the marker as pointcloud
    bool remove_latency = false; // To remove latency from the frame data published on ros publisher
    bool use_helper_node = false; // flag to chnage parameter while using helper node
    rclcpp::Duration frame_delay = rclcpp::Duration(std::chrono::nanoseconds::zero());
    std::string serverIP;
    std::string clientIP;
    std::string serverType;
    std::string multicastAddress;
    int serverCommandPort;
    int serverDataPort;
    std::string global_frame;

    bool nearest_nbr = true;
    bool kalman = false; // Yet to implement
    bool individual_error = false;
    float E=0.04, E_x=0.01, E_y=0.01, E_z=0.01;
    float error_amp = 1.0;
    
    std::vector<std::string> object_names;
    std::vector<object_data> object_list;
    std::vector<object_data> object_list_prev;
    object_data tmp_obj;
    uint32_t frame_number{0};
    int unlabled_count;
    

    std::map<int32_t,std::string> ListRigidBodies; 
    std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr> RigidbodyPub;
    std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr> RigidbodyMarkerPub;
    std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr> IndividualMarkerPosePub;
    std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr> IndividualMarkerPointPub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr PointcloudPub;
    sensor_msgs::msg::PointCloud msgPointcloud;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    
};

void NATNET_CALLCONV frame_callback(sFrameOfMocapData *data, void *pUserData);


#endif
