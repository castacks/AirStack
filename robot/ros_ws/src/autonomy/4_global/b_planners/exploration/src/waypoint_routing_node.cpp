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

#include "../include/exploration_node.hpp"
#include "../include/exploration_logic.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "utils/utils.hpp"

class WaypointRoutingNode : public ExplorationNode
{
    using ExplorationNode::ExplorationNode;
public:
    void initialize() override;
    
private:
    std::string sub_target_path_topic_;
    std::deque<geometry_msgs::msg::Pose> target_path;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_target_path;
    void targetpathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    void timerCallback() override;
    void generate_plan() override;
};

void WaypointRoutingNode::initialize()
{
    this->declare_parameter<std::string>("sub_target_path_topic");
    if (!this->get_parameter("sub_target_path_topic", this->sub_target_path_topic_))
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_target_path_topic");
        return;
    }

    // TF buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_lidar_topic_, rclcpp::QoS(1).durability_volatile().best_effort(), std::bind(&WaypointRoutingNode::lidarCallback, this, std::placeholders::_1));
    this->sub_target_path = this->create_subscription<geometry_msgs::msg::PoseArray>(
        sub_target_path_topic_, rclcpp::QoS(10), std::bind(&WaypointRoutingNode::targetpathCallback, this, std::placeholders::_1));

    this->pub_global_plan = this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
    this->pub_goal_point =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_viz_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);
    this->pub_vdb =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_grid_viz_topic_, 10);

    // Set up the timer
    this->timer = this->create_wall_timer(std::chrono::seconds(5),
                                          std::bind(&WaypointRoutingNode::timerCallback, this));
    // Set up the service
    this->srv_exploration_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_exploration_toggle_topic_, std::bind(&WaypointRoutingNode::ExplorationToggleCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Exploration node initialized");
}

void WaypointRoutingNode::targetpathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received target path with %zu poses", msg->poses.size());
    this->target_path.assign(msg->poses.begin(), msg->poses.end());
}

void WaypointRoutingNode::generate_plan()
{
    RCLCPP_INFO(this->get_logger(), "Starting to generate plan...");

    // std::tuple<float, float, float, float> start_loc;
    // if (this->generated_paths.size() == 0)
    // {
    //     geometry_msgs::msg::Quaternion orientation = this->current_location.rotation;
    //     tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    //     q.normalize();
    //     double roll, pitch, yaw;
    //     tf2::Matrix3x3 m(q);
    //     m.getRPY(roll, pitch, yaw);
    //     start_loc = std::make_tuple(this->current_location.translation.x,
    //                                 this->current_location.translation.y,
    //                                 this->current_location.translation.z, yaw);
    // }
    // else
    // {
    //     geometry_msgs::msg::Quaternion orientation =
    //         this->generated_paths.back().poses.back().pose.orientation;
    //     tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    //     q.normalize();
    //     double roll, pitch, yaw;
    //     tf2::Matrix3x3 m(q);
    //     m.getRPY(roll, pitch, yaw);
    //     start_loc = std::make_tuple(this->generated_paths.back().poses.back().pose.position.x,
    //                                 this->generated_paths.back().poses.back().pose.position.y,
    //                                 this->generated_paths.back().poses.back().pose.position.z, yaw);
    // }

    ViewPoint start_point;

    if (this->generated_paths.empty() || this->generated_paths.back().poses.empty())
    {
        // Case 1: no generated path yet → use current_location
        geometry_msgs::msg::Quaternion orientation = this->current_location.rotation;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        start_point.x = this->current_location.translation.x;
        start_point.y = this->current_location.translation.y;
        start_point.z = this->current_location.translation.z;

        start_point.orientation = orientation;
        start_point.orientation_yaw = yaw;
        start_point.orientation_set = true;
    }
    else
    {
        // Case 2: use last pose of last generated path
        const auto &last_pose = this->generated_paths.back().poses.back().pose;
        geometry_msgs::msg::Quaternion orientation = last_pose.orientation;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        start_point.x = last_pose.position.x;
        start_point.y = last_pose.position.y;
        start_point.z = last_pose.position.z;

        start_point.orientation = orientation;
        start_point.orientation_yaw = yaw;
        start_point.orientation_set = true;
    }

    // convert first goal waypoint to ViewPoint
    ViewPoint goal_viewpoint;
    
    const auto &first_pose = target_path.front();

    goal_viewpoint.x = first_pose.position.x;
    goal_viewpoint.y = first_pose.position.y;
    goal_viewpoint.z = first_pose.position.z;

    // Keep quaternion + yaw consistent
    tf2::Quaternion q(first_pose.orientation.x,
                        first_pose.orientation.y,
                        first_pose.orientation.z,
                        first_pose.orientation.w);
    q.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    goal_viewpoint.orientation = first_pose.orientation;
    goal_viewpoint.orientation_yaw = yaw;
    goal_viewpoint.orientation_set = true;

    float timeout_duration = 5.0;
    std::optional<Path> gen_path_opt =
        // this->exploration_planner->generate_straight_rand_path(start_loc, timeout_duration);
        // this->exploration_planner->select_viewpoint_and_plan(start_point, timeout_duration);
        this->exploration_planner->plan_to_given_waypoint(start_point, goal_viewpoint);
    if (gen_path_opt.has_value() && gen_path_opt.value().size() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Generated path with %ld points",
                    gen_path_opt.value().size());

        // set the current goal location
        this->current_goal_location = geometry_msgs::msg::Transform();
        this->current_goal_location.translation.x = std::get<0>(gen_path_opt.value().back());
        this->current_goal_location.translation.y = std::get<1>(gen_path_opt.value().back());
        this->current_goal_location.translation.z = std::get<2>(gen_path_opt.value().back());
        float z_rot = std::get<3>(gen_path_opt.value().back());
        tf2::Quaternion q;
        q.setRPY(0, 0, z_rot); // Roll = 0, Pitch = 0, Yaw = yaw
        q.normalize();
        this->current_goal_location.rotation.x = q.x();
        this->current_goal_location.rotation.y = q.y();
        this->current_goal_location.rotation.z = q.z();
        this->current_goal_location.rotation.w = q.w();

        // publish the path
        nav_msgs::msg::Path generated_single_path;
        generated_single_path = nav_msgs::msg::Path();
        generated_single_path.header.stamp = this->now();
        generated_single_path.header.frame_id = world_frame_id_;
        for (auto point : gen_path_opt.value())
        {
            geometry_msgs::msg::PoseStamped point_msg;
            point_msg.pose.position.x = std::get<0>(point);
            point_msg.pose.position.y = std::get<1>(point);
            point_msg.pose.position.z = std::get<2>(point);
            // convert yaw rotation to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, std::get<3>(point)); // Roll = 0, Pitch = 0, Yaw = yaw
            q.normalize();
            point_msg.pose.orientation.x = q.x();
            point_msg.pose.orientation.y = q.y();
            point_msg.pose.orientation.z = q.z();
            point_msg.pose.orientation.w = q.w();
            point_msg.header.stamp = this->now();
            generated_single_path.poses.push_back(point_msg);
        }
        geometry_msgs::msg::PoseStamped last_goal_loc = generated_single_path.poses.back();
        this->current_goal_location.translation.x = last_goal_loc.pose.position.x;
        this->current_goal_location.translation.y = last_goal_loc.pose.position.y;
        this->current_goal_location.translation.z = last_goal_loc.pose.position.z;
        this->current_goal_location.rotation.z = last_goal_loc.pose.orientation.z;
        this->generated_paths.push_back(generated_single_path);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
    }
}

void WaypointRoutingNode::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "timer callback start");
    // get current TF to world
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped =
            this->tf_buffer->lookupTransform(this->world_frame_id_, this->robot_frame_id_,
                                             rclcpp::Time(0));
        this->current_location = transform_stamped.transform;
        if (!this->received_first_robot_tf)
        {
            this->received_first_robot_tf = true;
            this->last_location = this->current_location;
            this->last_position_change = this->now();
            RCLCPP_WARN(this->get_logger(), "Received first robot_tf");
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Robot tf not received: %s", ex.what());
    }


    // visualize
    visualization_msgs::msg::Marker vdb_marker_msg;
    generateVDBMarker(this->exploration_planner->occ_grid, map_frame_id_, vdb_marker_msg);
    vdb_marker_msg.header.stamp = this->now();
    this->pub_vdb->publish(vdb_marker_msg);

    if (this->enable_exploration)
    {
        if (!this->is_path_executing)
        {
            if (!target_path.empty() && this->received_first_robot_tf)
            {

                this->generate_plan();

                this->publish_plan();
                this->is_path_executing = true;
                this->last_position_change = this->now(); // Reset stall timer when starting new plan
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "target path size is %zu", target_path.size());
                RCLCPP_WARN(this->get_logger(), "tf received is %d", this->received_first_robot_tf);
            }
        }
        else
        {
            // check if the robot has reached the goal point
            std::tuple<float, float, float> current_point = std::make_tuple(
                this->current_location.translation.x, this->current_location.translation.y,
                this->current_location.translation.z);
            std::tuple<float, float, float> goal_point = std::make_tuple(
                this->current_goal_location.translation.x, this->current_goal_location.translation.y,
                this->current_goal_location.translation.z);
            if (get_point_distance(current_point, goal_point) <
                this->exploration_planner->path_end_threshold_m)
            {
                this->is_path_executing = false;
                this->generated_paths.clear();
                RCLCPP_INFO(this->get_logger(), "Reached goal point");
            }
            else
            {
                // Check if position has changed significantly
                std::tuple<float, float, float> last_point = std::make_tuple(this->last_location.translation.x,
                                                                             this->last_location.translation.y,
                                                                             this->last_location.translation.z);

                if (get_point_distance(current_point, last_point) > this->position_change_threshold)
                {
                    this->last_location = this->current_location;
                    this->last_position_change = this->now();
                }
                else
                {
                    // Check if we've been stationary for too long
                    rclcpp::Duration stall_duration = this->now() - this->last_position_change;
                    if (stall_duration.seconds() > this->stall_timeout_seconds)
                    {
                        RCLCPP_INFO(this->get_logger(), "Robot stationary for %f seconds, clearing plan",
                                    stall_duration.seconds());
                        this->is_path_executing = false;
                        this->generated_paths.clear();
                        this->last_position_change = this->now(); // Reset timer to avoid spam
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "timer callback end");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointRoutingNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}