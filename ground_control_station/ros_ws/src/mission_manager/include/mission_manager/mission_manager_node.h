#ifndef MISSION_MANAGER_NODE_H_INCLUDED
#define MISSION_MANAGER_NODE_H_INCLUDED

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "airstack_common/ros2_helper.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "airstack_msgs/msg/search_mission_request.hpp"
#include "airstack_msgs/msg/task_assignment.hpp"
#include "airstack_msgs/msg/belief_map_data.hpp"
#include "mission_manager/BeliefMap.h"
#include "mission_manager/MissionManager.h"

/*
Subscirbers
- all the drones odoms
- search map updates
- targets

Publisher
- task assignment with plan request. Be together to avoid timing issues

When receive mission request or change in number of targets or change in number of drones
- check how many targets
- check how many drones
- see how many left for search
- divide up the space based on number of drones
- assign nearest drone to each task (targets and search regions)
- send plan request to each drone
*/


class MissionManagerNode : public rclcpp::Node
{
  public:
    MissionManagerNode()
    : Node("mission_manager")
    {
      double min_agent_altitude_to_be_active;
      double active_agent_check_n_seconds;
      double time_till_agent_not_valid;
      
      // grid_cell_size_ = airstack::get_param(this, "grid_cell_size", 10.0);
      this->declare_parameter("grid_cell_size", 10.0);
      this->declare_parameter("visualize_search_allocation", false);
      this->declare_parameter("max_number_agents", 5);
      this->declare_parameter("active_agent_check_n_seconds", 5.0);
      this->declare_parameter("min_agent_altitude_to_be_active", 2.0);
      this->declare_parameter("time_till_agent_not_valid", 10.0);
      this->declare_parameter("max_planning_time", 5.0);
      this->declare_parameter("budget", 100.0);
      this->declare_parameter("desired_speed", 4.0);
      this->get_parameter("grid_cell_size", grid_cell_size_);
      this->get_parameter("visualize_search_allocation", visualize_search_allocation_);
      this->get_parameter("max_number_agents", max_number_agents_);
      this->get_parameter("active_agent_check_n_seconds", active_agent_check_n_seconds);
      this->get_parameter("min_agent_altitude_to_be_active", min_agent_altitude_to_be_active);
      this->get_parameter("time_till_agent_not_valid", time_till_agent_not_valid);
      this->get_parameter("max_planning_time", max_planning_time_);
      this->get_parameter("budget", budget_);
      this->get_parameter("desired_speed", desired_speed_);
      
      mission_subscriber_ = this->create_subscription<airstack_msgs::msg::SearchMissionRequest>(
        "search_mission_request", 1, std::bind(&MissionManagerNode::search_mission_request_callback, this, std::placeholders::_1));

      // Create subscribers and publishers for max number of agents
      for (uint8_t i = 0; i < max_number_agents_; i++)
      {
        std::string topic_name = "agent_" + std::to_string(i) + "/odom";
        agent_odoms_subs_.push_back(
                this->create_subscription<nav_msgs::msg::Odometry>(
                    topic_name, 1,
                    [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        this->agent_odom_callback(msg, i);
                    }
                )
            );
        
        std::string agent_topic = "agent_" + std::to_string(i) + "/plan_request";
        plan_request_pubs_.push_back(
          this->create_publisher<airstack_msgs::msg::TaskAssignment>(agent_topic, 10));
      }

      tracked_targets_sub_ = this->create_subscription<std_msgs::msg::String>(
        "tracked_targets", 1, std::bind(&MissionManagerNode::tracked_targets_callback, this, std::placeholders::_1));

      belief_map_sub_ = this->create_subscription<airstack_msgs::msg::BeliefMapData>(
        "belief_map_updates", 1, std::bind(&MissionManagerNode::belief_map_callback, this, std::placeholders::_1));

      mission_manager_ = std::make_shared<MissionManager>(this->max_number_agents_, active_agent_check_n_seconds, min_agent_altitude_to_be_active, time_till_agent_not_valid);

      // TODO: set param for rate, make sure not communicated over network
      search_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "search_map_basestation", rclcpp::QoS(1).transient_local());

      viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("task_allocation_viz", 10.0);

      timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MissionManagerNode::search_map_publisher, this));
    }

  private:
    /* --- ROS SPECIFIC --- */

    // Publisher
    std::vector<rclcpp::Publisher<airstack_msgs::msg::TaskAssignment>::SharedPtr> plan_request_pubs_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr search_map_publisher_;

    // Subscribers
    rclcpp::Subscription<airstack_msgs::msg::SearchMissionRequest>::SharedPtr mission_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracked_targets_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> agent_odoms_subs_;
    rclcpp::Subscription<airstack_msgs::msg::BeliefMapData>::SharedPtr belief_map_sub_;


    /* --- MEMBER ATTRIBUTES --- */

    rclcpp::TimerBase::SharedPtr timer_;
    double grid_cell_size_;
    bool visualize_search_allocation_;
    int max_number_agents_; // TODO: get from param server
    airstack_msgs::msg::SearchMissionRequest latest_search_mission_request_;
    std::shared_ptr<MissionManager> mission_manager_;
    double max_planning_time_;
    double budget_;
    double desired_speed_;

    void publish_tasks(std::vector<airstack_msgs::msg::TaskAssignment> tasks) const
    {
      std::vector<bool> valid_agents = this->mission_manager_->get_valid_agents();
      for (uint8_t i = 0; i < this->max_number_agents_; i++)
      {
        if (valid_agents[i])
        {
          plan_request_pubs_[i]->publish(tasks[i]);
          RCLCPP_INFO(this->get_logger(), "Published task assignment for agent %d", i);
        }
      }
    }

    void search_map_publisher()
    {
      if (this->mission_manager_->belief_map_.is_initialized())
      {
        this->mission_manager_->belief_map_.map_.setTimestamp(this->now().nanoseconds());
        std::unique_ptr<grid_map_msgs::msg::GridMap> message;
        message = grid_map::GridMapRosConverter::toMessage(this->mission_manager_->belief_map_.map_);
        RCLCPP_DEBUG(
          this->get_logger(), "Publishing grid map (timestamp %f).",
          rclcpp::Time(message->header.stamp).nanoseconds() * 1e-9);
        search_map_publisher_->publish(std::move(message));
      }
    }

    void search_mission_request_callback(const airstack_msgs::msg::SearchMissionRequest::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received new search mission request");
      latest_search_mission_request_ = *msg;

      // TODO: visualize the seach mission request
      this->mission_manager_->belief_map_.reset_map(this->get_logger(), *msg, grid_cell_size_);
      this->publish_tasks(this->mission_manager_->assign_tasks(
          this->get_logger(), latest_search_mission_request_,
          viz_pub_, visualize_search_allocation_,
          max_planning_time_, budget_, desired_speed_));
    }

    void agent_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, const uint8_t &robot_id)
    {
      RCLCPP_INFO(this->get_logger(), "Received agent odom for robot %d", robot_id);
      if (this->mission_manager_->check_agent_changes(this->get_logger(), robot_id, msg->pose.pose, this->now()))
      {
        this->publish_tasks(this->mission_manager_->assign_tasks(
          this->get_logger(), latest_search_mission_request_,
          viz_pub_, visualize_search_allocation_,
          max_planning_time_, budget_, desired_speed_));
      }
    }

    void tracked_targets_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received target track list '%s'", msg->data.c_str());
      // TODO: save the list of tracked target

      // Check if change in the number of targets or id numbers
      if (this->mission_manager_->check_target_changes(this->get_logger(), msg->data, this->now()))
      {
        this->publish_tasks(this->mission_manager_->assign_tasks(
          this->get_logger(), latest_search_mission_request_,
          viz_pub_, visualize_search_allocation_,
          max_planning_time_, budget_, desired_speed_));
      }
    }

    void belief_map_callback(const airstack_msgs::msg::BeliefMapData::SharedPtr msg) const
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received new belief map data at x: " << msg->x_start << " y: " << msg->y_start);

      this->mission_manager_->belief_map_.update_map(this->get_logger(), msg);
    }

};
#endif /* MISSION_MANAGER_NODE_H_INCLUDED */