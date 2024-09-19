#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "airstack_msgs/msg/search_mission_request.hpp"
#include "airstack_msgs/msg/task_assignment.hpp"


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


class MissionManager : public rclcpp::Node
{
  public:
    MissionManager()
    : Node("mission_manager"), count_(0)
    {
      mission_subscriber_ = this->create_subscription<airstack_msgs::msg::SearchMissionRequest>(
        "search_mission_request", 1, std::bind(&MissionManager::search_mission_request_callback, this, std::placeholders::_1));

      for (uint8_t i = 0; i < max_number_agents; i++)
      {
        agent_odoms_subs_.push_back(this->create_subscription<std_msgs::msg::String>(
          "agent_" + std::to_string(i) + "/odom", 1, std::bind(&MissionManager::agent_odom_callback, this, std::placeholders::_1)));
        std::string agent_topic = "agent_" + std::to_string(i) + "/plan_request";
        plan_request_pubs_.push_back(
          this->create_publisher<airstack_msgs::msg::TaskAssignment>(agent_topic, 10));
      }

      tracked_targets_sub_ = this->create_subscription<std_msgs::msg::String>(
        "tracked_targets", 1, std::bind(&MissionManager::tracked_targets_callback, this, std::placeholders::_1));

      search_map_sub_ = this->create_subscription<std_msgs::msg::String>(
        "search_map_updates", 1, std::bind(&MissionManager::search_map_callback, this, std::placeholders::_1));

    }

  private:
    int max_number_agents = 5; // TODO: get from param server

    // Publisher
    std::vector<rclcpp::Publisher<airstack_msgs::msg::TaskAssignment>::SharedPtr> plan_request_pubs_;

    // Subscribers
    rclcpp::Subscription<airstack_msgs::msg::SearchMissionRequest>::SharedPtr mission_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracked_targets_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> agent_odoms_subs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_map_sub_;

    size_t count_;
    airstack_msgs::msg::SearchMissionRequest latest_search_mission_request_;

    // TODO this should be in a separate class!
    void assign_tasks() const
    {
      RCLCPP_INFO(this->get_logger(), "Assigning tasks to drones");
    }

    void search_mission_request_callback(const airstack_msgs::msg::SearchMissionRequest::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received new search mission request");
      latest_search_mission_request_ = *msg;

      this->assign_tasks();
      // TODO: publish the task assignment to the drones
    }

    void agent_odom_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received agent odom '%s'", msg->data.c_str());
      // TODO: save the list of agent id and their odometry

      // TODO: check if change in the number of agents or id numbers
      // if so, reassign tasks
      // this->assign_tasks();
      // TODO: publish the task assignment to the drones
    }

    void tracked_targets_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received target track list '%s'", msg->data.c_str());
      // TODO: save the list of tracked target

      // TODO: check if change in the number of targets or id numbers
      // if so, reassign tasks
      // this->assign_tasks();
      // TODO: publish the task assignment to the drones
    }

    void search_map_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received search map '%s'", msg->data.c_str());

      // TODO: save the search map using that class
    }

    

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>());
    rclcpp::shutdown();
    return 0;
  }
