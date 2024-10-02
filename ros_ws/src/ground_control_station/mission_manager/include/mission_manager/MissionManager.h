#ifndef MISSIONMANAGER_H_INCLUDED
#define MISSIONMANAGER_H_INCLUDED

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
#include "mission_manager/BeliefMap.h"

class MissionManager
{
  public:
    MissionManager(int max_number_agents);

    std::vector<airstack_msgs::msg::TaskAssignment> assign_tasks(rclcpp::Logger logger) const;
    bool check_agent_changes(rclcpp::Logger, uint8_t robot_id, rclcpp::Time current_time);
    bool check_target_changes(rclcpp::Logger, std::string target_list, rclcpp::Time current_time);

  private: 
    BeliefMap belief_map_;
    int max_number_agents_;
    std::vector<rclcpp::Time> time_of_last_call_;
    std::vector<bool> valid_agents_;

  };
  #endif /* MISSIONMANAGER_H_INCLUDED */