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
    MissionManager();

    void assign_tasks(rclcpp::Logger logger) const;

  private: 
    BeliefMap belief_map_;

  };
  #endif /* MISSIONMANAGER_H_INCLUDED */