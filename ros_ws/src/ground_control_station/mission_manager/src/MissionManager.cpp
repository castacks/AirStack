#include "mission_manager/MissionManager.h"

/*
Empty Constructor
*/
MissionManager::MissionManager(int max_number_agents) : max_number_agents_(max_number_agents)
{
  rclcpp::Time default_time(0, 0, RCL_ROS_TIME);
  time_of_last_call_.resize(max_number_agents_, default_time);
  valid_agents_.resize(max_number_agents_, false);
}

bool MissionManager::check_agent_changes(rclcpp::Logger logger, uint8_t robot_id, rclcpp::Time current_time)
{
  RCLCPP_INFO(logger, "Checking agent changes");

  time_of_last_call_[robot_id] = current_time;

  // TODO this logic does not need to happen every time an odom is received

  // Check how many agents have reported in the last x seconds
  // If change in the agents reporting, reassign tasks
  std::vector<bool> curr_valid_agents{false, false, false, false, false};
  rclcpp::Duration time_till_agent_not_valid = rclcpp::Duration::from_seconds(10.0);
  for (uint8_t i = 0; i < max_number_agents_; i++)
  {
    if (current_time - time_of_last_call_[i] < time_till_agent_not_valid)
    {
      curr_valid_agents[i] = true;
    }
  }
  bool change_in_agents = false;
  if (curr_valid_agents != valid_agents_)
  {
    change_in_agents = true;
  }
  valid_agents_ = curr_valid_agents;

  return change_in_agents;
}


// TODO
void MissionManager::assign_tasks(rclcpp::Logger logger) const
{
  RCLCPP_INFO(logger, "Assigning tasks to drones");
}