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

bool check_target_changes(rclcpp::Logger, std::string target_list, rclcpp::Time current_time)
{
  // TODO
  return false;
}


// TODO
std::vector<airstack_msgs::msg::TaskAssignment> MissionManager::assign_tasks(rclcpp::Logger logger) const
{
  RCLCPP_INFO(logger, "Assigning tasks to drones");

  // Find how many active robots
  int number_of_agents = std::accumulate(valid_agents_.begin(), valid_agents_.end(), 0);

  // Decide how many search vs track tasks to assign
  int number_of_track_tasks = 0; // TODO
  int number_of_search_tasks = std::max(number_of_agents - number_of_track_tasks, 0);

  // Send out the request for the search map division
  // TODO Nayana

  // Assign the search and track tasks to drones based on distance to the task
  
  // return the msg to be published
  std::vector<airstack_msgs::msg::TaskAssignment> task_assignments(this->max_number_agents_);
  for (int i = 0; i < this->max_number_agents_; i++)
  {
    task_assignments[i].assigned_task_type = airstack_msgs::msg::TaskAssignment::SEARCH;
    task_assignments[i].assigned_task_number = i;
  }

  return task_assignments;
}