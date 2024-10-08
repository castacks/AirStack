
#ifndef RANDOM_WALK_ACTION_H
#define RANDOM_WALK_ACTION_H

#include <global_planner_msgs/action/get_random_walk_plan.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace RandomWalkAction {

// for client
void send_goal(rclcpp_action::Client<global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr
                   random_walk_mode_client,
               global_planner_msgs::action::GetRandomWalkPlan::Goal goal);

// for server
rclcpp_action::CancelResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const global_planner_msgs::action::GetRandomWalkPlan::Goal> goal);

rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
        goal_handle);

void handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
        goal_handle);


}  // namespace RandomWalkAction

#endif  // RANDOM_WALK_ACTION_H
