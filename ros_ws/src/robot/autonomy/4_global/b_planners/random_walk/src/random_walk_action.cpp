#include "../include/random_walk_action.hpp"

void RandomWalkAction::send_goal(
    rclcpp_action::Client<global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr
        random_walk_mode_client,
    global_planner_msgs::action::GetRandomWalkPlan::Goal goal) {
    auto send_goal_options =
        rclcpp_action::Client<global_planner_msgs::action::GetRandomWalkPlan>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [](std::shared_future<rclcpp_action::ClientGoalHandle<
               global_planner_msgs::action::GetRandomWalkPlan>::SharedPtr>
               future) {
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };
    auto goal_handle_future = random_walk_mode_client->async_send_goal(goal, send_goal_options);
}

rclcpp_action::GoalResponse RandomWalkAction::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const global_planner_msgs::action::GetRandomWalkPlan::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received request to generate plan");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RandomWalkAction::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
        goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RandomWalkAction::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<global_planner_msgs::action::GetRandomWalkPlan>>
        goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Accepted request to generate plan");
    std::thread{std::bind(&RandomWalkAction::generate_plan, this, std::placeholders::_1),
                goal_handle}
        .detach();
}
