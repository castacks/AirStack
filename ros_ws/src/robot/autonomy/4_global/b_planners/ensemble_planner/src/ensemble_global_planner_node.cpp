
#include "../include/ensemble_global_planner_node.hpp"

void EnsembleGlobalPlannerNode::readParameters() {
    this->declare_parameter<std::string>("srv_random_walk_toggle_topic");
    if (!this->get_parameter("srv_random_walk_toggle_topic", this->srv_random_walk_toggle_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: srv_random_walk_toggle_topic");
        return std::optional<init_params>{};
    }
}

EnsembleGlobalPlannerNode::EnsembleGlobalPlannerNode() : Node("ensemble_global_planner_node") {
    // Initialize the random walk planner
    EnsembleGlobalPlannerNode::readParameters();
    this->srv_random_walk_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_random_walk_toggle_topic_,
        std::bind(&RandomWalkNode::randomWalkToggleCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
}

void RandomWalkNode::randomWalkToggleCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (this->enable_random_walk == false) {
        this->enable_random_walk = true;
        response->success = true;
        response->message = "Random walk enabled";
        RCLCPP_INFO(this->get_logger(), "Random walk enabled");
    } else {
        this->enable_random_walk = false;
        response->success = true;
        response->message = "Random walk disabled";
        RCLCPP_INFO(this->get_logger(), "Random walk disabled");
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomWalkNode>());
    rclcpp::shutdown();
    return 0;
}