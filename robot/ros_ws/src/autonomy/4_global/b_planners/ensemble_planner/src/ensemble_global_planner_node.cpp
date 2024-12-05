
#include "../include/ensemble_global_planner_node.hpp"

void EnsembleGlobalPlannerNode::readParameters() {
    this->declare_parameter<std::string>("srv_global_plan_toggle_topic");
    if (!this->get_parameter("srv_global_plan_toggle_topic", this->srv_global_plan_toggle_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: srv_global_plan_toggle_topic");
    }
}

EnsembleGlobalPlannerNode::EnsembleGlobalPlannerNode() : Node("ensemble_global_planner_node") {
    // Initialize the Global Planner planner
    EnsembleGlobalPlannerNode::readParameters();
    this->srv_global_planner_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_global_plan_toggle_topic_,
        std::bind(&EnsembleGlobalPlannerNode::globalPlannnerToggleCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
}

void EnsembleGlobalPlannerNode::globalPlannnerToggleCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (this->enable_global_planner == false) {
        this->enable_global_planner = true;
        response->success = true;
        response->message = "Global Planner enabled";
        RCLCPP_INFO(this->get_logger(), "Global Planner enabled");
    } else {
        this->enable_global_planner = false;
        response->success = true;
        response->message = "Global Planer disabled";
        RCLCPP_INFO(this->get_logger(), "Global Planner disabled");
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnsembleGlobalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}