// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


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
