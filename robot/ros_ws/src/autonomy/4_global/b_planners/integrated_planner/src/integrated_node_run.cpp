#include "../include/integrated_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<PlannerNode>();
    auto node = planner->get_node_handle();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}