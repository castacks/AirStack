#include "../include/integrated_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<PlannerNode>();
    auto node = planner->get_node_handle();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}