#include <cstdio>
#include <single_agent_bridge/single_agent_bridge.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleAgentBridge>());
    rclcpp::shutdown();
    return 0;
}
