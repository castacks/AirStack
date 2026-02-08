#include <cstdio>
#include <multi_agent_bridge/multi_agent_bridge.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiAgentBridge>());
    rclcpp::shutdown();
    return 0;
}
