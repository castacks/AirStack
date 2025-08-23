#include <rclcpp/rclcpp.hpp>
#include "simple_global_navigator/simple_global_navigator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleGlobalNavigator>(rclcpp::NodeOptions());
    
    RCLCPP_INFO(node->get_logger(), "Simple Global Navigator node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}