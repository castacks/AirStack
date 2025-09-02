#include <rclcpp/rclcpp.hpp>
#include "jps_global_navigator/jps_global_navigator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<JPSGlobalNavigator>(rclcpp::NodeOptions());
    
    RCLCPP_INFO(node->get_logger(), "JPS Global Navigator node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}