#include <rclcpp/rclcpp.hpp>
#include "rrtstar_global_navigator/rrtstar_global_navigator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RRTStarGlobalNavigator>(rclcpp::NodeOptions());
    
    RCLCPP_INFO(node->get_logger(), "Simple Global Navigator node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}