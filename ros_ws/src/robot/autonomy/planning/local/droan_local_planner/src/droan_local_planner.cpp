#include <cstdio>
#include <droan_local_planner/droan_local_planner.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroanLocalPlanner>());
    rclcpp::shutdown();
    return 0;
}
