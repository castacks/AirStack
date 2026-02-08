#include <cstdio>
#include <gcs_bringup/gcs_bringup.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GcsBringup>());
    rclcpp::shutdown();
    return 0;
}