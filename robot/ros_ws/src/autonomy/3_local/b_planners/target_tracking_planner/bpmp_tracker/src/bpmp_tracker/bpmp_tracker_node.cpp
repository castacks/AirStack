#include "bpmp_tracker/BPMPTracker.h"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bpmp_tracker::BPMPTracker>());
    rclcpp::shutdown();
    return 0;
}