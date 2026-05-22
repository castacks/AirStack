#include "bpmp_predictor/BPMPPredictor.h"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bpmp_tracker::BPMPPredictor>());
    rclcpp::shutdown();
    return 0;
}