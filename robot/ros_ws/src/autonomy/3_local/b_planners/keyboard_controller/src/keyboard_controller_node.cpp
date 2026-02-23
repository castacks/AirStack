#include <cstdio>
#include <keyboard_controller/keyboard_controller.h>

int main(int argc, char** argv) {
    //rclcpp::init(argc, argv);
    //KeyboardController keyboard_operator;
    //signal(SIGINT,quit);
    //keyboard_operator.KeyLoop();
    //return 0;
    
	
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>();

    signal(SIGINT, quit);

    std::thread key_thread([&]() {
        node->KeyLoop();
    });

    rclcpp::spin(node);

    if (key_thread.joinable()) {
        key_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}                            
