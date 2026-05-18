#include <csignal>
#include <thread>
#include <keyboard_controller/keyboard_controller.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>();
    signal(SIGINT, quit);

    // KeyLoop blocks on raw terminal reads — run it in a separate thread
    // so rclcpp::spin() keeps processing subscriber callbacks concurrently.
    std::thread key_thread([&]() { node->KeyLoop(); });

    rclcpp::spin(node);

    if (key_thread.joinable()) key_thread.join();
    rclcpp::shutdown();
    return 0;
}
