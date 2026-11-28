// Explicit standard-library requirements of the unchanged included guard.
#include <future>
#include <stdexcept>
// RRM diagnostic host; original guard included by reference, never copied or patched.
#include <mavros_interface/px4_actuation_guard.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rrm_startup_guard");
  auto guard = std::make_unique<mavros_interface::Px4ActuationGuard>(*node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  guard.reset();
  rclcpp::shutdown();
  return 0;
}
