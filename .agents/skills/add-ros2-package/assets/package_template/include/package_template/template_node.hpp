#ifndef YOUR_PACKAGE_NAME__TEMPLATE_NODE_HPP_
#define YOUR_PACKAGE_NAME__TEMPLATE_NODE_HPP_

// TODO: Update include guard with your package name

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

// TODO: Add other includes as needed

namespace your_namespace
{

/**
 * @brief Template ROS 2 node demonstrating common patterns
 * 
 * This node shows how to:
 * - Subscribe to topics with callbacks
 * - Publish to topics
 * - Use parameters
 * - Use timers for periodic processing
 * - Handle transforms
 */
class TemplateNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Template Node object
   */
  TemplateNode();

  /**
   * @brief Destroy the Template Node object
   */
  ~TemplateNode() = default;

private:
  /**
   * @brief Initialize the node (called from constructor)
   */
  void initialize();

  /**
   * @brief Declare and get parameters
   */
  void setupParameters();

  /**
   * @brief Create publishers and subscribers
   */
  void setupPubSub();

  /**
   * @brief Callback for odometry messages
   * @param msg Odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Timer callback for periodic processing
   */
  void timerCallback();

  /**
   * @brief Process data and generate output
   */
  void process();

  // ============================================
  // ROS 2 Interfaces
  // ============================================

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // ============================================
  // Parameters
  // ============================================

  double update_rate_;
  double threshold_;
  bool enable_debug_;
  std::string input_frame_;
  std::string output_frame_;

  // ============================================
  // State Variables
  // ============================================

  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  bool initialized_;

  // TODO: Add your module-specific state variables
};

}  // namespace your_namespace

#endif  // YOUR_PACKAGE_NAME__TEMPLATE_NODE_HPP_
