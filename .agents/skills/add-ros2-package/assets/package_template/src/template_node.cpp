#include "package_template/template_node.hpp"

// TODO: Update include path with your package name

namespace your_namespace
{

TemplateNode::TemplateNode()
: Node("template_node"), initialized_(false)
{
  RCLCPP_INFO(this->get_logger(), "Initializing TemplateNode...");
  
  initialize();
  
  RCLCPP_INFO(this->get_logger(), "TemplateNode initialized successfully");
}

void TemplateNode::initialize()
{
  // Setup parameters
  setupParameters();
  
  // Setup publishers and subscribers
  setupPubSub();
  
  // Create timer for periodic processing
  auto timer_period = std::chrono::milliseconds(
    static_cast<int>(1000.0 / update_rate_));
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&TemplateNode::timerCallback, this));
  
  initialized_ = true;
}

void TemplateNode::setupParameters()
{
  // Declare parameters with defaults
  this->declare_parameter<double>("update_rate", 10.0);
  this->declare_parameter<double>("threshold", 0.5);
  this->declare_parameter<bool>("enable_debug", false);
  this->declare_parameter<std::string>("input_frame", "base_link");
  this->declare_parameter<std::string>("output_frame", "map");
  
  // TODO: Declare your module-specific parameters
  
  // Get parameter values
  update_rate_ = this->get_parameter("update_rate").as_double();
  threshold_ = this->get_parameter("threshold").as_double();
  enable_debug_ = this->get_parameter("enable_debug").as_bool();
  input_frame_ = this->get_parameter("input_frame").as_string();
  output_frame_ = this->get_parameter("output_frame").as_string();
  
  // Log parameters
  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  update_rate: %.2f Hz", update_rate_);
  RCLCPP_INFO(this->get_logger(), "  threshold: %.2f", threshold_);
  RCLCPP_INFO(this->get_logger(), "  enable_debug: %s", 
              enable_debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  input_frame: %s", input_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  output_frame: %s", output_frame_.c_str());
}

void TemplateNode::setupPubSub()
{
  // Create subscribers
  // Note: Topic names are generic and will be remapped in launch file
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 
    10,
    std::bind(&TemplateNode::odomCallback, this, std::placeholders::_1));
  
  // TODO: Add more subscribers as needed
  
  // Create publishers
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "output", 10);
  
  if (enable_debug_) {
    debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "debug/value", 10);
  }
  
  // TODO: Add more publishers as needed
  
  RCLCPP_INFO(this->get_logger(), "Publishers and subscribers initialized");
}

void TemplateNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Store latest odometry
  current_odom_ = msg;
  
  if (enable_debug_) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "Received odometry: pos=[%.2f, %.2f, %.2f]",
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z);
  }
  
  // TODO: Add your callback logic
}

void TemplateNode::timerCallback()
{
  if (!initialized_ || !current_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), 
                         *this->get_clock(), 
                         1000,  // Log every 1 second
                         "Waiting for odometry data...");
    return;
  }
  
  // Process and publish
  process();
}

void TemplateNode::process()
{
  // TODO: Implement your algorithm here
  
  // Example: Create output message
  auto output_msg = geometry_msgs::msg::Twist();
  
  // Example: Simple processing
  // In a real implementation, replace with your algorithm
  output_msg.linear.x = 0.5;  // Example value
  output_msg.linear.y = 0.0;
  output_msg.linear.z = 0.0;
  output_msg.angular.x = 0.0;
  output_msg.angular.y = 0.0;
  output_msg.angular.z = 0.1;  // Example value
  
  // Publish output
  cmd_pub_->publish(output_msg);
  
  // Publish debug information if enabled
  if (enable_debug_ && debug_pub_) {
    auto debug_msg = std_msgs::msg::Float64();
    debug_msg.data = 123.45;  // Example debug value
    debug_pub_->publish(debug_msg);
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Processing complete, output published");
}

}  // namespace your_namespace

// ============================================
// Main Function
// ============================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<your_namespace::TemplateNode>();
  
  RCLCPP_INFO(node->get_logger(), "Node running...");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  
  return 0;
}
