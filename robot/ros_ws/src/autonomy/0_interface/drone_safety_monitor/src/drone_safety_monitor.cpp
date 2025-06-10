// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <drone_safety_monitor/drone_safety_monitor.hpp>

TimeChecker::TimeChecker()
  : time_initialized(false){
}

void TimeChecker::update(rclcpp::Time time){
  this->time = time;
  time_initialized = true;
}

double TimeChecker::elapsed_since_last_update(rclcpp::Time time){
  if(!time_initialized)
    return std::numeric_limits<float>::infinity();
  return (time - this->time).seconds();
}


class DroneSafetyMonitorNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimate_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_estimate_timeout_pub;
  rclcpp::TimerBase::SharedPtr timer;
  float state_estimate_timeout;

  TimeChecker state_estimate_time_checker;
  
  DroneSafetyMonitorNode() : Node("odom_timeout_checker"){
    state_estimate_timeout = airstack::get_param(this, "state_estimate_timeout", 1.);

    // subscribers
    state_estimate_sub = this->create_subscription<nav_msgs::msg::Odometry>("state_estimate", 10,
									    std::bind(&DroneSafetyMonitorNode::state_estimate_callback,
										      this, std::placeholders::_1));

    // publishers
    state_estimate_timeout_pub = this->create_publisher<std_msgs::msg::Bool>("state_estimate_timed_out", 10);

    // timers
    timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(1000),
				 std::bind(&DroneSafetyMonitorNode::timer_callback, this));
				    
  }

  void state_estimate_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    state_estimate_time_checker.update(this->get_clock()->now());
  }

  void timer_callback(){
    std_msgs::msg::Bool state_estimate_timeout_msg;
    state_estimate_timeout_msg.data =
      state_estimate_time_checker.elapsed_since_last_update(this->get_clock()->now()) > state_estimate_timeout;
    state_estimate_timeout_pub->publish(state_estimate_timeout_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneSafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
} 
