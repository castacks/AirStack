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
: time_initialized(false) {}

void TimeChecker::update(rclcpp::Time time)
{
  this->time = time;
  time_initialized = true;
}

double TimeChecker::elapsed_since_last_update(rclcpp::Time time)
{
  if (!time_initialized) {
    return std::numeric_limits<double>::infinity();
  }
  return (time - this->time).seconds();
}


class DroneSafetyMonitorNode : public rclcpp::Node
{
public:
  DroneSafetyMonitorNode()
  : Node("drone_safety_monitor")
  {
    state_estimate_timeout_ = airstack::get_param(this, "state_estimate_timeout", 1.0);

    // subscribers
    state_estimate_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "state_estimate", 10,
      std::bind(&DroneSafetyMonitorNode::state_estimate_callback, this, std::placeholders::_1));

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "command", 10,
      std::bind(&DroneSafetyMonitorNode::command_callback, this, std::placeholders::_1));

    // publishers
    state_estimate_timed_out_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("state_estimate_timed_out", 10);

    // service client
    traj_mode_client_ =
      this->create_client<airstack_msgs::srv::TrajectoryMode>("set_trajectory_mode");

    // timer at 1 Hz
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(1000),
      std::bind(&DroneSafetyMonitorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "DroneSafetyMonitorNode started. "
      "Publish to 'command' topic with: pause | resume | rewind");
  }

private:
  float state_estimate_timeout_;
  bool was_timed_out_{false};

  TimeChecker state_estimate_time_checker_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimate_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_estimate_timed_out_pub_;
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void set_trajectory_mode(int32_t mode)
  {
    if (!traj_mode_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "set_trajectory_mode service not available");
      return;
    }
    auto req = std::make_shared<airstack_msgs::srv::TrajectoryMode::Request>();
    req->mode = mode;
    traj_mode_client_->async_send_request(req);
  }

  void state_estimate_callback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    state_estimate_time_checker_.update(this->get_clock()->now());
  }

  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & cmd = msg->data;
    if (cmd == "pause") {
      RCLCPP_INFO(this->get_logger(), "Safety command: PAUSE");
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::PAUSE);
    } else if (cmd == "resume") {
      RCLCPP_INFO(this->get_logger(), "Safety command: RESUME");
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::ROBOT_POSE);
    } else if (cmd == "rewind") {
      RCLCPP_INFO(this->get_logger(), "Safety command: REWIND");
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::REWIND);
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Unknown safety command: '%s'. Valid commands: pause | resume | rewind", cmd.c_str());
    }
  }

  void timer_callback()
  {
    bool timed_out =
      state_estimate_time_checker_.elapsed_since_last_update(this->get_clock()->now()) >
      state_estimate_timeout_;

    std_msgs::msg::Bool msg;
    msg.data = timed_out;
    state_estimate_timed_out_pub_->publish(msg);

    // auto-pause if state estimate just timed out
    if (timed_out && !was_timed_out_) {
      RCLCPP_WARN(this->get_logger(),
        "State estimate timed out! Auto-pausing trajectory controller.");
      set_trajectory_mode(airstack_msgs::srv::TrajectoryMode::Request::PAUSE);
    }
    was_timed_out_ = timed_out;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneSafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
