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

#include <chrono>
#include <cmath>
#include <future>
#include <optional>
#include <stdexcept>

using namespace std::chrono_literals;

class DroneSafetyMonitorNode : public rclcpp::Node
{
public:
  DroneSafetyMonitorNode()
  : Node("drone_safety_monitor")
  {
    state_timeout_s_ = positive_parameter("state_estimate_timeout", 1.0);
    receipt_timeout_s_ = positive_parameter("receipt_timeout_wall_s", 2.0);
    response_timeout_s_ = positive_parameter("pause_response_timeout_wall_s", 0.75);
    retry_period_s_ = positive_parameter("pause_retry_period_wall_s", 1.0);
    const double period = positive_parameter("health_publish_period_s", 0.25);
    if (period > 1.0 || response_timeout_s_ > retry_period_s_) {
      throw std::invalid_argument("health period must be <=1s and response timeout <=retry period");
    }
    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "state_estimate", 1,
      std::bind(&DroneSafetyMonitorNode::state_callback, this, std::placeholders::_1));
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10,
      std::bind(&DroneSafetyMonitorNode::command_callback, this, std::placeholders::_1));
    timed_out_pub_ = create_publisher<std_msgs::msg::Bool>("state_estimate_timed_out", 10);
    pause_ack_pub_ = create_publisher<std_msgs::msg::Bool>("pause_acknowledged", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("state_health_reason", 10);
    mode_client_ = create_client<Mode>("set_trajectory_mode");
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&DroneSafetyMonitorNode::timer_callback, this));
    RCLCPP_INFO(get_logger(),
      "Health wall period %.3fs; source/receipt ROS timeout %.3fs; receipt wall timeout %.3fs",
      period, state_timeout_s_, receipt_timeout_s_);
    latch_pause();
  }

private:
  using Mode = airstack_msgs::srv::TrajectoryMode;
  using Steady = std::chrono::steady_clock;
  double state_timeout_s_;
  double receipt_timeout_s_;
  double response_timeout_s_;
  double retry_period_s_;
  int64_t last_clock_ns_{-1};
  int64_t last_source_ns_{0};
  int64_t receipt_ros_ns_{0};
  Steady::time_point receipt_wall_{};
  bool have_state_{false};
  bool fault_active_{false};
  bool pause_pending_{false};
  bool pause_acknowledged_{false};
  bool operator_superseded_{false};
  bool operator_timed_out_{false};
  std::string invalid_reason_{"waiting_for_state"};
  std::string reported_reason_;
  std::optional<rclcpp::Client<Mode>::FutureAndRequestId> pause_future_;
  std::optional<rclcpp::Client<Mode>::FutureAndRequestId> operator_future_;
  Steady::time_point operator_request_wall_{};
  Steady::time_point request_wall_{};
  Steady::time_point next_attempt_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr timed_out_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Client<Mode>::SharedPtr mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  double positive_parameter(const std::string & name, double value)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return result;
  }

  int64_t observe_clock()
  {
    const int64_t now = get_clock()->now().nanoseconds();
    if (last_clock_ns_ >= 0 && now < last_clock_ns_) {
      have_state_ = false;
      last_source_ns_ = 0;
      invalid_reason_ = "clock_reset";
      if (!fault_active_) {latch_pause();}
      fault_active_ = true;
    }
    last_clock_ns_ = now;
    return now;
  }

  void latch_pause()
  {
    // PAUSE may overtake an operator RPC locally, but cannot cancel it remotely.
    // Keep its future so a late mode change can be followed by a new PAUSE.
    if (operator_future_) {operator_superseded_ = true;}
    // A new fault needs a new acknowledgement; duplicate PAUSE does not discard an RPC.
    if (!pause_pending_) {
      pause_pending_ = true;
      pause_acknowledged_ = false;
      next_attempt_ = Steady::now();
    }
  }

  void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const int64_t now = observe_clock();
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const auto & v = msg->twist.twist.linear;
    const auto & w = msg->twist.twist.angular;
    const double values[] = {p.x, p.y, p.z, q.x, q.y, q.z, q.w,
      v.x, v.y, v.z, w.x, w.y, w.z};
    invalid_reason_.clear();
    for (const double value : values) {
      if (!std::isfinite(value)) {invalid_reason_ = "nonfinite_state";}
    }
    const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (invalid_reason_.empty() && (norm2 < 0.98 || norm2 > 1.02)) {
      invalid_reason_ = "invalid_orientation";
    }
    const auto & stamp = msg->header.stamp;
    const int64_t source = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    if (now <= 0) {invalid_reason_ = "waiting_for_clock";} else if (
      stamp.sec < 0 || stamp.nanosec >= 1000000000 || source <= 0)
    {
      invalid_reason_ = "invalid_source_stamp";
    } else if (source - now > 100000000LL) {
      invalid_reason_ = "future_source_stamp";
    } else if ((now - source) * 1e-9 > state_timeout_s_) {
      invalid_reason_ = "stale_source_stamp";
    } else if (last_source_ns_ > 0 && source < last_source_ns_) {
      invalid_reason_ = "backwards_source_stamp";
    }
    if (!invalid_reason_.empty()) {
      // Do not let invalid arrivals refresh either accepted-state deadline.
      if (!fault_active_) {latch_pause();}
      fault_active_ = true;
      return;
    }
    // Duplicate/replayed stamps must not extend the accepted receipt deadline,
    // including when /clock itself has stopped advancing.
    if (have_state_ && source == last_source_ns_) {return;}
    have_state_ = true;
    last_source_ns_ = source;
    receipt_ros_ns_ = now;
    receipt_wall_ = Steady::now();
  }

  std::string health_reason(int64_t now)
  {
    if (!invalid_reason_.empty()) {return invalid_reason_;}
    if (!have_state_) {return "waiting_for_state";}
    if (now <= 0) {return "waiting_for_clock";}
    if ((now - last_source_ns_) * 1e-9 > state_timeout_s_) {return "stale_source_stamp";}
    if ((now - receipt_ros_ns_) * 1e-9 > state_timeout_s_) {return "receipt_ros_timeout";}
    if (std::chrono::duration<double>(Steady::now() - receipt_wall_).count() >
      receipt_timeout_s_)
    {
      return "receipt_wall_timeout";
    }
    return "healthy";
  }

  void drive_pause()
  {
    auto now = Steady::now();
    if (operator_future_) {
      if (operator_future_->future.wait_for(0s) == std::future_status::ready) {
        try {
          RCLCPP_INFO(get_logger(), "Operator mode acknowledged=%d",
            operator_future_->future.get()->success);
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Operator mode reply failed: %s", e.what());
        }
        operator_future_.reset();
        operator_timed_out_ = false;
        if (operator_superseded_) {
          operator_superseded_ = false;
          // A PAUSE sent before this operator completion may have applied first.
          // Discard its local response and require a causally newer PAUSE reply.
          if (pause_future_) {
            mode_client_->remove_pending_request(pause_future_->request_id);
            pause_future_.reset();
          }
          pause_pending_ = false;
          latch_pause();
        }
      } else if (!operator_timed_out_ &&
        std::chrono::duration<double>(now - operator_request_wall_).count() >=
        response_timeout_s_)
      {
        operator_timed_out_ = true;
        latch_pause();
        RCLCPP_WARN(get_logger(),
          "Operator mode response timed out; retained for reconciliation, PAUSE required");
      }
    }
    // latch_pause may have set next_attempt_ during operator reconciliation.
    // Compare against a fresh instant so a newly required PAUSE is not deferred.
    now = Steady::now();
    if (pause_future_) {
      if (pause_future_->future.wait_for(0s) == std::future_status::ready) {
        try {
          const bool success = pause_future_->future.get()->success;
          if (success) {
            pause_pending_ = false;
            pause_acknowledged_ = !operator_superseded_;
            RCLCPP_INFO(get_logger(), "PAUSE acknowledged by controller; physical stop unverified");
          } else {
            RCLCPP_WARN(get_logger(), "PAUSE rejected; intervention remains pending");
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "PAUSE reply failed: %s", e.what());
        }
        pause_future_.reset();
      } else if (std::chrono::duration<double>(now - request_wall_).count() >=
        response_timeout_s_)
      {
        mode_client_->remove_pending_request(pause_future_->request_id);
        pause_future_.reset();
        RCLCPP_WARN(get_logger(), "PAUSE response timed out; intervention remains pending");
      }
    }
    if (!pause_pending_ || pause_future_ || now < next_attempt_) {return;}
    next_attempt_ = now + std::chrono::duration_cast<Steady::duration>(
      std::chrono::duration<double>(retry_period_s_));
    if (!mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "PAUSE service unavailable; intervention remains pending");
      return;
    }
    auto request = std::make_shared<Mode::Request>();
    request->mode = Mode::Request::PAUSE;
    try {
      pause_future_.emplace(mode_client_->async_send_request(request));
      request_wall_ = now;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "PAUSE submission failed: %s", e.what());
    }
  }

  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "pause") {
      RCLCPP_INFO(get_logger(), "Safety command: PAUSE");
      latch_pause();
      drive_pause();
      return;
    }
    if (msg->data != "resume" && msg->data != "rewind") {
      RCLCPP_WARN(get_logger(), "Unknown safety command: '%s'", msg->data.c_str());
      return;
    }
    const auto reason = health_reason(observe_clock());
    if (reason != "healthy" || pause_pending_ || !pause_acknowledged_ || operator_future_) {
      RCLCPP_WARN(get_logger(), "Safety command blocked: %s (health=%s, pause pending=%d)",
        msg->data.c_str(), reason.c_str(), pause_pending_);
      return;
    }
    if (!mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Safety command service unavailable");
      return;
    }
    // No deferred resume/rewind retries: they could arrive after a new safety fault.
    auto request = std::make_shared<Mode::Request>();
    request->mode = msg->data == "resume" ? Mode::Request::ROBOT_POSE : Mode::Request::REWIND;
    try {
      operator_future_.emplace(mode_client_->async_send_request(request));
      operator_request_wall_ = Steady::now();
      operator_timed_out_ = false;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Operator mode submission failed: %s", e.what());
    }
  }

  void timer_callback()
  {
    const auto reason = health_reason(observe_clock());
    const bool fault = reason != "healthy";
    if (fault && !fault_active_) {latch_pause();}
    fault_active_ = fault;
    drive_pause();
    const auto published_reason = !fault && operator_superseded_ ? "operator_request_unresolved" :
      (!fault && pause_pending_ ? "pause_pending" : reason);
    if (published_reason != reported_reason_) {
      RCLCPP_WARN(get_logger(), "State health: %s", published_reason.c_str());
      reported_reason_ = published_reason;
    }
    std_msgs::msg::Bool timeout;
    // Existing task consumers must not treat fresh state as intervention readiness.
    timeout.data = fault || pause_pending_ || operator_superseded_;
    timed_out_pub_->publish(timeout);
    std_msgs::msg::Bool ack;
    ack.data = pause_acknowledged_ && !pause_pending_ && !operator_superseded_;
    pause_ack_pub_->publish(ack);
    std_msgs::msg::String detail;
    detail.data = published_reason;
    reason_pub_->publish(detail);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneSafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
