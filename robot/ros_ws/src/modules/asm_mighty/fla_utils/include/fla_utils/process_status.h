#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <unistd.h>                      // for getpid()
#include <rclcpp/rclcpp.hpp>
#include "fla_interfaces/msg/process_status.hpp"

namespace fla_utils {

/// \brief Publishes a /globalstatus heartbeat on the user’s rclcpp::Node executor.
class ProcessStatus
{
public:
  /// \param parent  the node under whose executor & clock we publish
  /// \param id      an application-specific status ID
  /// \param rate_hz publication rate in Hz
  ProcessStatus(
    rclcpp::Node::SharedPtr parent,
    uint8_t                     id,
    double                      rate_hz)
  : parent_(std::move(parent)),
    id_(id),
    publish_rate_(rate_hz),
    running_(true)
  {
    status_pub_ = parent_->create_publisher<
      fla_interfaces::msg::ProcessStatus>(
        "/globalstatus", rclcpp::QoS(1));
    status_thread_ = std::thread(&ProcessStatus::Heartbeat, this);
  }

  ~ProcessStatus()
  {
    // signal the thread to stop, then join
    running_.store(false, std::memory_order_relaxed);
    if (status_thread_.joinable()) {
      status_thread_.join();
    }
  }

  /// Update the “arg” field in the next heartbeat.
  void SetArg(uint8_t a)
  {
    std::lock_guard<std::mutex> lg(mutex_);
    arg_ = a;
  }

  /// Update the “status” field in the next heartbeat.
  void SetStatus(uint8_t s)
  {
    std::lock_guard<std::mutex> lg(mutex_);
    status_ = s;
  }

private:
  void Heartbeat()
  {
    // run until either ROS shuts down or we’re asked to stop
    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      fla_interfaces::msg::ProcessStatus ps;
      ps.id     = id_;
      ps.pid    = static_cast<uint32_t>(::getpid());

      {
        std::lock_guard<std::mutex> lg(mutex_);
        ps.status = status_;
        ps.arg    = arg_;
      }

      status_pub_->publish(ps);
      publish_rate_.sleep();
    }
  }

  rclcpp::Node::SharedPtr                                           parent_;
  rclcpp::Publisher<fla_interfaces::msg::ProcessStatus>::SharedPtr  status_pub_;
  std::thread                                                      status_thread_;
  std::atomic<bool>                                                running_;
  std::mutex                                                       mutex_;
  rclcpp::Rate                                                     publish_rate_;
  uint8_t                                                          id_{0}, status_{0}, arg_{0};
};

}  // namespace fla_utils
