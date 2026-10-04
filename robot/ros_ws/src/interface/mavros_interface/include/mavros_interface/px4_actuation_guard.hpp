// Copyright (c) 2026 Carnegie Mellon University. BSD-3-Clause-Clear.
#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>

namespace mavros_interface {
// PX4 profile guard. Startup writes are allowed only with fresh grounded evidence;
// command admission is read-only and never trusts a previous successful readback.
class Px4ActuationGuard {
  using Clock = std::chrono::steady_clock;
  using Get = rcl_interfaces::srv::GetParameters;
  using Set = rcl_interfaces::srv::SetParameters;
  rclcpp::Node &node_;
  rclcpp::CallbackGroup::SharedPtr clients_group_, timer_group_;
  rclcpp::Client<Get>::SharedPtr get_;
  rclcpp::Client<Set>::SharedPtr set_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ground_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex state_mutex_;
  std::timed_mutex query_mutex_;
  mavros_msgs::msg::State state_;
  uint8_t landed_ = 0;
  Clock::time_point state_at_{}, ground_at_{}, verified_at_{};
  const Clock::time_point started_ = Clock::now();
  double startup_timeout_ = 10.0;
  bool config_valid_ = false, attempted_ = false;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> unsafe_seen_{false};

  template<class S>
  typename S::Response::SharedPtr call(
      const typename rclcpp::Client<S>::SharedPtr &client,
      const typename S::Request::SharedPtr &request, bool grounded_write = false) {
    if (!client->wait_for_service(std::chrono::milliseconds(250))) { return nullptr; }
    if (grounded_write && !safe_to_configure()) { return nullptr; }
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
      client->remove_pending_request(future);
      return nullptr;
    }
    return future.get();
  }

  bool safe_to_configure() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = Clock::now();
    return !unsafe_seen_ && state_at_ != Clock::time_point{} && ground_at_ != Clock::time_point{} &&
      now - state_at_ <= std::chrono::milliseconds(500) &&
      now - ground_at_ <= std::chrono::milliseconds(500) &&
      state_.connected && !state_.armed &&
      landed_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }

  bool readback() {
    auto request = std::make_shared<Get::Request>();
    request->names = {"thrust_scaling"};
    auto response = call<Get>(get_, request);
    const bool valid = response && response->values.size() == 1 &&
      response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE &&
      std::isfinite(response->values[0].double_value) && response->values[0].double_value == 1.0;
    std::lock_guard<std::mutex> lock(state_mutex_);
    verified_at_ = valid ? Clock::now() : Clock::time_point{};
    return valid;
  }

  void tick() {
    try {
      std::unique_lock<std::timed_mutex> query_lock(query_mutex_, std::try_to_lock);
      if (!query_lock.owns_lock()) { return; }
      if (config_valid_ && !attempted_) {
        if (unsafe_seen_ || std::chrono::duration<double>(Clock::now() - started_).count() >= startup_timeout_) {
          attempted_ = true;
          RCLCPP_ERROR(node_.get_logger(), "Actuation NOT_READY: no safe startup evidence before deadline");
        } else if (safe_to_configure()) {
          // Wait for service before the final safety check; no delayed write after
          // waiting on discovery. This guard never writes again after an attempt.
          if (!set_->wait_for_service(std::chrono::milliseconds(250))) { return; }
          if (!safe_to_configure()) { return; }
          attempted_ = true;
          auto request = std::make_shared<Set::Request>();
          rcl_interfaces::msg::Parameter parameter;
          parameter.name = "thrust_scaling";
          parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          parameter.value.double_value = 1.0;
          request->parameters = {parameter};
          auto response = call<Set>(set_, request, true);
          initialized_ = response && response->results.size() == 1 &&
            response->results[0].successful && readback() && safe_to_configure();
          RCLCPP_INFO(node_.get_logger(), "PX4 actuation startup: %s",
            initialized_ ? "VERIFIED" : "NOT_READY (restart required)");
        }
      } else if (initialized_) {
        readback();  // Only observe after startup; never repair an active vehicle.
      }
    } catch (const std::exception &error) {
      attempted_ = true; initialized_ = false;
      RCLCPP_ERROR(node_.get_logger(), "Actuation NOT_READY: %s", error.what());
    }
    std_msgs::msg::Bool message;
    message.data = ready();
    ready_pub_->publish(message);
  }

public:
  explicit Px4ActuationGuard(rclcpp::Node &node) : node_(node) {
    try {
      auto path = node_.declare_parameter<std::string>("mavros_actuation_config", "");
      if (path.empty()) {
        path = ament_index_cpp::get_package_share_directory("interface_bringup") + "/config/px4_config.yaml";
      }
      startup_timeout_ = node_.declare_parameter<double>("actuation_startup_timeout_s", 10.0);
      const auto value = YAML::LoadFile(path)["/**/setpoint_raw"]["ros__parameters"]["thrust_scaling"];
      // Require the explicit normalized floating-point profile, not coercible
      // strings, booleans, integers or an arbitrary deployment scaling factor.
      const auto scalar = value.IsScalar() ? value.Scalar() : "";
      config_valid_ = value.IsScalar() && (value.Tag() == "?" ||
        value.Tag() == "tag:yaml.org,2002:float") &&
        scalar.find_first_of(".eE") != std::string::npos &&
        std::isfinite(value.as<double>()) && value.as<double>() == 1.0 &&
        std::isfinite(startup_timeout_) && startup_timeout_ > 0.0 && startup_timeout_ <= 60.0;
      if (!config_valid_) { throw std::runtime_error("invalid normalized PX4 actuation profile"); }
    } catch (const std::exception &error) {
      config_valid_ = false; attempted_ = true;
      RCLCPP_ERROR(node_.get_logger(), "Actuation config rejected: %s", error.what());
    }
    clients_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    get_ = node_.create_client<Get>("mavros/setpoint_raw/get_parameters",
      rmw_qos_profile_services_default, clients_group_);
    set_ = node_.create_client<Set>("mavros/setpoint_raw/set_parameters",
      rmw_qos_profile_services_default, clients_group_);
    state_sub_ = node_.create_subscription<mavros_msgs::msg::State>("mavros/state", 10,
      [this](mavros_msgs::msg::State::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = *message; state_at_ = Clock::now();
        if (!initialized_ && message->armed) { unsafe_seen_ = true; }
      });
    ground_sub_ = node_.create_subscription<mavros_msgs::msg::ExtendedState>("mavros/extended_state", 10,
      [this](mavros_msgs::msg::ExtendedState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        landed_ = message->landed_state; ground_at_ = Clock::now();
        if (!initialized_ && landed_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR) {
          unsafe_seen_ = true;
        }
      });
    ready_pub_ = node_.create_publisher<std_msgs::msg::Bool>("actuation_ready", 10);
    timer_ = node_.create_wall_timer(std::chrono::milliseconds(200),
      [this]() { tick(); }, timer_group_);
  }

  bool ready() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return initialized_ && verified_at_ != Clock::time_point{} &&
      Clock::now() - verified_at_ <= std::chrono::milliseconds(500);
  }

  bool admit() {
    if (!initialized_) { return false; }
    std::unique_lock<std::timed_mutex> lock(query_mutex_, std::defer_lock);
    if (!lock.try_lock_for(std::chrono::milliseconds(100))) { return false; }
    try { return readback(); }
    catch (const std::exception &) { return false; }
  }
};
}  // namespace mavros_interface
