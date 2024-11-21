#pragma once

#include <algorithm>
#include <behavior_tree_msgs/msg/active.hpp>
#include <behavior_tree_msgs/msg/status.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

namespace bt {

class Condition {
   private:
    std_msgs::msg::Bool success;
    std::string label;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_pub;

   public:
    Condition(std::string label, rclcpp::Node* node);
    void set(bool b);
    void publish();
    bool get();
    std::string get_label();

    static std::string get_published_topic_name(std::string label);
};

class Action {
   private:
    bool active, prev_active, active_changed;
    uint64_t current_id;
    behavior_tree_msgs::msg::Status status;
    std::string label;

    rclcpp::Subscription<behavior_tree_msgs::msg::Active>::SharedPtr active_sub;
    rclcpp::Publisher<behavior_tree_msgs::msg::Status>::SharedPtr status_pub;

    void active_callback(const behavior_tree_msgs::msg::Active::SharedPtr msg);

    std::function<void()> active_callback_func;
    std::function<void()> inactive_callback_func;

    void init(std::string label, rclcpp::Node* node);

   public:
    Action(std::string label, rclcpp::Node* node);

    // constructor for active and inactive callback that are class members
    template <class ActiveClassType, class InactiveClassType>
    Action(std::string label, rclcpp::Node* node, void (ActiveClassType::*active_callback)(),
           ActiveClassType* active_class_object, void (InactiveClassType::*inactive_callback)(),
           InactiveClassType* inactive_class_object) {
        init(label, node);
        active_callback_func = std::bind(active_callback, active_class_object);
        inactive_callback_func = std::bind(inactive_callback, inactive_class_object);
    }

    // constructor for an active callback that is a class member
    template <class ActiveClassType>
    Action(std::string label, rclcpp::Node* node, void (ActiveClassType::*active_callback)(),
           ActiveClassType* active_class_object) {
        init(label, node);
        active_callback_func = std::bind(active_callback, active_class_object);
    }

    // constructor for active and inactive callbacks that are not class members. The inactive
    // callback is optional
    Action(std::string label, rclcpp::Node* node, void (*active_callback)(),
           void (*inactive_callback)() = NULL) {
        init(label, node);
        active_callback_func = active_callback;
        inactive_callback_func = inactive_callback;
    }

    // constructor for an active callback that is not a class member and inactive callback that is a
    // class member
    template <class InactiveClassType>
    Action(std::string label, rclcpp::Node* node, void (*active_callback)(),
           void (InactiveClassType::*inactive_callback)(),
           InactiveClassType* inactive_class_object) {
        init(label, node);
        active_callback_func = active_callback;
        inactive_callback_func = std::bind(inactive_callback, inactive_class_object);
    }

    // constructor for an active callback that is a class member and inactive callback that is not a
    // class member
    template <class ActiveClassType>
    Action(std::string label, rclcpp::Node* node, void (ActiveClassType::*active_callback)(),
           ActiveClassType* active_class_object, void (*inactive_callback)()) {
        init(label, node);
        active_callback_func = std::bind(active_callback, active_class_object);
        inactive_callback_func = inactive_callback;
    }

    static std::string get_published_topic_name(std::string label);
    static std::string get_subscribed_topic_name(std::string label);

    void set_success();
    void set_running();
    void set_failure();

    bool is_active();
    bool active_has_changed();

    bool is_success();
    bool is_running();
    bool is_failure();

    std::string get_label();

    void publish();
};

};  // namespace bt
