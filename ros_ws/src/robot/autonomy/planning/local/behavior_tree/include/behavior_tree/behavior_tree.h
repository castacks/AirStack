#ifndef _BEHAVIOR_TREE_H_
#define _BEHAVIOR_TREE_H_

#include <algorithm>
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
    Condition(std::string label, rclcpp::Node::SharedPtr node);
    void set(bool b);
    void publish();
    bool get();
    std::string get_label();

    static std::string get_published_topic_name(std::string label);
};

class Action {
   private:
    bool active, prev_active, active_changed;
    behavior_tree_msgs::msg::Status status;
    std::string label;

    // ros::Subscriber active_sub;
    // ros::Publisher status_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub;
    rclcpp::Publisher<behavior_tree_msgs::msg::Status>::SharedPtr status_pub;

    void active_callback(const std_msgs::msg::Bool::SharedPtr msg);

   public:
    Action(std::string label, rclcpp::Node::SharedPtr node);

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

#endif
