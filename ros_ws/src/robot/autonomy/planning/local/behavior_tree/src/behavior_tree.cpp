#include <behavior_tree/behavior_tree.h>

namespace bt {

// ==============================================================================
// -------------------------------- Condition -----------------------------------
// ==============================================================================

std::string Condition::get_published_topic_name(std::string label) {
    std::string topic_name = label;
    std::transform(topic_name.begin(), topic_name.end(), topic_name.begin(), ::tolower);
    std::replace(topic_name.begin(), topic_name.end(), ' ', '_');
    topic_name += "_success";
    return topic_name;
}

Condition::Condition(std::string label, rclcpp::Node::SharedPtr node) : label(label) {
    success_pub =
        node->create_publisher<std_msgs::msg::Bool>(Condition::get_published_topic_name(label), 10);
    success.data = false;
}

std::string Condition::get_label() { return label; }

void Condition::set(bool b) { success.data = b; }

bool Condition::get() { return success.data; }

void Condition::publish() { success_pub->publish(success); }

// ==============================================================================
// --------------------------------- Action -------------------------------------
// ==============================================================================

std::string Action::get_published_topic_name(std::string label) {
    std::string topic_name = label;
    std::transform(topic_name.begin(), topic_name.end(), topic_name.begin(), ::tolower);
    std::replace(topic_name.begin(), topic_name.end(), ' ', '_');
    topic_name += "_status";
    return topic_name;
}

std::string Action::get_subscribed_topic_name(std::string label) {
    std::string topic_name = label;
    std::transform(topic_name.begin(), topic_name.end(), topic_name.begin(), ::tolower);
    std::replace(topic_name.begin(), topic_name.end(), ' ', '_');
    topic_name += "_active";
    return topic_name;
}

Action::Action(std::string label, rclcpp::Node::SharedPtr node) : label(label) {
    active_sub = node->create_subscription<std_msgs::msg::Bool>(
        Action::get_subscribed_topic_name(label), 10,
        std::bind(&Action::active_callback, this, std::placeholders::_1));
    status_pub = node->create_publisher<behavior_tree_msgs::msg::Status>(
        Action::get_published_topic_name(label), 1);

    active = false;
    prev_active = false;
    active_changed = false;
    status.status = behavior_tree_msgs::msg::Status::FAILURE;
}

// void Action::active_callback(std_msgs::msg::Bool msg) { active = msg.data; }
void Action::active_callback(const std_msgs::msg::Bool::SharedPtr msg) { active = msg->data; }

void Action::set_success() { status.status = behavior_tree_msgs::msg::Status::SUCCESS; }

void Action::set_running() { status.status = behavior_tree_msgs::msg::Status::RUNNING; }

void Action::set_failure() { status.status = behavior_tree_msgs::msg::Status::FAILURE; }

bool Action::is_active() {
    active_changed = prev_active != active;
    prev_active = active;
    return active;
}

bool Action::active_has_changed() { return active_changed; }

bool Action::is_success() { return status.status == behavior_tree_msgs::msg::Status::SUCCESS; }

bool Action::is_running() { return status.status == behavior_tree_msgs::msg::Status::RUNNING; }

bool Action::is_failure() { return status.status == behavior_tree_msgs::msg::Status::FAILURE; }

std::string Action::get_label() { return label; }

void Action::publish() { status_pub->publish(status); }

};  // namespace bt
