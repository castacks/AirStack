#pragma once

#include <stdint.h>

#include <behavior_tree_msgs/msg/active.hpp>
#include <behavior_tree_msgs/msg/status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>
#include <vector>

//===============================================================================================
// ------------------------------------- Base Node Class ----------------------------------------
//===============================================================================================

class Node {
   private:
   public:
    static double max_wait_time;

    std::string label;
    bool is_active;
    uint8_t status;
    std::vector<Node*> children;

    virtual void add_child(Node*);
    virtual bool tick(bool active, int traversal_count) = 0;
};

//===============================================================================================
// ------------------------------------ Control Flow Nodes  -------------------------------------
//===============================================================================================

class ControlFlowNode : public Node {
   private:
   public:
};

class FallbackNode : public ControlFlowNode {
   private:
   public:
    FallbackNode();
    bool tick(bool active, int traversal_count) override;
};

class SequenceNode : public ControlFlowNode {
   private:
   public:
    SequenceNode();
    bool tick(bool active, int traversal_count) override;
};

class ParallelNode : public ControlFlowNode {
   private:
   public:
    int child_success_threshold;
    ParallelNode(int child_success_threshold);
    bool tick(bool active, int traversal_count) override;
};

//===============================================================================================
// -------------------------------------- Execution Nodes  --------------------------------------
//===============================================================================================

class ExecutionNode : public Node {
   private:
   public:
    rclcpp::Time status_modification_time;
    int current_traversal_count;
    rclcpp::Node* node;
    // rclcpp::Subscription subscriber;
    // rclcpp::Publisher publisher;

    ExecutionNode(rclcpp::Node* node);
    void init_ros();
    virtual std::string get_publisher_name() = 0;
    virtual void init_publisher() = 0;
    virtual std::string get_subscriber_name() = 0;
    virtual void init_subscriber() = 0;
    void set_status(uint8_t status);
    double get_status_age();
};

class ConditionNode : public ExecutionNode {
   private:
    bool callback_status_updated;
    // rclcpp::Node* node;

   public:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber;
    // rclcpp::Publisher<>::SharedPtr publisher;

    ConditionNode(std::string label, rclcpp::Node* node);
    bool tick(bool active, int traversal_count) override;
    std::string get_publisher_name() override;
    void init_publisher() override;
    std::string get_subscriber_name() override;
    void init_subscriber() override;
    void callback(const std_msgs::msg::Bool::SharedPtr msg);
};

class ActionNode : public ExecutionNode {
   private:
    int current_id;
    bool publisher_initialized;
    bool callback_status_updated;

   public:
    bool is_newly_active;

    // rclcpp::Node* node;

    rclcpp::Subscription<behavior_tree_msgs::msg::Status>::SharedPtr subscriber;
    rclcpp::Publisher<behavior_tree_msgs::msg::Active>::SharedPtr publisher;

    ActionNode(std::string label, rclcpp::Node* node);
    bool tick(bool active, int traversal_count) override;
    std::string get_publisher_name() override;
    void init_publisher() override;
    std::string get_subscriber_name() override;
    void init_subscriber() override;
    void callback(const behavior_tree_msgs::msg::Status::SharedPtr msg);
    void publish_active_msg(int active_id);
};

//===============================================================================================
// -------------------------------------- Decorator Nodes  --------------------------------------
//===============================================================================================

class DecoratorNode : public Node {
   private:
   public:
    void add_child(Node* node) override;
};

class NotNode : public DecoratorNode {
   private:
   public:
    NotNode();
    void add_child(Node* node) override;
    bool tick(bool active, int traversal_count) override;
};

//===============================================================================================
// --------------------------------------- Behavior Tree  ---------------------------------------
//===============================================================================================

class BehaviorTree {
   public:
    rclcpp::Node* ros2_node;
    std::string config_filename;
    std::vector<Node*> nodes;
    Node* root;
    int traversal_count;
    std::unordered_map<std::string, int> active_ids;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_actions_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graphviz_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compressed_pub;
    bool first_tick;

    void parse_config();
    int count_tabs(std::string str);
    std::string strip_space(std::string str);
    std::string strip_brackets(std::string str);
    std::vector<int> get_arguments(std::string str);
    std::string get_graphviz();
    // public:
    BehaviorTree(std::string config_filename, rclcpp::Node* node);
    bool tick();
};
