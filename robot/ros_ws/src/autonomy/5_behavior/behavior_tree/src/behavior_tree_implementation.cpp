#include <ctype.h>

#include <algorithm>
#include <behavior_tree/behavior_tree_implementation.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <fstream>
#include <sstream>
#include <typeinfo>

//===============================================================================================
// ------------------------------------- Base Node Class ----------------------------------------
//===============================================================================================

double Node::max_wait_time = 1.0;

void Node::add_child(Node* node) { children.push_back(node); }

//===============================================================================================
// ------------------------------------ Control Flow Nodes  -------------------------------------
//===============================================================================================

FallbackNode::FallbackNode() {
    label = "?";
    is_active = false;
    status = behavior_tree_msgs::msg::Status::FAILURE;
}

bool FallbackNode::tick(bool active, int traversal_count) {
    uint8_t prev_status = status;
    bool child_changed = false;

    if (!active) {
        // TODO: is ticking children necessary if inactive?
        for (int i = 0; i < children.size(); i++)
            child_changed |= children[i]->tick(false, traversal_count);
    } else {
        status = behavior_tree_msgs::msg::Status::FAILURE;
        for (int i = 0; i < children.size(); i++) {
            Node* child = children[i];
            if (status == behavior_tree_msgs::msg::Status::FAILURE) {
                child_changed |= child->tick(true, traversal_count);
                status = child->status;
            } else
                child_changed |= child->tick(false, traversal_count);
        }
    }

    bool status_changed = (status != prev_status) || child_changed;
    bool active_changed = (is_active != active);

    is_active = active;

    return status_changed || active_changed;
}

SequenceNode::SequenceNode() {
    label = "->";  //"\u2192"; // unicode arrow character
    is_active = false;
    status = behavior_tree_msgs::msg::Status::FAILURE;
}

bool SequenceNode::tick(bool active, int traversal_count) {
    uint8_t prev_status = status;
    bool child_changed = false;

    if (!active) {
        for (int i = 0; i < children.size(); i++)
            child_changed |= children[i]->tick(false, traversal_count);

    } else {
        status = behavior_tree_msgs::msg::Status::SUCCESS;
        for (int i = 0; i < children.size(); i++) {
            Node* child = children[i];
            if (status == behavior_tree_msgs::msg::Status::SUCCESS) {
                child_changed |= child->tick(true, traversal_count);
                status = child->status;
            } else
                child_changed |= child->tick(false, traversal_count);
        }
    }

    bool status_changed = (status != prev_status) || child_changed;
    bool active_changed = (is_active != active);

    is_active = active;

    return status_changed || active_changed;
}

ParallelNode::ParallelNode(int child_success_threshold) {
    label = "||";
    is_active = false;
    status = behavior_tree_msgs::msg::Status::FAILURE;
    this->child_success_threshold = child_success_threshold;
}

bool ParallelNode::tick(bool active, int traversal_count) {
    uint8_t prev_status = status;
    bool child_changed = false;

    if (!active) {
        for (int i = 0; i < children.size(); i++)
            child_changed |= children[i]->tick(false, traversal_count);
    } else {
        int child_success_count = 0;
        int child_failure_count = 0;
        for (int i = 0; i < children.size(); i++) {
            Node* child = children[i];
            child_changed |= child->tick(true, traversal_count);

            if (child->status == behavior_tree_msgs::msg::Status::SUCCESS)
                child_success_count++;
            else if (child->status == behavior_tree_msgs::msg::Status::FAILURE)
                child_failure_count++;
        }

        if (child_success_count >= child_success_threshold)
            status = behavior_tree_msgs::msg::Status::SUCCESS;
        else if (child_failure_count >= (children.size() - child_success_threshold + 1))
            status = behavior_tree_msgs::msg::Status::FAILURE;
        else
            status = behavior_tree_msgs::msg::Status::RUNNING;
    }

    bool status_changed = (status != prev_status) || child_changed;
    bool active_changed = (is_active != active);

    is_active = active;

    return status_changed || active_changed;
}

//===============================================================================================
// -------------------------------------- Execution Nodes  --------------------------------------
//===============================================================================================

ExecutionNode::ExecutionNode(rclcpp::Node* node) : node(node) {}

void ExecutionNode::init_ros() {
    init_publisher();
    init_subscriber();
}

void ExecutionNode::set_status(uint8_t status) {
    this->status = status;
    status_modification_time = node->get_clock()->now();  // rclcpp::Time::now();
}

double ExecutionNode::get_status_age() {
    // return (rclcpp::Time::now() - status_modification_time).toSec();
    return (node->get_clock()->now() - status_modification_time).seconds();
}

ConditionNode::ConditionNode(std::string label, rclcpp::Node* node) : ExecutionNode(node) {
    this->label = label;
    status_modification_time = node->get_clock()->now();  // rclcpp::Time::now();
    is_active = false;
    status = behavior_tree_msgs::msg::Status::FAILURE;
    current_traversal_count = -1;
    callback_status_updated = false;
}

bool ConditionNode::tick(bool active, int traversal_count) {
    // if(active)
    //   ROS_INFO_STREAM("Condition " << label << " active");

    uint8_t prev_status = status;
    bool child_changed = false;

    if (current_traversal_count != traversal_count)
        current_traversal_count = traversal_count;
    else if (is_active)
        return false;  // TODO: is this correct?

    if (!is_active && active && status == behavior_tree_msgs::msg::Status::FAILURE)
        set_status(behavior_tree_msgs::msg::Status::FAILURE);

    if (get_status_age() > Node::max_wait_time)
        set_status(behavior_tree_msgs::msg::Status::FAILURE);

    bool status_changed = (status != prev_status) || callback_status_updated;
    bool active_changed = (is_active != active);

    is_active = active;
    callback_status_updated = false;

    return status_changed || active_changed;
}

std::string ConditionNode::get_publisher_name() { return ""; }

void ConditionNode::init_publisher() {}

std::string ConditionNode::get_subscriber_name() {
    std::string name = label;
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    std::replace(name.begin(), name.end(), ' ', '_');
    name = name + "_success";
    // ROS_INFO_STREAM("condition get subscriber name: " << name << " " << label);
    return name;
}

void ConditionNode::init_subscriber() {
    // ros::NodeHandle nh;
    subscriber = node->create_subscription<std_msgs::msg::Bool>(
        get_subscriber_name(), 1, std::bind(&ConditionNode::callback, this, std::placeholders::_1));
}

void ConditionNode::callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // ROS_INFO_STREAM(label << ": " << (int)msg.data);
    //  TODO: bug where if status is set here the changed flag won't be set correctly in tick
    uint8_t prev_status = status;

    if (msg->data)
        set_status(behavior_tree_msgs::msg::Status::SUCCESS);
    else
        set_status(behavior_tree_msgs::msg::Status::FAILURE);

    if (status != prev_status) callback_status_updated = true;
}

ActionNode::ActionNode(std::string label, rclcpp::Node* node) : ExecutionNode(node) {
    this->label = label;
    status_modification_time = node->get_clock()->now();  // rclcpp::Time::now();
    is_active = false;
    status = behavior_tree_msgs::msg::Status::FAILURE;
    current_traversal_count = -1;
    is_newly_active = false;
    current_id = 0;
    publisher_initialized = false;
    callback_status_updated = false;
}

bool ActionNode::tick(bool active, int traversal_count) {
    // if(active)
    //   ROS_INFO_STREAM("Action " << label << " active");

    uint8_t prev_status = status;

    if (current_traversal_count != traversal_count)
        current_traversal_count = traversal_count;
    else if (is_active)
        return false;  // TOOD: is this correct?

    if (!is_active and active) {
        set_status(behavior_tree_msgs::msg::Status::RUNNING);
        is_newly_active = true;
    } else
        is_newly_active = false;

    if (get_status_age() > Node::max_wait_time)
        set_status(behavior_tree_msgs::msg::Status::FAILURE);

    bool status_changed = (status != prev_status) || callback_status_updated;
    bool active_changed = (is_active != active);

    is_active = active;
    callback_status_updated = false;

    return status_changed || active_changed;
}

std::string ActionNode::get_publisher_name() {
    std::string name = label;
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    std::replace(name.begin(), name.end(), ' ', '_');
    name = name + "_active";
    // ROS_INFO_STREAM("action get publisher name: " << name << " " << label);
    return name;
}

void ActionNode::init_publisher() {
    publisher = node->create_publisher<behavior_tree_msgs::msg::Active>(get_publisher_name(), 1);
    publisher_initialized = true;
}

std::string ActionNode::get_subscriber_name() {
    std::string name = label;
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    std::replace(name.begin(), name.end(), ' ', '_');
    name = name + "_status";
    // ROS_INFO_STREAM("action get subscriber name: " << name << " " << label);
    return name;
}

void ActionNode::init_subscriber() {
    subscriber = node->create_subscription<behavior_tree_msgs::msg::Status>(
        get_subscriber_name(), 1, std::bind(&ActionNode::callback, this, std::placeholders::_1));
}

void ActionNode::callback(const behavior_tree_msgs::msg::Status::SharedPtr msg) {
    uint8_t prev_status = status;
    if (is_active) {
        if (msg->id != current_id) {
            // ROS_ERROR_STREAM(label << " Incorrect ID " << msg->id << " " << current_id << " " <<
            // is_active);
            std::cout << label << " Incorrect ID " << msg->id << " " << current_id << " "
                      << is_active << std::endl;
            return;
        }
        set_status(msg->status);
    }

    if (status != prev_status) callback_status_updated = true;
}

void ActionNode::publish_active_msg(int active_id) {
    if (publisher_initialized) {
        behavior_tree_msgs::msg::Active active_msg;
        active_msg.active = is_active;
        active_msg.id = active_id;
        current_id = active_id;
        publisher->publish(active_msg);
    }
}

//===============================================================================================
// -------------------------------------- Decorator Nodes  --------------------------------------
//===============================================================================================

void DecoratorNode::add_child(Node* node) {
    if (children.size() == 0)
        children.push_back(node);
    else {
        // ROS_ERROR_STREAM("A decorator node can only have one child.");
        std::cout << "A decorator node can only have one child." << std::endl;
        exit(1);
    }
}

NotNode::NotNode() {
    label = "!";
    is_active = false;
    status = behavior_tree_msgs::msg::Status::FAILURE;
}

void NotNode::add_child(Node* node) {
    ConditionNode* condition_node = dynamic_cast<ConditionNode*>(node);
    if (condition_node != NULL)
        DecoratorNode::add_child(condition_node);
    else {
        // ROS_ERROR_STREAM("A not decorator node can only have a condition node as a child.");
        std::cout << "A not decorator node can only have a condition node as a child." << std::endl;
        exit(1);
    }
}

bool NotNode::tick(bool active, int traversal_count) {
    uint8_t prev_status = status;
    bool child_changed = false;

    if (children.size() > 0) {
        Node* child = children[0];
        child_changed |= child->tick(active, traversal_count);

        if (child->status == behavior_tree_msgs::msg::Status::SUCCESS)
            status = behavior_tree_msgs::msg::Status::FAILURE;
        else if (child->status == behavior_tree_msgs::msg::Status::FAILURE)
            status = behavior_tree_msgs::msg::Status::SUCCESS;
    }

    bool status_changed = (status != prev_status) || child_changed;
    bool active_changed = (is_active != active);

    is_active = active;

    return status_changed || active_changed;
}

//===============================================================================================
// --------------------------------------- Behavior Tree  ---------------------------------------
//===============================================================================================

BehaviorTree::BehaviorTree(std::string config_filename, rclcpp::Node* node) : ros2_node(node) {
    this->config_filename = config_filename;
    root = NULL;
    traversal_count = 0;
    first_tick = true;

    active_actions_pub = node->create_publisher<std_msgs::msg::String>("active_actions", 1);
    graphviz_pub = node->create_publisher<std_msgs::msg::String>("behavior_tree_graphviz", 1);
    compressed_pub =
        node->create_publisher<std_msgs::msg::String>("behavior_tree_graphviz_compressed", 1);

    parse_config();
    for (int i = 0; i < nodes.size(); i++) {
        ActionNode* action_node = dynamic_cast<ActionNode*>(nodes[i]);
        if (action_node != NULL) action_node->init_ros();
        ConditionNode* condition_node = dynamic_cast<ConditionNode*>(nodes[i]);
        if (condition_node != NULL) condition_node->init_ros();
    }
}

void BehaviorTree::parse_config() {
    std::ifstream in(config_filename);

    // ROS_INFO_STREAM("Config file: " << config_filename);

    if (in.is_open()) {
        std::vector<Node*> nodes_worklist;
        int prev_tabs = 0;

        std::string line;
        while (getline(in, line)) {
            // skip empty lines
            if (line.size() == 0) continue;

            // ROS_INFO_STREAM("Line: " << line);

            // initialization
            int curr_tabs = count_tabs(line);
            std::string label = strip_space(line);
            Node* node = NULL;

            // ROS_INFO_STREAM("curr tabs: " << curr_tabs << " label: " << label);

            // create a node of the right type
            if (label.compare(0, 2, "->") == 0)
                node = new SequenceNode();
            else if (label.compare(0, 1, "?") == 0)
                node = new FallbackNode();
            else if (label.compare(0, 2, "||") == 0) {
                std::vector<int> arguments = get_arguments(label);
                int child_success_threshold = 0;
                if (arguments.size() > 0)
                    child_success_threshold = arguments[0];
                else {
                    // ROS_ERROR_STREAM("Arguments not provided to parallel node");
                    std::cout << "Arguments not provided to parallel node" << std::endl;
                    exit(1);
                }
                node = new ParallelNode(child_success_threshold);
            } else if (label.compare(0, 1, "(") == 0)
                node = new ConditionNode(strip_brackets(label), ros2_node);
            else if (label.compare(0, 1, "[") == 0) {
                node = new ActionNode(strip_brackets(label), ros2_node);
                active_ids[node->label] = 0;
            } else if (label.compare(0, 1, "<") == 0) {
                std::string decorator_label = strip_brackets(label);
                if (decorator_label.compare(0, 1, "!") == 0) node = new NotNode();
            }

            if (node != NULL) {
                nodes.push_back(node);

                if (root == NULL) {
                    root = node;
                    nodes_worklist.push_back(node);
                    continue;
                }

                if (curr_tabs == prev_tabs + 1) {
                    Node* parent = nodes_worklist[nodes_worklist.size() - 1];
                    parent->add_child(node);
                } else {
                    for (int i = 0; i < prev_tabs - curr_tabs + 1; i++) nodes_worklist.pop_back();
                    Node* parent = nodes_worklist[nodes_worklist.size() - 1];
                    parent->add_child(node);
                }

                nodes_worklist.push_back(node);
                prev_tabs = curr_tabs;
            }
        }
        in.close();
    } else
        std::cout << "Failed to open behavior tree config file: " << config_filename << std::endl;
    // ROS_ERROR_STREAM("Failed to open behavior tree config file: " << config_filename);
}

int BehaviorTree::count_tabs(std::string str) {
    int count = 0;
    for (int i = 0; i < str.size(); i++)
        if (str[i] == '\t')
            count++;
        else
            break;

    return count;
}

std::string BehaviorTree::strip_space(std::string str) {
    std::string result = str;
    while (result.size() > 0)
        if (std::isspace(result[0]))
            result.erase(0, 1);
        else
            break;

    while (result.size() > 0)
        if (std::isspace(result[result.size() - 1]))
            result.erase(result.size() - 1, 1);
        else
            break;

    return result;
}

std::string BehaviorTree::strip_brackets(std::string str) {
    std::string result = str;
    while (result.size() > 0)
        if (result[0] == '(' || result[0] == '[' || result[0] == '<')
            result.erase(0, 1);
        else
            break;

    while (result.size() > 0)
        if (result[result.size() - 1] == ')' || result[result.size() - 1] == ']' ||
            result[result.size() - 1] == '>')
            result.erase(result.size() - 1, 1);
        else
            break;

    return result;
}

std::vector<int> BehaviorTree::get_arguments(std::string str) {
    std::vector<int> arguments;

    std::istringstream ss(str);
    std::string first;
    ss >> first;

    int i;
    while (ss >> i) arguments.push_back(i);

    return arguments;
}

bool BehaviorTree::tick() {
    bool changed = false || first_tick;
    first_tick = false;

    if (root != NULL) {
        changed = root->tick(true, traversal_count);
        traversal_count++;

        std_msgs::msg::String active_actions;

        std::unordered_map<std::string, Node*> unique_action_nodes;
        for (int i = 0; i < nodes.size(); i++) {
            ActionNode* action_node = dynamic_cast<ActionNode*>(nodes[i]);
            if (action_node != NULL) {
                if (unique_action_nodes.count(action_node->label) == 0 || action_node->is_active) {
                    unique_action_nodes[action_node->label] = action_node;
                    if (action_node->is_newly_active) active_ids[action_node->label]++;
                }
            }
        }

        for (auto it = unique_action_nodes.begin(); it != unique_action_nodes.end(); it++) {
            ActionNode* action_node = dynamic_cast<ActionNode*>(it->second);
            if (action_node != NULL) {
                action_node->publish_active_msg(active_ids[it->first]);

                if (it->second->is_active) active_actions.data += action_node->label + ", ";
            } else {
                std::cout << "action node is NULL, something went wrong." << std::endl;
                // ROS_ERROR_STREAM("action node is NULL, something went wrong.");
            }
        }
        if (active_actions.data.length() >= 2) {
            active_actions.data.pop_back();
            active_actions.data.pop_back();
        }
        active_actions_pub->publish(active_actions);
    }

    return changed;
}

std::string BehaviorTree::get_graphviz() {
    std::string gv = "digraph G {\n";

    std::vector<Node*> nodes_worklist;
    nodes_worklist.push_back(root);
    int count = 0;
    std::unordered_map<Node*, std::string> node_names;

    while (!nodes_worklist.empty()) {
        Node* node = nodes_worklist.back();
        nodes_worklist.pop_back();
        std::string name = "node_" + std::to_string(count);
        count++;

        std::string style = "";
        if (node->is_active) {
            style += "penwidth=2 color=black style=filled ";
            if (node->status == behavior_tree_msgs::msg::Status::SUCCESS)
                style += "fillcolor=green";
            else if (node->status == behavior_tree_msgs::msg::Status::RUNNING)
                style += "fillcolor=blue";
            else if (node->status == behavior_tree_msgs::msg::Status::FAILURE)
                style += "fillcolor=red";
        } else {
            style += "penwidth=2 ";
            if (node->status == behavior_tree_msgs::msg::Status::SUCCESS)
                style += "color=green";
            else if (node->status == behavior_tree_msgs::msg::Status::RUNNING)
                style += "color=blue";
            else if (node->status == behavior_tree_msgs::msg::Status::FAILURE)
                style += "color=red";
        }

        if (dynamic_cast<ConditionNode*>(node) != NULL) {
            gv += "\t" + name + " [label=\"" + node->label + "\" " + style + "]\n";
        } else if (dynamic_cast<ActionNode*>(node) != NULL) {
            gv += "\t" + name + " [label=\"" + node->label + "\" shape=square " + style + "]\n";
        } else if (dynamic_cast<FallbackNode*>(node) != NULL) {
            gv += "\t" + name + " [label=\"" + node->label + "\" shape=square " + style + "]\n";
        } else if (dynamic_cast<SequenceNode*>(node) != NULL) {
            gv += "\t" + name + " [label=\"" + node->label + "\" shape=square " + style + "]\n";
        } else if (dynamic_cast<ParallelNode*>(node) != NULL) {
            gv += "\t" + name + " [label=\"" + node->label + "\" shape=parallelogram " + style +
                  "]\n";
        } else if (dynamic_cast<DecoratorNode*>(node) != NULL) {
            gv += "\t" + name + " [label=\"" + node->label + "\" shape=diamond " + style + "]\n";
        }

        node_names[node] = name;

        for (int i = 0; i < node->children.size(); i++) nodes_worklist.push_back(node->children[i]);
    }

    gv += "\n\tordering=out;\n\n";

    nodes_worklist.push_back(root);
    while (!nodes_worklist.empty()) {
        Node* node = nodes_worklist.back();
        nodes_worklist.pop_back();

        for (int i = 0; i < node->children.size(); i++) {
            gv += "\t" + node_names[node] + " -> " + node_names[node->children[i]] + ";\n";
            nodes_worklist.push_back(node->children[i]);
        }
    }

    gv += "}\n";

    return gv;
}

std::string string_compress_encode(const std::string& data) {
    std::stringstream compressed;
    std::stringstream original;
    original << data;
    boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
    out.push(boost::iostreams::zlib_compressor());
    out.push(original);
    boost::iostreams::copy(out, compressed);

    /**need to encode here **/
    // std::string compressed_encoded = base64_encode(reinterpret_cast<const unsigned
    // char*>(compressed.c_str()), compressed.length());

    return compressed.str();  // compressed_encoded;
}

BehaviorTree* bt;
std_msgs::msg::String graphviz_msg;
std_msgs::msg::String compressed_msg;
bool only_publish_on_change;
void timer_callback() {  // const rclcpp::TimerEvent& te){
    bool changed = bt->tick();
    // ROS_INFO_STREAM("Changed: " << changed);

    if (changed) {
        graphviz_msg.data = bt->get_graphviz();
        compressed_msg.data = string_compress_encode(graphviz_msg.data);

        if (only_publish_on_change) {
            bt->graphviz_pub->publish(graphviz_msg);
            bt->compressed_pub->publish(compressed_msg);
        }
    }

    if (!only_publish_on_change) {
        bt->graphviz_pub->publish(graphviz_msg);
        bt->compressed_pub->publish(compressed_msg);
    }
}

class BehaviorTreeNode : public rclcpp::Node {
   private:
    rclcpp::TimerBase::SharedPtr timer;

   public:
    BehaviorTreeNode() : Node("behavior_tree_node") {
        // timer = this->create_timer(std::chrono::milliseconds(50),
        // std::bind(&MinimalPublisher::timer_callback, this));

        this->declare_parameter("config", "");
        std::string config_filename = this->get_parameter("config").as_string();

        this->declare_parameter("timeout", 1.0);
        ::Node::max_wait_time = this->get_parameter("timeout").as_double();

        bt = new BehaviorTree(config_filename, this);

        timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(50),
                                     &timer_callback);
    }
};

int main(int argc, char** argv) {
    /*
    ros::init(argc, argv, "behavior_tree");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string config_filename = pnh.param("config", std::string(""));
    only_publish_on_change = pnh.param("only_publish_on_change", false);
    Node::max_wait_time = pnh.param("timeout", 1.0);

    rclcpp::Timer timer = nh.createTimer(ros::Duration(0.05), timer_callback);

    bt = new BehaviorTree(config_filename);

    ros::spin();
    */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorTreeNode>());
    rclcpp::shutdown();

    return 0;
}
