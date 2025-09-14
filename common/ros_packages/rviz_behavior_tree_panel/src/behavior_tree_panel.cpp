/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, AirLab - Carnegie Mellon University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AirLab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <rviz_behavior_tree_panel/behavior_tree_panel.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_behavior_tree_panel
{

BehaviorTreePanel::BehaviorTreePanel(QWidget * parent)
: Panel(parent), layout_(nullptr), topic_layout_(nullptr), topic_label_(nullptr), 
  topic_combo_(nullptr), refresh_button_(nullptr), status_label_(nullptr), 
  dot_widget_(nullptr), topic_refresh_timer_(nullptr), current_topic_("behavior_tree_graphviz")
{
  // Create the main layout
  layout_ = new QVBoxLayout(this);
  
  // Create topic selection layout
  topic_layout_ = new QHBoxLayout();
  
  // Create topic selection label
  topic_label_ = new QLabel("Topic:");
  topic_layout_->addWidget(topic_label_);
  
  // Create topic combo box
  topic_combo_ = new QComboBox();
  topic_combo_->setEditable(true);
  topic_combo_->setMinimumWidth(200);
  topic_combo_->setInsertPolicy(QComboBox::NoInsert);  // Don't auto-add typed text to dropdown
  topic_layout_->addWidget(topic_combo_);
  
  // Create refresh button
  refresh_button_ = new QPushButton("Refresh");
  refresh_button_->setMaximumWidth(80);
  topic_layout_->addWidget(refresh_button_);
  
  // Add some spacing between controls and status
  topic_layout_->addSpacing(20);
  
  // Create status label and add it to the same horizontal layout
  status_label_ = new QLabel("Waiting for behavior tree data...");
  status_label_->setStyleSheet("QLabel { color: gray; font-style: italic; }");
  topic_layout_->addWidget(status_label_);
  
  // Add stretch to push status to the right
  topic_layout_->addStretch();
  
  // Add topic layout to main layout
  layout_->addLayout(topic_layout_);
  
  // Create the xdot widget for rendering the behavior tree
  dot_widget_ = new xdot_cpp::ui::DotWidget(this);
  dot_widget_->setMinimumSize(400, 300);
  layout_->addWidget(dot_widget_);
  
  // Set layout stretch factors so the dot widget takes most of the space
  layout_->setStretchFactor(topic_layout_, 0);
  layout_->setStretchFactor(dot_widget_, 1);
  
  // Connect signals
  connect(topic_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
          this, &BehaviorTreePanel::onTopicChanged);
  connect(topic_combo_, &QComboBox::editTextChanged,
          this, &BehaviorTreePanel::onTopicTextChanged);
  connect(refresh_button_, &QPushButton::clicked, 
          this, &BehaviorTreePanel::onRefreshTopics);
  
  // Create timer for periodic topic refresh
  topic_refresh_timer_ = new QTimer(this);
  connect(topic_refresh_timer_, &QTimer::timeout, 
          this, &BehaviorTreePanel::updateAvailableTopics);
  topic_refresh_timer_->start(5000); // Refresh every 5 seconds
}

BehaviorTreePanel::~BehaviorTreePanel() = default;

void BehaviorTreePanel::onInitialize()
{
  // Access the abstract ROS Node and lock it for exclusive use
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  
  // Update available topics and subscribe to the current topic
  updateAvailableTopics();
  subscribeToTopic(current_topic_);
  
  // Create a single-shot timer to retry topic discovery after a short delay
  // This helps with the initial loading when topics might not be discovered yet
  QTimer::singleShot(1000, this, [this]() {
    updateAvailableTopics();
    // If we have a saved topic but it's not currently selected, try to select it
    if (!current_topic_.empty()) {
      int index = topic_combo_->findText(QString::fromStdString(current_topic_));
      if (index >= 0 && topic_combo_->currentIndex() != index) {
        topic_combo_->setCurrentIndex(index);
      }
    }
  });
}

void BehaviorTreePanel::behaviorTreeCallback(const behavior_tree_msgs::msg::GraphVizXdot::SharedPtr msg)
{
  // Only update if the graphviz data has changed to avoid unnecessary redraws
  if (msg->xdot.data != previous_graphviz_)
  {
    try
    {
      // Set the xdot code to render the behavior tree
      dot_widget_->set_dot_code(msg->xdot.data);
      
      // Update status
      status_label_->setText("Behavior tree updated");
      status_label_->setStyleSheet("QLabel { color: blue; }");
      
      // Store the current graphviz data
      previous_graphviz_ = msg->xdot.data;
    }
    catch (const std::exception& e)
    {
      // Handle any parsing errors
      status_label_->setText(QString("Error parsing behavior tree: %1").arg(e.what()));
      status_label_->setStyleSheet("QLabel { color: red; }");
    }
  }
}

void BehaviorTreePanel::updateAvailableTopics()
{
  if (!node_ptr_) {
    return;
  }
  
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  
  // Get all topics and their types
  auto topic_names_and_types = node->get_topic_names_and_types();
  
  // Find topics with GraphVizXdot message type
  std::vector<std::string> graphviz_topics;
  for (const auto& topic_info : topic_names_and_types) {
    const std::string& topic_name = topic_info.first;
    const std::vector<std::string>& types = topic_info.second;
    
    // Check if this topic publishes GraphVizXdot messages
    for (const std::string& type : types) {
      if (type == "behavior_tree_msgs/msg/GraphVizXdot") {
        graphviz_topics.push_back(topic_name);
        break;
      }
    }
  }
  
  // Update combo box
  QString current_selection = topic_combo_->currentText();
  topic_combo_->clear();
  
  if (graphviz_topics.empty()) {
    topic_combo_->addItem("No GraphVizXdot topics found");
    topic_combo_->setEnabled(false);
  } else {
    topic_combo_->setEnabled(true);
    
    // Add all found topics
    for (const std::string& topic : graphviz_topics) {
      topic_combo_->addItem(QString::fromStdString(topic));
    }
    
    // Try to restore previous selection
    int index = topic_combo_->findText(current_selection);
    if (index >= 0) {
      topic_combo_->setCurrentIndex(index);
    } else {
      // Try to select the current topic
      index = topic_combo_->findText(QString::fromStdString(current_topic_));
      if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
      }
    }
  }
}

void BehaviorTreePanel::subscribeToTopic(const std::string& topic_name)
{
  if (!node_ptr_) {
    return;
  }
  
  // Unsubscribe from previous topic
  behavior_tree_subscription_.reset();
  
  // Clear previous data
  previous_graphviz_.clear();
  dot_widget_->set_dot_code("");
  
  if (topic_name.empty()) {
    status_label_->setText("No topic selected");
    status_label_->setStyleSheet("QLabel { color: gray; font-style: italic; }");
    return;
  }
  
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  
  try {
    // Subscribe to the new topic
    behavior_tree_subscription_ = node->create_subscription<behavior_tree_msgs::msg::GraphVizXdot>(
      topic_name, 
      10, 
      std::bind(&BehaviorTreePanel::behaviorTreeCallback, this, std::placeholders::_1)
    );
    
    current_topic_ = topic_name;
    
    // Update status
    status_label_->setText(QString("Subscribed to %1").arg(QString::fromStdString(topic_name)));
    status_label_->setStyleSheet("QLabel { color: green; }");
  } catch (const std::exception& e) {
    status_label_->setText(QString("Error subscribing to %1: %2")
                          .arg(QString::fromStdString(topic_name))
                          .arg(e.what()));
    status_label_->setStyleSheet("QLabel { color: red; }");
  }
}

void BehaviorTreePanel::onTopicChanged()
{
  if (!topic_combo_->isEnabled()) {
    return;
  }
  
  QString selected_topic = topic_combo_->currentText();
  if (!selected_topic.isEmpty() && selected_topic != "No GraphVizXdot topics found") {
    subscribeToTopic(selected_topic.toStdString());
  }
}

void BehaviorTreePanel::onTopicTextChanged(const QString & text)
{
  if (!topic_combo_->isEnabled()) {
    return;
  }
  
  // Subscribe to manually typed topic name
  if (!text.isEmpty() && text != "No GraphVizXdot topics found") {
    subscribeToTopic(text.toStdString());
  }
}

void BehaviorTreePanel::onRefreshTopics()
{
  updateAvailableTopics();
}

void BehaviorTreePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("topic", QString::fromStdString(current_topic_));
}

void BehaviorTreePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  
  QString topic;
  if (config.mapGetString("topic", &topic)) {
    current_topic_ = topic.toStdString();
    
    // Update combo box selection if it's already populated
    if (topic_combo_->count() > 0) {
      int index = topic_combo_->findText(topic);
      if (index >= 0) {
        topic_combo_->setCurrentIndex(index);
      }
    }
  }
}

}  // namespace rviz_behavior_tree_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_behavior_tree_panel::BehaviorTreePanel, rviz_common::Panel)