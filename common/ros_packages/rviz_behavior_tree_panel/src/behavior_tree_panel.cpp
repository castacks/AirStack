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
: Panel(parent), layout_(nullptr), status_label_(nullptr), dot_widget_(nullptr)
{
  // Create the main layout
  layout_ = new QVBoxLayout(this);
  
  // Create status label
  status_label_ = new QLabel("Waiting for behavior tree data...");
  status_label_->setStyleSheet("QLabel { color: gray; font-style: italic; }");
  layout_->addWidget(status_label_);
  
  // Create the xdot widget for rendering the behavior tree
  dot_widget_ = new xdot_cpp::ui::DotWidget(this);
  dot_widget_->setMinimumSize(400, 300);
  layout_->addWidget(dot_widget_);
  
  // Set layout stretch factors so the dot widget takes most of the space
  layout_->setStretchFactor(status_label_, 0);
  layout_->setStretchFactor(dot_widget_, 1);
}

BehaviorTreePanel::~BehaviorTreePanel() = default;

void BehaviorTreePanel::onInitialize()
{
  // Access the abstract ROS Node and lock it for exclusive use
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  
  // Get a pointer to the familiar rclcpp::Node for making subscriptions
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  
  // Subscribe to the behavior tree graphviz topic
  behavior_tree_subscription_ = node->create_subscription<behavior_tree_msgs::msg::GraphVizXdot>(
    "behavior_tree_graphviz", 
    10, 
    std::bind(&BehaviorTreePanel::behaviorTreeCallback, this, std::placeholders::_1)
  );
  
  // Update status
  status_label_->setText("Subscribed to behavior_tree_graphviz topic");
  status_label_->setStyleSheet("QLabel { color: green; }");
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

}  // namespace rviz_behavior_tree_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_behavior_tree_panel::BehaviorTreePanel, rviz_common::Panel)