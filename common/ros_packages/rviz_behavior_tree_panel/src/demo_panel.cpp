/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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

/* Author: David V. Lu!! */

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_panel_tutorial/demo_panel.hpp>

namespace rviz_panel_tutorial
{
DemoPanel::DemoPanel(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("GO!");
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &DemoPanel::buttonActivated);
}

DemoPanel::~DemoPanel() = default;

void DemoPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/input", 10, std::bind(&DemoPanel::topicCallback, this, std::placeholders::_1));
}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label
void DemoPanel::topicCallback(const std_msgs::msg::String& msg)
{
  label_->setText(QString(msg.data.c_str()));
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void DemoPanel::buttonActivated()
{
  auto message = std_msgs::msg::String();
  message.data = "Button clicked!";
  publisher_->publish(message);
}

}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::DemoPanel, rviz_common::Panel)
