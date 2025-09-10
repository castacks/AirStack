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

#ifndef RVIZ_BEHAVIOR_TREE_PANEL__BEHAVIOR_TREE_PANEL_HPP_
#define RVIZ_BEHAVIOR_TREE_PANEL__BEHAVIOR_TREE_PANEL_HPP_

#include <QVBoxLayout>
#include <QLabel>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <xdot_cpp/ui/dot_widget.h>

namespace rviz_behavior_tree_panel
{

class BehaviorTreePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BehaviorTreePanel(QWidget * parent = nullptr);
  ~BehaviorTreePanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_tree_subscription_;

  void behaviorTreeCallback(const std_msgs::msg::String::SharedPtr msg);

private:
  QVBoxLayout * layout_;
  QLabel * status_label_;
  xdot_cpp::ui::DotWidget * dot_widget_;
  
  std::string previous_graphviz_;
};

}  // namespace rviz_behavior_tree_panel

#endif  // RVIZ_BEHAVIOR_TREE_PANEL__BEHAVIOR_TREE_PANEL_HPP_