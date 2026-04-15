/**
 * @file waypoint_tool.hpp
 * @author KoKoLates (the21515@gmail.com)
 * @version 2.0.0
 *
 * RViz2 Tool for placing/removing waypoints via mouse clicks in the 3-D
 * viewport.  All state is owned by WaypointManager; this class only converts
 * mouse events into manager calls.
 */

#pragma once

#ifndef Q_MOC_RUN
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#endif

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

#include "waypoint_manager.hpp"

namespace rviz_common {
class DisplayContext;
class ViewportMouseEvent;
}

namespace waypoint_rviz2_plugin {

class WaypointTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  WaypointTool();
  ~WaypointTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  Ogre::SceneNode * move_axis_node_{nullptr};
  std::shared_ptr<WaypointManager> manager_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace waypoint_rviz2_plugin
