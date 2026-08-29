/**
 * @file waypoint_tool.cpp
 * @author KoKoLates (the21515@gmail.com)
 * @version 2.0.0
 *
 * RViz2 Tool for placing/removing waypoints via mouse clicks.
 * All waypoint state is delegated to WaypointManager.
 */

#include "waypoint_rviz2_plugin/waypoint_tool.hpp"

#ifndef Q_MOC_RUN
#include <rclcpp/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/mesh_loader.hpp>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#endif

// Define slots macro for render_window.hpp compatibility
#ifndef slots
#define slots Q_SLOTS
#define signals Q_SIGNALS
#endif

#include <rviz_rendering/render_window.hpp>

namespace waypoint_rviz2_plugin {

WaypointTool::WaypointTool()
{
  shortcut_key_ = '1';
}

WaypointTool::~WaypointTool()
{
  // move_axis_node_ is owned by the Ogre SceneManager; it is cleaned up when
  // RViz destroys the scene.
}

void WaypointTool::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  std::string axis_resource = "package://waypoint_rviz2_plugin/media/axis.dae";
  if (rviz_rendering::loadMeshFromResource(axis_resource).isNull()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "WaypointTool: failed to load model resource '%s'.",
                 axis_resource.c_str());
    return;
  }

  move_axis_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity * entity = scene_manager_->createEntity(axis_resource);
  move_axis_node_->attachObject(entity);
  move_axis_node_->setVisible(false);

  // Create / retrieve the shared WaypointManager singleton
  manager_ = WaypointManager::instance();
  if (!manager_->isInitialized()) {
    manager_->initialize(context_, node_);
  }
}

void WaypointTool::activate()
{
  if (move_axis_node_) {
    move_axis_node_->setVisible(true);
  }
}

void WaypointTool::deactivate()
{
  if (move_axis_node_) {
    move_axis_node_->setVisible(false);
  }
}

int WaypointTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!move_axis_node_ || !manager_ || !manager_->isInitialized()) {
    return Render;
  }

  double height = manager_->defaultHeight();
  Ogre::Vector3 intersection;
  Ogre::Quaternion quaternion;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, height);

  auto render_window = event.panel->getRenderWindow();
  Ogre::Camera * camera =
    rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window);
  Ogre::Viewport * viewport =
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);

  int width = viewport->getActualWidth();
  int height_px = viewport->getActualHeight();
  float screen_x = static_cast<float>(event.x) / static_cast<float>(width);
  float screen_y = static_cast<float>(event.y) / static_cast<float>(height_px);

  Ogre::Ray mouse_ray = camera->getCameraToViewportRay(screen_x, screen_y);
  std::pair<bool, Ogre::Real> result = mouse_ray.intersects(ground_plane);

  if (result.first) {
    intersection = mouse_ray.getPoint(result.second);
    move_axis_node_->setVisible(true);
    move_axis_node_->setPosition(intersection);

    // Check proximity to existing waypoints
    const auto & node_map = manager_->nodeMap();
    for (auto it = node_map.begin(); it != node_map.end(); ++it) {
      Ogre::Vector3 stored_pose = it->second->getPosition();
      double distance = std::sqrt(
        std::pow(stored_pose.x - intersection.x, 2) +
        std::pow(stored_pose.y - intersection.y, 2));

      if (distance < 0.4) {
        move_axis_node_->setVisible(false);

        if (event.rightDown()) {
          manager_->removeWaypoint(it->first);
          move_axis_node_->setVisible(true);
          return Render | Finished;
        }
      }
    }

    if (event.leftDown()) {
      manager_->addWaypoint(intersection, quaternion);
      return Render | Finished;
    }
  } else {
    move_axis_node_->setVisible(false);
  }
  return Render;
}

void WaypointTool::save(rviz_common::Config config) const
{
  config.mapSetValue("Class", getClassId());
  rviz_common::Config wp_config = config.mapMakeChild("WaypointTool");

  if (manager_) {
    wp_config.mapSetValue("topic",
                          QString::fromStdString(manager_->outputTopic()));
    wp_config.mapSetValue("frame_id",
                          QString::fromStdString(manager_->frameId()));
    wp_config.mapSetValue("default_height", manager_->defaultHeight());
  }
}

void WaypointTool::load(const rviz_common::Config & config)
{
  rviz_common::Config wp_config = config.mapGetChild("WaypointTool");
  QString topic, frame;
  float height = 0.0f;

  if (!wp_config.mapGetString("topic", &topic)) {
    topic = "/waypoints";
  }
  if (!wp_config.mapGetString("frame_id", &frame)) {
    frame = "map";
  }
  wp_config.mapGetFloat("default_height", &height);

  if (manager_) {
    manager_->setOutputTopic(topic.toStdString());
    manager_->setFrameId(frame.toStdString());
    manager_->setDefaultHeight(height);
  }
}

}  // namespace waypoint_rviz2_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz2_plugin::WaypointTool, rviz_common::Tool)
