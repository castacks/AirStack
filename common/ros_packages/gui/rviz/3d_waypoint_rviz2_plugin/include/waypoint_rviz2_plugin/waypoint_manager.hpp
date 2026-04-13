/**
 * @file waypoint_manager.hpp
 *
 * Singleton that owns all waypoint state (interactive markers, Ogre scene
 * nodes, persistence).  Both WaypointTool (3-D mouse interaction) and the
 * Tasks Panel Navigate tab (UI controls) hold a shared_ptr to the same
 * instance so they share a single source of truth.
 */

#pragma once

#ifndef Q_MOC_RUN
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#endif

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

namespace rviz_common {
class DisplayContext;
}

namespace waypoint_rviz2_plugin {

constexpr char kWaypointPrefix[] = "point#";

class WaypointManager : public QObject
{
  Q_OBJECT

public:
  // ── singleton ──────────────────────────────────────────────────────────────
  static std::shared_ptr<WaypointManager> instance();

  ~WaypointManager() override;

  /** Called once by WaypointTool::onInitialize() which has the DisplayContext. */
  void initialize(rviz_common::DisplayContext * context,
                  rclcpp::Node::SharedPtr node);
  bool isInitialized() const { return initialized_; }

  // ── waypoint operations ────────────────────────────────────────────────────
  void addWaypoint(const Ogre::Vector3 & position, const Ogre::Quaternion & quaternion);
  void removeWaypoint(int idx);
  void clearAll();

  /** Build a nav_msgs/Path from current waypoint state. */
  nav_msgs::msg::Path getPath() const;

  /** Publish current waypoints on the configured topic. */
  void publishPath();

  void saveBag(const std::string & filename);
  void loadBag(const std::string & filename);

  /** Update pose of the currently-selected marker from spinbox values. */
  void updateSelectedPose(double x, double y, double z, double yaw);

  // ── accessors / mutators ───────────────────────────────────────────────────
  std::string frameId() const;
  void setFrameId(const std::string & frame);

  std::string outputTopic() const;
  void setOutputTopic(const std::string & topic);

  double defaultHeight() const;
  void setDefaultHeight(double h);

  int waypointCount() const;

  /** Read-only access to the node map (used by WaypointTool for proximity). */
  const std::map<int, Ogre::SceneNode *> & nodeMap() const { return node_map_; }

  /** Expose the interactive marker server (needed by WaypointTool for menu). */
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server() const { return server_; }

  interactive_markers::MenuHandler & menuHandler() { return menu_handler_; }

Q_SIGNALS:
  void waypointCountChanged(int count);
  void selectedMarkerChanged(const QString & name, double x, double y, double z, double yaw);
  void waypointsCleared();

private:
  WaypointManager();  // private – use instance()

  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  bool initialized_{false};
  rclcpp::Node::SharedPtr node_;
  rviz_common::DisplayContext * context_{nullptr};
  Ogre::SceneManager * scene_manager_{nullptr};

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  std::map<int, Ogre::SceneNode *> node_map_;
  int unique_idx_{0};

  std::string frame_id_{"map"};
  std::string output_topic_{"/waypoints"};
  double default_height_{0.0};
  std::string selected_marker_name_;

  std::string axis_resource_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  mutable std::mutex mtx_;
};

}  // namespace waypoint_rviz2_plugin
