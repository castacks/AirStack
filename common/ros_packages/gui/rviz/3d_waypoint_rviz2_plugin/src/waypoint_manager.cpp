/**
 * @file waypoint_manager.cpp
 *
 * Implementation of the shared WaypointManager singleton.
 */

#include "waypoint_rviz2_plugin/waypoint_manager.hpp"

#include <sstream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/mesh_loader.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace waypoint_rviz2_plugin {

// ─────────────────────────── singleton ─────────────────────────────────────────

std::shared_ptr<WaypointManager> WaypointManager::instance()
{
  static std::weak_ptr<WaypointManager> weak_instance;
  static std::mutex singleton_mtx;
  std::lock_guard<std::mutex> lock(singleton_mtx);

  auto sp = weak_instance.lock();
  if (!sp) {
    sp = std::shared_ptr<WaypointManager>(new WaypointManager());
    weak_instance = sp;
  }
  return sp;
}

WaypointManager::WaypointManager()
{
  selected_marker_name_ = std::string(kWaypointPrefix) + "1";
}

WaypointManager::~WaypointManager()
{
  if (scene_manager_) {
    for (auto & [idx, node_ptr] : node_map_) {
      scene_manager_->destroySceneNode(node_ptr);
    }
  }
}

// ─────────────────────────── initialization ───────────────────────────────────

void WaypointManager::initialize(rviz_common::DisplayContext * context,
                                 rclcpp::Node::SharedPtr node)
{
  if (initialized_) { return; }

  context_ = context;
  node_ = node;
  scene_manager_ = context_->getSceneManager();

  axis_resource_ = "package://waypoint_rviz2_plugin/media/axis.dae";
  if (rviz_rendering::loadMeshFromResource(axis_resource_).isNull()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "WaypointManager: failed to load model resource '%s'.",
                 axis_resource_.c_str());
    return;
  }

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "waypoint_plugin", node_);

  path_publisher_ =
    node_->create_publisher<nav_msgs::msg::Path>(output_topic_, 1);

  menu_handler_.insert(
    "delete",
    std::bind(&WaypointManager::processFeedback, this, std::placeholders::_1));
  menu_handler_.insert(
    "set manual",
    std::bind(&WaypointManager::processFeedback, this, std::placeholders::_1));

  initialized_ = true;
}

// ─────────────────────────── waypoint operations ──────────────────────────────

void WaypointManager::addWaypoint(const Ogre::Vector3 & position,
                                  const Ogre::Quaternion & quaternion)
{
  if (!initialized_) { return; }

  unique_idx_++;
  std::stringstream wp_name;
  wp_name << kWaypointPrefix << unique_idx_;
  std::string str_name(wp_name.str());

  if (rviz_rendering::loadMeshFromResource(axis_resource_).isNull()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "WaypointManager: failed to load model resource '%s'.",
                 axis_resource_.c_str());
    return;
  }

  Ogre::SceneNode * node_ptr =
    scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity * entity = scene_manager_->createEntity(axis_resource_);
  node_ptr->attachObject(entity);
  node_ptr->setVisible(true);
  node_ptr->setPosition(position);
  node_ptr->setOrientation(quaternion);

  auto node_entry = node_map_.find(unique_idx_);
  if (node_entry == node_map_.end()) {
    node_map_.insert(std::make_pair(unique_idx_, node_ptr));
  } else {
    RCLCPP_WARN(node_->get_logger(), "%s already in map", str_name.c_str());
    return;
  }

  // Build interactive marker
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = position.x;
  pose.pose.position.y = position.y;
  pose.pose.position.z = position.z;
  pose.pose.orientation.x = quaternion.x;
  pose.pose.orientation.y = quaternion.y;
  pose.pose.orientation.z = quaternion.z;
  pose.pose.orientation.w = quaternion.w;

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.stamp = node_->now();
  {
    std::lock_guard<std::mutex> lock(mtx_);
    int_marker.header.frame_id = frame_id_;
  }
  int_marker.pose = pose.pose;
  int_marker.scale = 1.0;
  int_marker.name = str_name;

  // Invisible cylinder for picking
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.0;

  visualization_msgs::msg::InteractiveMarkerControl c_control;
  c_control.always_visible = true;
  c_control.markers.push_back(marker);
  int_marker.controls.push_back(c_control);

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = 0.707106781;
  control.orientation.x = 0;
  control.orientation.y = 0.707106781;
  control.orientation.z = 0;
  control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);

  control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_delete";
  control.description = str_name;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(
    int_marker.name,
    std::bind(&WaypointManager::processFeedback, this, std::placeholders::_1));
  menu_handler_.apply(*server_, int_marker.name);
  server_->applyChanges();

  selected_marker_name_ = str_name;

  // Convert quaternion to yaw for signal
  tf2::Quaternion qt(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Matrix3x3 m(qt);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  Q_EMIT waypointCountChanged(static_cast<int>(node_map_.size()));
  Q_EMIT selectedMarkerChanged(
    QString::fromStdString(str_name), position.x, position.y, position.z, yaw);
}

void WaypointManager::removeWaypoint(int idx)
{
  if (!initialized_) { return; }

  auto it = node_map_.find(idx);
  if (it == node_map_.end()) { return; }

  it->second->detachAllObjects();
  scene_manager_->destroySceneNode(it->second);

  std::stringstream wp_name;
  wp_name << kWaypointPrefix << idx;
  server_->erase(wp_name.str());
  server_->applyChanges();

  node_map_.erase(it);

  Q_EMIT waypointCountChanged(static_cast<int>(node_map_.size()));
}

void WaypointManager::clearAll()
{
  if (!initialized_) { return; }

  for (auto & [idx, node_ptr] : node_map_) {
    scene_manager_->destroySceneNode(node_ptr);
  }
  node_map_.clear();
  unique_idx_ = 0;

  server_->clear();
  server_->applyChanges();

  Q_EMIT waypointCountChanged(0);
  Q_EMIT waypointsCleared();
}

nav_msgs::msg::Path WaypointManager::getPath() const
{
  nav_msgs::msg::Path path;
  for (const auto & [idx, node_ptr] : node_map_) {
    Ogre::Vector3 pos = node_ptr->getPosition();
    Ogre::Quaternion quat = node_ptr->getOrientation();

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = pos.x;
    pose.pose.position.y = pos.y;
    pose.pose.position.z = pos.z;
    pose.pose.orientation.x = quat.x;
    pose.pose.orientation.y = quat.y;
    pose.pose.orientation.z = quat.z;
    pose.pose.orientation.w = quat.w;
    path.poses.push_back(pose);
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    path.header.frame_id = frame_id_;
  }
  if (node_) {
    path.header.stamp = node_->now();
  }
  return path;
}

void WaypointManager::publishPath()
{
  if (!path_publisher_) { return; }
  path_publisher_->publish(getPath());
}

void WaypointManager::updateSelectedPose(double x, double y, double z,
                                         double yaw)
{
  if (!initialized_) { return; }

  int selected_idx = 0;
  try {
    selected_idx = std::stoi(
      selected_marker_name_.substr(strlen(kWaypointPrefix)));
  } catch (const std::logic_error & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
    return;
  }

  auto it = node_map_.find(selected_idx);
  if (it == node_map_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "%s not found in map",
                 selected_marker_name_.c_str());
    return;
  }

  Ogre::Vector3 position(x, y, z);
  tf2::Quaternion qt;
  qt.setRPY(0.0, 0.0, yaw);
  Ogre::Quaternion quaternion(qt.w(), qt.x(), qt.y(), qt.z());

  it->second->setPosition(position);
  it->second->setOrientation(quaternion);

  std::stringstream wp_name;
  wp_name << kWaypointPrefix << it->first;
  std::string wp_name_str(wp_name.str());

  visualization_msgs::msg::InteractiveMarker int_marker;
  if (server_->get(wp_name_str, int_marker)) {
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;
    int_marker.pose.orientation.x = qt.x();
    int_marker.pose.orientation.y = qt.y();
    int_marker.pose.orientation.z = qt.z();
    int_marker.pose.orientation.w = qt.w();
    server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
  }
  server_->applyChanges();
}

// ─────────────────────────── persistence ──────────────────────────────────────

void WaypointManager::saveBag(const std::string & filename)
{
  nav_msgs::msg::Path path = getPath();

  try {
    rosbag2_cpp::Writer writer;

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = filename;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer.open(storage_options, converter_options);

    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = "waypoints";
    topic_metadata.type = "nav_msgs/msg/Path";
    topic_metadata.serialization_format = rmw_get_serialization_format();
    writer.create_topic(topic_metadata);

    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<nav_msgs::msg::Path> serialization;
    serialization.serialize_message(&path, serialized_msg.get());

    auto bag_message =
      std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->topic_name = "waypoints";
    bag_message->recv_timestamp = node_->now().nanoseconds();
    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [](rcutils_uint8_array_t * data) {
        auto fini_return = rcutils_uint8_array_fini(data);
        delete data;
        if (fini_return != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "waypoint_rviz2_plugin",
            "Failed to destroy serialized message %s",
            rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data =
      serialized_msg->release_rcl_serialized_message();

    writer.write(bag_message);

    RCLCPP_INFO(node_->get_logger(), "Waypoints saved successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save bag: %s", e.what());
  }
}

void WaypointManager::loadBag(const std::string & filename)
{
  try {
    rosbag2_cpp::Reader reader;

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = filename;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      if (bag_message->topic_name == "waypoints") {
        rclcpp::SerializedMessage serialized_msg(
          *bag_message->serialized_data);
        nav_msgs::msg::Path path;

        rclcpp::Serialization<nav_msgs::msg::Path> serialization;
        serialization.deserialize_message(&serialized_msg, &path);

        RCLCPP_INFO(node_->get_logger(), "Loading %zu waypoints",
                    path.poses.size());

        for (size_t i = 0; i < path.poses.size(); i++) {
          const auto & pos = path.poses[i];
          Ogre::Vector3 position(
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);
          Ogre::Quaternion quaternion(
            pos.pose.orientation.w, pos.pose.orientation.x,
            pos.pose.orientation.y, pos.pose.orientation.z);
          addWaypoint(position, quaternion);
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load bag: %s", e.what());
  }
}

// ─────────────────────────── accessors ────────────────────────────────────────

std::string WaypointManager::frameId() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return frame_id_;
}

void WaypointManager::setFrameId(const std::string & frame)
{
  std::lock_guard<std::mutex> lock(mtx_);
  frame_id_ = frame;
}

std::string WaypointManager::outputTopic() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return output_topic_;
}

void WaypointManager::setOutputTopic(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (topic == output_topic_) { return; }
  output_topic_ = topic;
  if (node_ && !topic.empty() && topic != "/") {
    path_publisher_.reset();
    path_publisher_ =
      node_->create_publisher<nav_msgs::msg::Path>(output_topic_, 1);
  }
}

double WaypointManager::defaultHeight() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return default_height_;
}

void WaypointManager::setDefaultHeight(double h)
{
  std::lock_guard<std::mutex> lock(mtx_);
  default_height_ = h;
}

int WaypointManager::waypointCount() const
{
  return static_cast<int>(node_map_.size());
}

// ─────────────────────────── feedback ─────────────────────────────────────────

void WaypointManager::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &
    feedback)
{
  switch (feedback->event_type) {
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT: {
      int idx = 0;
      try {
        idx = std::stoi(
          feedback->marker_name.substr(strlen(kWaypointPrefix)));
      } catch (const std::logic_error & e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
        return;
      }

      auto node_entry = node_map_.find(idx);
      if (node_entry == node_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "%s not found in map",
                     feedback->marker_name.c_str());
        return;
      }

      if (feedback->menu_entry_id == 1) {
        // delete
        std::stringstream wp_name;
        wp_name << kWaypointPrefix << node_entry->first;
        server_->erase(wp_name.str());
        menu_handler_.reApply(*server_);
        server_->applyChanges();
        node_entry->second->detachAllObjects();
        scene_manager_->destroySceneNode(node_entry->second);
        node_map_.erase(node_entry);

        Q_EMIT waypointCountChanged(static_cast<int>(node_map_.size()));
      } else {
        // "set manual" – noop for now, marker stays at current spinbox pose
      }
      break;
    }
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE: {
      int idx = 0;
      try {
        idx = std::stoi(
          feedback->marker_name.substr(strlen(kWaypointPrefix)));
      } catch (const std::logic_error & e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
        return;
      }

      auto node_entry = node_map_.find(idx);
      if (node_entry == node_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "%s not found in map",
                     feedback->marker_name.c_str());
        return;
      }

      const auto & p = feedback->pose;
      Ogre::Vector3 position(p.position.x, p.position.y, p.position.z);
      Ogre::Quaternion quaternion(
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);

      node_entry->second->setPosition(position);
      node_entry->second->setOrientation(quaternion);

      selected_marker_name_ = feedback->marker_name;

      tf2::Quaternion qt(quaternion.x, quaternion.y, quaternion.z,
                         quaternion.w);
      tf2::Matrix3x3 m(qt);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      Q_EMIT selectedMarkerChanged(
        QString::fromStdString(feedback->marker_name),
        position.x, position.y, position.z, yaw);
      break;
    }
  }
}

}  // namespace waypoint_rviz2_plugin
