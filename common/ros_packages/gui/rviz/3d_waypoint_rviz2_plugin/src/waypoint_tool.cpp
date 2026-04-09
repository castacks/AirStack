/**
 * @file waypoint_tool.cpp
 * @author KoKoLates (the21515@gmail.com)
 * @version 1.0.0
 * @date 2023-06-20
 * 
 * ROS2 port of the interactive waypoint tool implementation
 */

#include "waypoint_rviz2_plugin/waypoint_tool.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#ifndef Q_MOC_RUN
#include <rclcpp/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/mesh_loader.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
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
    : move_axis_node_(nullptr), 
      widget_dock_(nullptr), 
      widget_(nullptr), 
      unique_idx_(0)
{
    shortcut_key_ = '1';
}

WaypointTool::~WaypointTool() {
    str2nodeptr::iterator node_itr;
    for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
        scene_manager_->destroySceneNode(node_itr->second);
    }
    delete widget_;
    delete widget_dock_;
}

void WaypointTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    
    axis_resource_ = "package://waypoint_rviz2_plugin/media/axis.dae";

    if (rviz_rendering::loadMeshFromResource(axis_resource_).isNull()) {
        RCLCPP_ERROR(node_->get_logger(), "Waypoint Tool: failed to load model resource '%s'.", axis_resource_.c_str());
        return;
    }

    move_axis_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity(axis_resource_);
    move_axis_node_->attachObject(entity);
    move_axis_node_->setVisible(false);

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "waypoint_plugin", node_
    );

    rviz_common::WindowManagerInterface* window_context_ = context_->getWindowManager();
    widget_ = new WaypointWidget(context_, node_, &node_map_, server_, &unique_idx_, nullptr, this);

    if (window_context_) {
        widget_dock_ = window_context_->addPane("Waypoint Plugin", widget_);
    }
    widget_->enable();

    menu_handler_.insert("delete", 
        std::bind(&WaypointTool::processFeedBack, this, std::placeholders::_1));
    menu_handler_.insert("set manual", 
        std::bind(&WaypointTool::processFeedBack, this, std::placeholders::_1));
}

void WaypointTool::activate() {
    if (move_axis_node_) {
        move_axis_node_->setVisible(true);
    }
}

void WaypointTool::deactivate() {
    if (move_axis_node_) {
        move_axis_node_->setVisible(false);
    }
}

int WaypointTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    if (!move_axis_node_) {
        return Render;
    }

    double height = widget_->getDefaultHeight();
    Ogre::Vector3 intersection;
    Ogre::Quaternion quaternion;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, height);
    
    // Get viewport and camera from the render window
    auto render_window = event.panel->getRenderWindow();
    Ogre::Camera * camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window);
    Ogre::Viewport * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);
    
    // Convert mouse coordinates to normalized device coordinates
    int width = viewport->getActualWidth();
    int height_px = viewport->getActualHeight();
    float screen_x = static_cast<float>(event.x) / static_cast<float>(width);
    float screen_y = static_cast<float>(event.y) / static_cast<float>(height_px);
    
    // Create ray from camera through mouse position
    Ogre::Ray mouse_ray = camera->getCameraToViewportRay(screen_x, screen_y);
    
    // Intersect ray with ground plane
    std::pair<bool, Ogre::Real> result = mouse_ray.intersects(ground_plane);
    
    if (result.first) {
        intersection = mouse_ray.getPoint(result.second);
        move_axis_node_->setVisible(true);
        move_axis_node_->setPosition(intersection);

        widget_->setWaypointLabel(intersection);

        str2nodeptr::iterator node_itr;
        for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
            Ogre::Vector3 stored_pose = node_itr->second->getPosition();
            double distance = std::sqrt(
                pow(stored_pose.x - intersection.x, 2) + pow(stored_pose.y - intersection.y, 2)
            );

            if (distance < 0.4) {
                move_axis_node_->setVisible(false);

                if (event.rightDown()) {
                    node_itr->second->detachAllObjects();
                    std::stringstream waypoint_name;
                    waypoint_name << waypoint_name_prefix << node_itr->first;
                    server_->erase(waypoint_name.str());
                    server_->applyChanges();
                    node_map_.erase(node_itr);

                    move_axis_node_->setVisible(true);
                    return Render | Finished;
                }
            }
        }

        if (event.leftDown()) {
            makeItem(intersection, quaternion);
            return Render | Finished;
        }
    }
    else {
        move_axis_node_->setVisible(false);
    }
    return Render;
}

void WaypointTool::makeItem(const Ogre::Vector3& position, const Ogre::Quaternion& quaternion) {
    unique_idx_++;
    std::stringstream waypoint_name;
    waypoint_name << waypoint_name_prefix << unique_idx_;
    std::string str_name(waypoint_name.str());

    if (rviz_rendering::loadMeshFromResource(axis_resource_).isNull()) {
        RCLCPP_ERROR(node_->get_logger(), "WaypointTool: failed to load model resource '%s'.", axis_resource_.c_str());
        return;
    }

    Ogre::SceneNode *node_ptr = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity *entity = scene_manager_->createEntity(axis_resource_);
    node_ptr->attachObject(entity);
    node_ptr->setVisible(true);
    node_ptr->setPosition(position);
    node_ptr->setOrientation(quaternion);

    str2nodeptr::iterator node_entry = node_map_.find(unique_idx_);
    if (node_entry == node_map_.end()) {
        node_map_.insert(std::make_pair(unique_idx_, node_ptr));
    } else {
        RCLCPP_WARN(node_->get_logger(), "%s already in map", str_name.c_str());
        return;
    }
    widget_->setWaypointCount(node_map_.size());

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
    int_marker.header.frame_id = widget_->getFrameId().toStdString();
    int_marker.pose = pose.pose;
    int_marker.scale = 1.0;
    int_marker.name = str_name;

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
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    control.name = "menu_delete";
    control.description = str_name;
    int_marker.controls.push_back(control);

    server_->insert(int_marker);
    server_->setCallback(int_marker.name, 
        std::bind(&WaypointTool::processFeedBack, this, std::placeholders::_1));
    menu_handler_.apply(*server_, int_marker.name);

    Ogre::Vector3 p = position;
    Ogre::Quaternion q = quaternion;
    widget_->setSelectedMarkerName(str_name);
    widget_->setWaypointLabel(p);
    widget_->setPose(p, q);
    server_->applyChanges();
}

void WaypointTool::processFeedBack(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) 
{
    switch (feedback->event_type) {
        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        {
            str2nodeptr::iterator node_entry = node_map_.find(std::stoi(
                feedback->marker_name.substr(strlen(waypoint_name_prefix))));
            if (node_entry == node_map_.end()) {
                RCLCPP_ERROR(node_->get_logger(), "%s not found in map", feedback->marker_name.c_str());
            }
            else {
                if (feedback->menu_entry_id == 1) {
                    std::stringstream waypoint_name;
                    waypoint_name << waypoint_name_prefix << node_entry->first;
                    server_->erase(waypoint_name.str());

                    menu_handler_.reApply(*server_);
                    server_->applyChanges();
                    node_entry->second->detachAllObjects();
                    node_map_.erase(node_entry);

                    int waypoint_num = node_map_.size();
                    widget_->setWaypointCount(waypoint_num);
                }
                else {
                    Ogre::Vector3 position;
                    Ogre::Quaternion quaternion;

                    widget_->getPose(position, quaternion);

                    geometry_msgs::msg::Pose pose;
                    pose.position.x = position.x;
                    pose.position.y = position.y;
                    pose.position.z = position.z;

                    pose.orientation.x = quaternion.x;
                    pose.orientation.y = quaternion.y;
                    pose.orientation.z = quaternion.z;
                    pose.orientation.w = quaternion.w;

                    node_entry->second->setPosition(position);
                    node_entry->second->setOrientation(quaternion);

                    widget_->setWaypointLabel(position);
                    server_->setPose(feedback->marker_name, pose);
                    server_->applyChanges();
                }
            }
        }
        break;
        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            str2nodeptr::iterator node_entry = node_map_.find(std::stoi(
                feedback->marker_name.substr(strlen(waypoint_name_prefix))
            ));
            if (node_entry == node_map_.end()) {
                RCLCPP_ERROR(node_->get_logger(), "%s not found in map", feedback->marker_name.c_str());
            }
            else {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose = feedback->pose;

                Ogre::Vector3 position;
                position.x = pose.pose.position.x;
                position.y = pose.pose.position.y;
                position.z = pose.pose.position.z;
                node_entry->second->setPosition(position);

                Ogre::Quaternion quaternion;
                quaternion.x = pose.pose.orientation.x;
                quaternion.y = pose.pose.orientation.y;
                quaternion.z = pose.pose.orientation.z;
                quaternion.w = pose.pose.orientation.w;
                node_entry->second->setOrientation(quaternion);

                widget_->setWaypointLabel(position);
                widget_->setPose(position, quaternion);
                widget_->setSelectedMarkerName(feedback->marker_name);
            }
        }
        break;
    }
}

void WaypointTool::getMarkerPose() {
    str2nodeptr::iterator node_itr;
    for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
        visualization_msgs::msg::InteractiveMarker int_marker;

        std::stringstream waypoint_name;
        waypoint_name << waypoint_name_prefix << node_itr->first;
        server_->get(waypoint_name.str(), int_marker);

        RCLCPP_INFO(
            node_->get_logger(),
            "pose: %g %g %g", 
            int_marker.pose.position.x, 
            int_marker.pose.position.y, 
            int_marker.pose.position.z
        );
    }
}

void WaypointTool::clearAllMarker() {
    str2nodeptr::iterator node_itr;
    for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
        scene_manager_->destroySceneNode(node_itr->second);
    }
    node_map_.clear();
    unique_idx_ = 0;
}

void WaypointTool::save(rviz_common::Config config) const {
    config.mapSetValue("Class", getClassId());
    rviz_common::Config waypoint_config = config.mapMakeChild("WaypointTool");

    waypoint_config.mapSetValue("topic", widget_->getOutputTopic());
    waypoint_config.mapSetValue("frame_id", widget_->getFrameId());
    waypoint_config.mapSetValue("default_height", widget_->getDefaultHeight());
}

void WaypointTool::load(const rviz_common::Config &config) {
    rviz_common::Config waypoint_config = config.mapGetChild("WaypointTool");
    QString topic, frame;
    float height;
    if (!waypoint_config.mapGetString("topic", &topic)) {
        topic = "/waypoints";
    }
    if (!waypoint_config.mapGetString("frame_id", &frame)) {
        frame = "map";
    }
    waypoint_config.mapGetFloat("default_height", &height);

    widget_->setConfig(topic, frame, height);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz2_plugin::WaypointTool, rviz_common::Tool)
