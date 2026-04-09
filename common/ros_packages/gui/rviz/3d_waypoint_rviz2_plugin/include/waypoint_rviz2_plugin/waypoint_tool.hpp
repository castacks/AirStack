/**
 * @file waypoint_tool.hpp
 * @author KoKoLates (the21515@gmail.com)
 * @version 1.0.0
 * @date 2023-06-19
 * 
 * ROS2 port of the interactive waypoint tool for RViz2
 */

#pragma once

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include "waypoint_widget.hpp"

namespace rviz_common {
    class DisplayContext;
    class ViewportMouseEvent;
    namespace properties {
        class VectorProperty;
    }
    class PanelDockWidget;
}

namespace waypoint_rviz2_plugin {

class WaypointTool: public rviz_common::Tool {
    Q_OBJECT
public:
    WaypointTool();
    ~WaypointTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

    void makeItem(const Ogre::Vector3&, const Ogre::Quaternion&);

    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    void getMarkerPose();
    void clearAllMarker();
    void processFeedBack(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&);

    Ogre::SceneNode *move_axis_node_;
    std::string axis_resource_;

    WaypointWidget *widget_;
    rviz_common::PanelDockWidget *widget_dock_;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_handler_;

    typedef std::map<int, Ogre::SceneNode*> str2nodeptr;
    str2nodeptr node_map_;

    int unique_idx_;
    
    rclcpp::Node::SharedPtr node_;
};

}
