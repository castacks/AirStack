/**
 * @file waypoint_widget.hpp
 * @author KoKoLates (the21515@gmail.com)
 * @version 1.0.0
 * @date 2023-12-10
 * 
 * ROS2 port of the interactive waypoint widget for RViz2
 */

#pragma once

#ifndef Q_MOC_RUN
#include <mutex>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#endif

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include "ui_waypoint_plugin.h"

namespace rviz_common {
    class DisplayContext;
}

namespace interactive_markers {
    class InteractiveMarkerServer;
}

namespace Ui {
    class PluginWidget;
}

namespace waypoint_rviz2_plugin {
    class WaypointTool;
}

namespace waypoint_rviz2_plugin {
    constexpr char waypoint_name_prefix[] = "point#";
    
    class WaypointWidget: public QWidget {
        friend class WaypointTool;
        Q_OBJECT

    public: 
        WaypointWidget(rviz_common::DisplayContext *context, 
                       rclcpp::Node::SharedPtr node,
                       std::map<int, Ogre::SceneNode*> *map_ptr, 
                       std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                       int *unique_idx, 
                       QWidget *parent=nullptr, 
                       WaypointTool *waypoint_tool=nullptr);
        ~WaypointWidget();

        void enable();
        void disable();
    
        void setPose(const Ogre::Vector3&, const Ogre::Quaternion&);
        void setConfig(QString topic, QString frame, float height);
        void setWaypointLabel(Ogre::Vector3 position);
        void setWaypointCount(int size);
        void setSelectedMarkerName(std::string name);

        void getPose(Ogre::Vector3&, Ogre::Quaternion&);
        QString getFrameId();
        QString getOutputTopic();
        double getDefaultHeight();

        void saveBag(const std::string& filename);
        void loadBag(const std::string& filename);

    protected:
        Ui::PluginWidget *ui_;
        rviz_common::DisplayContext *context_;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

        WaypointTool *waypoint_tool_;
        std::map<int, Ogre::SceneNode*> *map_ptr_;
        Ogre::SceneManager *scene_manager_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        int* unique_idx_;
        double default_height_;

        QString output_topic_;
        QString frame_id_;

        std::mutex frame_updates_mutex_;
        std::string selected_marker_name_;

    private Q_SLOTS:
        void saveHandler();
        void loadHandler();
        void clearHandler();
        void publishHandler();
        void frameChange();
        void topicChange();
        void poseChange(double value);
        void heightChange(double height);
    };
}
