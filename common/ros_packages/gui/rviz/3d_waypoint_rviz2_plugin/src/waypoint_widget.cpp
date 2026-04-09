/**
 * @file waypoint_widget.cpp
 * @author KoKoLates (the21515@gmail.com)
 * @version 1.0.0
 * @date 2023-06-18
 * 
 * ROS2 port of the interactive waypoint widget implementation
 */

#include "waypoint_rviz2_plugin/waypoint_widget.hpp"
#include "waypoint_rviz2_plugin/waypoint_tool.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <QFileDialog>
#include <rviz_common/display_context.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace waypoint_rviz2_plugin {

WaypointWidget::WaypointWidget(rviz_common::DisplayContext *context, 
                               rclcpp::Node::SharedPtr node,
                               std::map<int, Ogre::SceneNode*> *map_ptr,
                               std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                               int *unique_idx,
                               QWidget *parent, 
                               WaypointTool *waypoint_tool): QWidget(parent) 
{
    context_ = context;
    node_ = node;
    ui_ = new Ui::PluginWidget();
    map_ptr_ = map_ptr;
    unique_idx_ = unique_idx;
    server_ = server;
    frame_id_ = "map";
    waypoint_tool_ = waypoint_tool;
    default_height_ = 0.0;
    selected_marker_name_ = std::string(waypoint_name_prefix) + "1";

    scene_manager_ = context_->getSceneManager();
    ui_->setupUi(this);
    
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("waypoints", 1);

    connect(ui_->button_save,    SIGNAL(clicked()), this, SLOT(saveHandler()));
    connect(ui_->button_load,    SIGNAL(clicked()), this, SLOT(loadHandler()));
    connect(ui_->button_clear,   SIGNAL(clicked()), this, SLOT(clearHandler()));
    connect(ui_->button_publish, SIGNAL(clicked()), this, SLOT(publishHandler()));

    connect(ui_->x_spinbox,      SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->y_spinbox,      SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->z_spinbox,      SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->yaw_spinbox,    SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->height_spinbox, SIGNAL(valueChanged(double)), this, SLOT(heightChange(double)));
    
    connect(ui_->topic_input,    SIGNAL(editingFinished()), this, SLOT(topicChange()));
    connect(ui_->frame_input,    SIGNAL(editingFinished()), this, SLOT(frameChange()));
}

WaypointWidget::~WaypointWidget() {
    delete ui_;
    map_ptr_ = nullptr;
}

void WaypointWidget::enable() {
    show();
}

void WaypointWidget::disable() {
    path_publisher_.reset();
    hide();
}

void WaypointWidget::saveHandler() {
    QString filename = QFileDialog::getSaveFileName(
        0, tr("Waypoints Save"), "waypoints", 
        tr("Save Files (*.db3)")
    );
    if (filename.isEmpty()) {
        RCLCPP_ERROR(node_->get_logger(), "No saving file selected");
        return;
    }
    const std::string str_filename = filename.toStdString();
    RCLCPP_INFO_STREAM(node_->get_logger(), "Saving the waypoints into " << str_filename);

    if (filename.endsWith(".db3")) {
        saveBag(str_filename);
    }
    else {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Invalid saving file format: " << str_filename);
    }
}

void WaypointWidget::loadHandler() {
    const QString filename = QFileDialog::getOpenFileName(
        0, tr("Waypoint load"), "~/", 
        tr("Load Files (*.db3)")
    );
    if (filename.isEmpty()) {
        RCLCPP_ERROR(node_->get_logger(), "No loading file selected");
        return;
    }
    const std::string str_filename = filename.toStdString();
    RCLCPP_INFO(node_->get_logger(), "loading waypoints from %s", str_filename.c_str());
    if (filename.endsWith(".db3")) {
        loadBag(str_filename);
    } 
    else {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Invalid loading file format: " << str_filename);
    }
}

void WaypointWidget::clearHandler() {
    std::map<int, Ogre::SceneNode*>::iterator node_itr;
    for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); node_itr++) {
        scene_manager_->destroySceneNode(node_itr->second);
    }
    map_ptr_->clear();
    *unique_idx_ = 0;

    server_->clear();
    server_->applyChanges();
}

void WaypointWidget::publishHandler() {
    nav_msgs::msg::Path path;
    std::map<int, Ogre::SceneNode*>::iterator node_itr;

    for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); node_itr++) {
        Ogre::Vector3 position;
        position = node_itr->second->getPosition();

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = position.x;
        pose.pose.position.y = position.y;
        pose.pose.position.z = position.z;

        Ogre::Quaternion quat;
        quat = node_itr->second->getOrientation();
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;

        path.poses.push_back(pose);
    }
    path.header.frame_id = frame_id_.toStdString();
    path.header.stamp = node_->now();
    path_publisher_->publish(path);
}

void WaypointWidget::poseChange(double value) {
    auto node_entry = map_ptr_->end();
    try {
        const int selected_marker_idx = std::stoi(selected_marker_name_.substr(strlen(waypoint_name_prefix)));
        node_entry = map_ptr_->find(selected_marker_idx);
    }
    catch (const std::logic_error& e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
        return;
    }

    if (node_entry == map_ptr_->end())
        RCLCPP_ERROR(node_->get_logger(), "%s not found in map", selected_marker_name_.c_str());
    else
    {
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        getPose(position, quaternion);

        node_entry->second->setPosition(position);
        node_entry->second->setOrientation(quaternion);

        std::stringstream waypoint_name;
        waypoint_name << waypoint_name_prefix << node_entry->first;
        std::string waypoint_name_str(waypoint_name.str());

        visualization_msgs::msg::InteractiveMarker int_marker;
        if(server_->get(waypoint_name_str, int_marker)) {
            int_marker.pose.position.x = position.x;
            int_marker.pose.position.y = position.y;
            int_marker.pose.position.z = position.z;

            int_marker.pose.orientation.x = quaternion.x;
            int_marker.pose.orientation.y = quaternion.y;
            int_marker.pose.orientation.z = quaternion.z;
            int_marker.pose.orientation.w = quaternion.w;

            server_->setPose(waypoint_name_str, int_marker.pose, int_marker.header);
        }
        server_->applyChanges();
    }
}

void WaypointWidget::frameChange() {
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    QString new_frame = ui_->frame_input->text();

    if ((new_frame != frame_id_) && (new_frame != "")) {
        frame_id_ = new_frame;
        RCLCPP_INFO(node_->get_logger(), "new frame: %s", frame_id_.toStdString().c_str());

        std::map<int, Ogre::SceneNode *>::iterator node_itr;
        for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); node_itr++) {
            std::stringstream waypoint_name;
            waypoint_name << "waypoint" << node_itr->first;
            std::string waypoint_name_str(waypoint_name.str());

            visualization_msgs::msg::InteractiveMarker int_marker;
            if(server_->get(waypoint_name_str, int_marker)) {
                int_marker.header.frame_id = new_frame.toStdString();
                server_->setPose(waypoint_name_str, int_marker.pose, int_marker.header);
            }
        }
        server_->applyChanges();
    }
}

void WaypointWidget::topicChange() {
    QString new_topic = ui_->topic_input->text();

    if(new_topic != output_topic_) {
        path_publisher_.reset();
        output_topic_ = new_topic;
        if((output_topic_ != "") && (output_topic_ != "/")) {
            path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
                output_topic_.toStdString(), 1);
        }
    }
}

void WaypointWidget::heightChange(double value) {
    auto sn_entry = map_ptr_->end();
    try {
        const int selected_marker_idx = std::stoi(
            selected_marker_name_.substr((strlen((waypoint_name_prefix))))
        );
        sn_entry = map_ptr_->find(selected_marker_idx);
    } catch(const std::logic_error &error) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), error.what());
        return;
    }

    if (sn_entry == map_ptr_->end()) {
        RCLCPP_ERROR(node_->get_logger(), "%s not found in map", selected_marker_name_.c_str());
    } else {
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        getPose(position, quaternion);

        sn_entry->second->setPosition(position);
        sn_entry->second->setOrientation(quaternion);

        std::stringstream wp_name;
        wp_name << waypoint_name_prefix << sn_entry->first;
        std::string wp_name_str(wp_name.str());

        visualization_msgs::msg::InteractiveMarker int_marker;
        if (server_->get(wp_name_str, int_marker)) {
            int_marker.pose.position.x = position.x;
            int_marker.pose.position.y = position.y;
            int_marker.pose.position.z = position.z;

            int_marker.pose.orientation.x = quaternion.x;
            int_marker.pose.orientation.y = quaternion.y;
            int_marker.pose.orientation.z = quaternion.z;
            int_marker.pose.orientation.w = quaternion.w;

            server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
        }
        server_->applyChanges();
    }
}

void WaypointWidget::saveBag(const std::string& filename) {
    nav_msgs::msg::Path path;
    std::map<int, Ogre::SceneNode*>::iterator node_itr;
    for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); ++node_itr) {
        Ogre::Vector3 position;
        position = node_itr->second->getPosition();

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = position.x;
        pose.pose.position.y = position.y;
        pose.pose.position.z = position.z;

        Ogre::Quaternion quat;
        quat = node_itr->second->getOrientation();
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;

        path.poses.push_back(pose);
    }
    path.header.frame_id = frame_id_.toStdString();
    path.header.stamp = node_->now();

    try {
        rosbag2_cpp::Writer writer;
        
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = filename;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        writer.open(storage_options, converter_options);

        writer.create_topic({
            "waypoints",
            "nav_msgs/msg/Path",
            rmw_get_serialization_format(),
            ""
        });

        auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<nav_msgs::msg::Path> serialization;
        serialization.serialize_message(&path, serialized_msg.get());

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->topic_name = "waypoints";
        bag_message->time_stamp = node_->now().nanoseconds();
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t,
            [](rcutils_uint8_array_t* data) {
                auto fini_return = rcutils_uint8_array_fini(data);
                delete data;
                if (fini_return != RCUTILS_RET_OK) {
                    RCUTILS_LOG_ERROR_NAMED(
                        "waypoint_rviz2_plugin",
                        "Failed to destroy serialized message %s", rcutils_get_error_string().str);
                }
            });
        *bag_message->serialized_data = serialized_msg->release_rcl_serialized_message();

        writer.write(bag_message);
        
        RCLCPP_INFO(node_->get_logger(), "Waypoints saved successfully");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to save bag: %s", e.what());
    }
}

void WaypointWidget::loadBag(const std::string& filename) {
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
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                nav_msgs::msg::Path path;
                
                rclcpp::Serialization<nav_msgs::msg::Path> serialization;
                serialization.deserialize_message(&serialized_msg, &path);

                RCLCPP_INFO(node_->get_logger(), "n waypoints %zu", path.poses.size());

                for (size_t i = 0; i < path.poses.size(); i++) {
                    geometry_msgs::msg::PoseStamped pos = path.poses[i];
                    Ogre::Vector3 position;
                    position.x = pos.pose.position.x;
                    position.y = pos.pose.position.y;
                    position.z = pos.pose.position.z;

                    Ogre::Quaternion quaternion;
                    quaternion.x = pos.pose.orientation.x;
                    quaternion.y = pos.pose.orientation.y;
                    quaternion.z = pos.pose.orientation.z;
                    quaternion.w = pos.pose.orientation.w;

                    waypoint_tool_->makeItem(position, quaternion);
                }
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load bag: %s", e.what());
    }
}

void WaypointWidget::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quaternion) {
    ui_->x_spinbox->blockSignals(true);
    ui_->y_spinbox->blockSignals(true);
    ui_->z_spinbox->blockSignals(true);
    ui_->yaw_spinbox->blockSignals(true);

    ui_->x_spinbox->setValue(position.x);
    ui_->y_spinbox->setValue(position.y);
    ui_->z_spinbox->setValue(position.z);

    tf2::Quaternion qt(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 m(qt);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ui_->yaw_spinbox->setValue(yaw);

    ui_->x_spinbox->blockSignals(false);
    ui_->y_spinbox->blockSignals(false);
    ui_->z_spinbox->blockSignals(false);
    ui_->yaw_spinbox->blockSignals(false);
}

void WaypointWidget::setConfig(QString topic, QString frame, float height) {
    {
        std::lock_guard<std::mutex> lock(frame_updates_mutex_);
        ui_->topic_input->blockSignals(true);
        ui_->frame_input->blockSignals(true);
        ui_->height_spinbox->blockSignals(true);

        ui_->topic_input->setText(topic);
        ui_->frame_input->setText(frame);
        ui_->height_spinbox->setValue(height);

        ui_->topic_input->blockSignals(false);
        ui_->frame_input->blockSignals(false);
        ui_->height_spinbox->blockSignals(false);
    }

    topicChange();
    frameChange();
    heightChange(height);
}

void WaypointWidget::setWaypointLabel(Ogre::Vector3 position) {
    std::ostringstream string_stream;
    string_stream.precision(2);
    string_stream << selected_marker_name_;
    std::string label = string_stream.str();
    ui_->text_selected->setText(QString::fromStdString(label));
}

void WaypointWidget::setWaypointCount(int size) {
    std::ostringstream string_stream;
    string_stream << "Total waypoints: " << size;
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    ui_->text_total->setText(
        QString::fromStdString(string_stream.str())
    );
}

void WaypointWidget::setSelectedMarkerName(std::string name) {
    selected_marker_name_ = name;
}

void WaypointWidget::getPose(Ogre::Vector3& position, Ogre::Quaternion& quaternion) {
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    position.x = ui_->x_spinbox->value();
    position.y = ui_->y_spinbox->value();
    position.z = ui_->z_spinbox->value();
    double yaw = ui_->yaw_spinbox->value();

    tf2::Quaternion qt;
    qt.setRPY(0.0, 0.0, yaw);
    quaternion.x = qt.x();
    quaternion.y = qt.y();
    quaternion.z = qt.z();
    quaternion.w = qt.w();
}

double WaypointWidget::getDefaultHeight() {
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    return default_height_;
}

QString WaypointWidget::getFrameId() {
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    return frame_id_;
}

QString WaypointWidget::getOutputTopic() {
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    return output_topic_;
}

}
