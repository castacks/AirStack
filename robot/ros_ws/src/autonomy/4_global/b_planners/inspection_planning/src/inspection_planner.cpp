// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "inspection_planner.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <map>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

InspectionPlanner::InspectionPlanner()
    : Node("inspection_planner"),
      is_active_(true),
      vdb_map_received_(false),
      odometry_received_(false) {
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    this->declare_parameter("robot_frame_id", "base_link");
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("pub_global_plan_topic", "~/global_plan");
    this->declare_parameter("sub_vdb_map_topic", "vdb_map_visualization");
    this->declare_parameter("sub_observed_semantics_topic", "observed_semantics");
    this->declare_parameter("sub_predicted_semantics_topic", "predicted_semantics");
    this->declare_parameter("sub_odometry_topic", "/odometry");
    this->declare_parameter("srv_inspection_toggle_topic", "~/inspection_toggle");
    this->declare_parameter("pub_observed_inspection_poses_topic", "~/observed_inspection_poses");
    this->declare_parameter("pub_predicted_inspection_poses_topic", "~/predicted_inspection_poses");
    this->declare_parameter("pub_observed_semantic_poses_topic", "~/observed_semantic_poses");
    this->declare_parameter("pub_predicted_semantic_poses_topic", "~/predicted_semantic_poses");
    this->declare_parameter("pub_observed_local_frames_topic", "~/observed_local_frames");
    this->declare_parameter("pub_predicted_local_frames_topic", "~/predicted_local_frames");
    this->declare_parameter("inspection_offsets",
                            std::vector<double>{1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                -1.0, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter("robot_radius", 0.5);
    this->declare_parameter("workspace_x_min", -10.0);
    this->declare_parameter("workspace_x_max", 10.0);
    this->declare_parameter("workspace_y_min", -10.0);
    this->declare_parameter("workspace_y_max", 10.0);
    this->declare_parameter("workspace_z_min", 0.0);
    this->declare_parameter("workspace_z_max", 5.0);

    // Get parameters
    robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    pub_global_plan_topic_ = this->get_parameter("pub_global_plan_topic").as_string();
    sub_vdb_map_topic_ = this->get_parameter("sub_vdb_map_topic").as_string();
    sub_observed_semantics_topic_ = this->get_parameter("sub_observed_semantics_topic").as_string();
    sub_predicted_semantics_topic_ =
        this->get_parameter("sub_predicted_semantics_topic").as_string();
    sub_odometry_topic_ = this->get_parameter("sub_odometry_topic").as_string();
    srv_inspection_toggle_topic_ = this->get_parameter("srv_inspection_toggle_topic").as_string();
    pub_observed_inspection_poses_topic_ =
        this->get_parameter("pub_observed_inspection_poses_topic").as_string();
    pub_predicted_inspection_poses_topic_ =
        this->get_parameter("pub_predicted_inspection_poses_topic").as_string();
    pub_observed_semantic_poses_topic_ =
        this->get_parameter("pub_observed_semantic_poses_topic").as_string();
    pub_predicted_semantic_poses_topic_ =
        this->get_parameter("pub_predicted_semantic_poses_topic").as_string();
    pub_observed_local_frames_topic_ =
        this->get_parameter("pub_observed_local_frames_topic").as_string();
    pub_predicted_local_frames_topic_ =
        this->get_parameter("pub_predicted_local_frames_topic").as_string();
    // Parse inspection offsets - ROS2 doesn't support nested arrays directly in YAML
    // So we'll use a simpler approach with individual parameters
    inspection_offsets_ = {
        {1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};

    // Try to get the parameter as a string array and parse it manually if needed
    try {
        auto offsets_param = this->get_parameter("inspection_offsets");
        if (offsets_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            // If it's a flat double array, assume it's flattened [x1,y1,z1,x2,y2,z2,...]
            auto flat_offsets = offsets_param.as_double_array();
            if (flat_offsets.size() % 3 == 0) {
                inspection_offsets_.clear();
                for (size_t i = 0; i < flat_offsets.size(); i += 3) {
                    inspection_offsets_.push_back(
                        {flat_offsets[i], flat_offsets[i + 1], flat_offsets[i + 2]});
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not parse inspection_offsets parameter, using defaults: %s", e.what());
    }
    
    // Get robot radius parameter
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    
    // Get workspace boundary parameters
    workspace_x_min_ = this->get_parameter("workspace_x_min").as_double();
    workspace_x_max_ = this->get_parameter("workspace_x_max").as_double();
    workspace_y_min_ = this->get_parameter("workspace_y_min").as_double();
    workspace_y_max_ = this->get_parameter("workspace_y_max").as_double();
    workspace_z_min_ = this->get_parameter("workspace_z_min").as_double();
    workspace_z_max_ = this->get_parameter("workspace_z_max").as_double();
}

void InspectionPlanner::initialize() {
    // Create publishers
    pub_global_plan_ = this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
    pub_observed_inspection_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_observed_inspection_poses_topic_, 10);
    pub_predicted_inspection_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_predicted_inspection_poses_topic_, 10);
    pub_observed_semantic_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_observed_semantic_poses_topic_, 10);
    pub_predicted_semantic_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_predicted_semantic_poses_topic_, 10);
    pub_observed_local_frames_ =
        this->create_publisher<geometry_msgs::msg::PoseArray>(pub_observed_local_frames_topic_, 10);
    pub_predicted_local_frames_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_predicted_local_frames_topic_, 10);

    // Create subscribers
    sub_vdb_map_ = this->create_subscription<visualization_msgs::msg::Marker>(
        sub_vdb_map_topic_, 10,
        std::bind(&InspectionPlanner::vdbMapCallback, this, std::placeholders::_1));

    sub_observed_semantics_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        sub_observed_semantics_topic_, 10,
        std::bind(&InspectionPlanner::observedSemanticsCallback, this, std::placeholders::_1));

    sub_predicted_semantics_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        sub_predicted_semantics_topic_, 10,
        std::bind(&InspectionPlanner::predictedSemanticsCallback, this, std::placeholders::_1));

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odometry_topic_, 10,
        std::bind(&InspectionPlanner::odometryCallback, this, std::placeholders::_1));

    // Create services
    srv_inspection_toggle_ = this->create_service<std_srvs::srv::Trigger>(
        srv_inspection_toggle_topic_, std::bind(&InspectionPlanner::inspectionToggleCallback, this,
                                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Inspection Planner initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to VDB map: %s", sub_vdb_map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to observed semantics: %s",
                sub_observed_semantics_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to predicted semantics: %s",
                sub_predicted_semantics_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to odometry: %s", sub_odometry_topic_.c_str());
}

void InspectionPlanner::vdbMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    vdb_map_data_ = *msg;
    vdb_map_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Received VDB map marker: %s", msg->ns.c_str());

    // Generate inspection plan if active
    if (is_active_) {
        generateInspectionPlan();
    }
}

void InspectionPlanner::observedSemanticsCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Clear existing observed semantics
    observed_semantics_.clear();

    // Create a map to match sphere markers with their corresponding arrow markers
    std::map<int, SemanticObject> sphere_objects;
    std::map<int, geometry_msgs::msg::Vector3> arrow_axes;

    // First pass: collect all sphere markers and arrow markers
    for (const auto& marker : msg->markers) {
        if (marker.type == visualization_msgs::msg::Marker::SPHERE) {
            SemanticObject semantic_obj = extractSemanticFromMarker(marker);
            sphere_objects[marker.id] = semantic_obj;
        } else if (marker.type == visualization_msgs::msg::Marker::ARROW &&
                   marker.ns == "panoptic_axes") {
            // Arrow markers have id offset by 2000 from their corresponding sphere markers
            int sphere_id = marker.id - 2000;
            geometry_msgs::msg::Vector3 axis = extractPrincipalAxisFromArrow(marker);
            arrow_axes[sphere_id] = axis;
        }
    }

    // Second pass: combine sphere objects with their principal axes
    for (auto& [sphere_id, obj] : sphere_objects) {
        if (arrow_axes.find(sphere_id) != arrow_axes.end()) {
            obj.principal_axis = arrow_axes[sphere_id];
        }
        observed_semantics_.push_back(obj);
    }

    // RCLCPP_INFO(this->get_logger(), "Received %zu observed semantic objects",
    //             observed_semantics_.size());

    // Generate inspection plan if active
    if (is_active_) {
        generateInspectionPlan();
    }
}

void InspectionPlanner::predictedSemanticsCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Clear existing predicted semantics
    predicted_semantics_.clear();

    // Create a map to match sphere markers with their corresponding arrow markers
    std::map<int, SemanticObject> sphere_objects;
    std::map<int, geometry_msgs::msg::Vector3> arrow_axes;

    // First pass: collect all sphere markers and arrow markers
    for (const auto& marker : msg->markers) {
        if (marker.type == visualization_msgs::msg::Marker::SPHERE) {
            SemanticObject semantic_obj = extractSemanticFromMarker(marker);
            sphere_objects[marker.id] = semantic_obj;
        } else if (marker.type == visualization_msgs::msg::Marker::ARROW &&
                   marker.ns == "panoptic_axes") {
            // Arrow markers have id offset by 2000 from their corresponding sphere markers
            int sphere_id = marker.id - 2000;
            geometry_msgs::msg::Vector3 axis = extractPrincipalAxisFromArrow(marker);
            arrow_axes[sphere_id] = axis;
        }
    }

    // Second pass: combine sphere objects with their principal axes
    for (auto& [sphere_id, obj] : sphere_objects) {
        if (arrow_axes.find(sphere_id) != arrow_axes.end()) {
            obj.principal_axis = arrow_axes[sphere_id];
        }
        predicted_semantics_.push_back(obj);
    }

    RCLCPP_DEBUG(this->get_logger(), "Received %zu predicted semantic objects",
                 predicted_semantics_.size());

    // Generate inspection plan if active
    if (is_active_) {
        generateInspectionPlan();
    }
}

void InspectionPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_odometry_ = *msg;
    odometry_received_ = true;

    RCLCPP_DEBUG(this->get_logger(), "Received odometry: position [%.2f, %.2f, %.2f]",
                 msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void InspectionPlanner::inspectionToggleCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    is_active_ = !is_active_;

    if (is_active_) {
        RCLCPP_INFO(this->get_logger(), "Inspection planning activated");
        if (vdb_map_received_ && odometry_received_) {
            generateInspectionPlan();
        } else {
            RCLCPP_WARN(this->get_logger(), "VDB map or odometry not yet received, waiting...");
        }
        response->success = true;
        response->message = "Inspection planning activated";
    } else {
        RCLCPP_INFO(this->get_logger(), "Inspection planning deactivated");
        response->success = true;
        response->message = "Inspection planning deactivated";
    }
}

void InspectionPlanner::generateInspectionPlan() {
    RCLCPP_INFO(this->get_logger(), "Generating inspection plan...");

    // Log current semantics data
    logSemanticsInfo();

    // Create inspection path
    nav_msgs::msg::Path inspection_path = createInspectionPath();

    // Generate inspection poses
    geometry_msgs::msg::PoseArray observed_inspection_poses_raw =
        generateInspectionPoses(observed_semantics_);
    geometry_msgs::msg::PoseArray predicted_inspection_poses_raw =
        generateInspectionPoses(predicted_semantics_);
    
    // Filter out poses that are in collision with VDB map
    geometry_msgs::msg::PoseArray observed_collision_free =
        filterCollisionFreePoses(observed_inspection_poses_raw);
    geometry_msgs::msg::PoseArray predicted_collision_free =
        filterCollisionFreePoses(predicted_inspection_poses_raw);
    
    // Filter out poses that are outside workspace boundaries
    geometry_msgs::msg::PoseArray observed_inspection_poses =
        filterWorkspacePoses(observed_collision_free);
    geometry_msgs::msg::PoseArray predicted_inspection_poses =
        filterWorkspacePoses(predicted_collision_free);

    // Generate and publish semantic object poses and local frames
    geometry_msgs::msg::PoseArray observed_semantic_poses =
        generateSemanticObjectPoses(observed_semantics_);
    geometry_msgs::msg::PoseArray predicted_semantic_poses =
        generateSemanticObjectPoses(predicted_semantics_);
    geometry_msgs::msg::PoseArray observed_local_frames =
        generateLocalFramePoses(observed_semantics_);
    geometry_msgs::msg::PoseArray predicted_local_frames =
        generateLocalFramePoses(predicted_semantics_);

    // Publish all poses
    // pub_global_plan_->publish(inspection_path);
    pub_observed_inspection_poses_->publish(observed_inspection_poses);
    pub_predicted_inspection_poses_->publish(predicted_inspection_poses);
    pub_observed_semantic_poses_->publish(observed_semantic_poses);
    pub_predicted_semantic_poses_->publish(predicted_semantic_poses);
    pub_observed_local_frames_->publish(observed_local_frames);
    pub_predicted_local_frames_->publish(predicted_local_frames);

    RCLCPP_INFO(this->get_logger(),
                "Inspection plan generated with %zu waypoints, %zu observed inspection poses, %zu "
                "predicted inspection poses, %zu observed objects, %zu predicted objects",
                inspection_path.poses.size(), observed_inspection_poses.poses.size(),
                predicted_inspection_poses.poses.size(), observed_semantic_poses.poses.size(),
                predicted_semantic_poses.poses.size());
}

nav_msgs::msg::Path InspectionPlanner::createInspectionPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_id_;
    path.header.stamp = this->get_clock()->now();

    // Simple inspection strategy: visit all semantic points
    std::vector<SemanticObject> all_semantic_objects;

    // Add observed semantics
    for (const auto& obj : observed_semantics_) {
        all_semantic_objects.push_back(obj);
    }

    // Add predicted semantics
    for (const auto& obj : predicted_semantics_) {
        all_semantic_objects.push_back(obj);
    }

    // Create waypoints (simple approach - can be enhanced with more sophisticated planning)
    for (const auto& obj : all_semantic_objects) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_id_;
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position = obj.position;

        // Use principal axis for orientation if available
        if (obj.principal_axis.x != 0.0 || obj.principal_axis.y != 0.0 ||
            obj.principal_axis.z != 0.0) {
            // Convert principal axis vector to quaternion orientation
            tf2::Vector3 axis(obj.principal_axis.x, obj.principal_axis.y, obj.principal_axis.z);
            tf2::Vector3 default_forward(1.0, 0.0, 0.0);
            tf2::Vector3 cross = default_forward.cross(axis.normalized());
            double dot = default_forward.dot(axis.normalized());

            tf2::Quaternion q;
            if (cross.length() > 1e-6) {
                q.setRotation(cross.normalized(), acos(std::max(-1.0, std::min(1.0, dot))));
            } else {
                // q.setIdentity();
            }

            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
        } else {
            pose.pose.orientation.w = 1.0;  // Default orientation
        }

        path.poses.push_back(pose);
    }

    return path;
}

geometry_msgs::msg::Point InspectionPlanner::transformPoint(const geometry_msgs::msg::Point& point,
                                                            const std::string& target_frame) {
    geometry_msgs::msg::PointStamped input_point;
    input_point.header.frame_id = robot_frame_id_;
    input_point.header.stamp = this->get_clock()->now();
    input_point.point = point;

    try {
        geometry_msgs::msg::PointStamped output_point;
        tf_buffer_->transform(input_point, output_point, target_frame);
        return output_point.point;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        return point;  // Return original point if transform fails
    }
}

double InspectionPlanner::calculateDistance(const geometry_msgs::msg::Point& p1,
                                            const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void InspectionPlanner::logSemanticsInfo() {
    RCLCPP_INFO(this->get_logger(), "Current semantics data:");
    RCLCPP_INFO(this->get_logger(), "  Observed semantics: %zu objects",
                observed_semantics_.size());
    RCLCPP_INFO(this->get_logger(), "  Predicted semantics: %zu objects",
                predicted_semantics_.size());
    RCLCPP_INFO(this->get_logger(), "  VDB map received: %s", vdb_map_received_ ? "Yes" : "No");
    RCLCPP_INFO(this->get_logger(), "  Odometry received: %s", odometry_received_ ? "Yes" : "No");
    if (odometry_received_) {
        RCLCPP_INFO(this->get_logger(), "  Current position: [%.2f, %.2f, %.2f]",
                    current_odometry_.pose.pose.position.x, current_odometry_.pose.pose.position.y,
                    current_odometry_.pose.pose.position.z);
    }

    // Log details about semantic objects
    for (size_t i = 0; i < observed_semantics_.size(); ++i) {
        const auto& obj = observed_semantics_[i];
        RCLCPP_DEBUG(
            this->get_logger(),
            "  Observed[%zu]: %s instance %d at [%.2f, %.2f, %.2f], axis [%.2f, %.2f, %.2f]", i,
            obj.semantic_class.c_str(), obj.instance_id, obj.position.x, obj.position.y,
            obj.position.z, obj.principal_axis.x, obj.principal_axis.y, obj.principal_axis.z);
    }

    for (size_t i = 0; i < predicted_semantics_.size(); ++i) {
        const auto& obj = predicted_semantics_[i];
        RCLCPP_DEBUG(
            this->get_logger(),
            "  Predicted[%zu]: %s instance %d at [%.2f, %.2f, %.2f], axis [%.2f, %.2f, %.2f]", i,
            obj.semantic_class.c_str(), obj.instance_id, obj.position.x, obj.position.y,
            obj.position.z, obj.principal_axis.x, obj.principal_axis.y, obj.principal_axis.z);
    }
}

void InspectionPlanner::generateLocalFrame(const geometry_msgs::msg::Vector3& principal_axis,
                                           tf2::Vector3& x_axis, tf2::Vector3& y_axis,
                                           tf2::Vector3& z_axis) {
    // X-axis: principal axis of the semantic object
    x_axis.setValue(principal_axis.x, principal_axis.y, principal_axis.z);

    // Handle case where principal axis is not set or is zero
    if (x_axis.length() < 1e-6) {
        x_axis.setValue(1.0, 0.0, 0.0);  // Default to forward
    } else {
        x_axis.normalize();
    }

    // Generate Y-axis (left direction) - perpendicular to principal axis
    tf2::Vector3 world_up(0.0, 0.0, 1.0);
    tf2::Vector3 world_y(0.0, 1.0, 0.0);

    // Check if principal axis is near vertical (close to parallel with world Z-axis)
    double vertical_threshold = 0.9;  // cos(~25.8 degrees)
    double dot_with_vertical = std::abs(x_axis.dot(world_up));

    if (dot_with_vertical > vertical_threshold) {
        // Principal axis is near vertical - align local Y-axis with world Y-axis
        // Project world Y onto the plane perpendicular to the principal axis
        y_axis = world_y - world_y.dot(x_axis) * x_axis;

        if (y_axis.length() < 1e-6) {
            // If world Y is exactly parallel to principal axis, use world X as fallback
            tf2::Vector3 world_x(1.0, 0.0, 0.0);
            y_axis = world_x - world_x.dot(x_axis) * x_axis;
        }
    } else {
        // Principal axis is not near vertical - use cross product method
        y_axis = x_axis.cross(world_up);  // Note: switched order for correct handedness

        if (y_axis.length() < 1e-6) {
            // If principal axis is exactly aligned with world up, use world Y as reference
            y_axis = world_y - world_y.dot(x_axis) * x_axis;
            if (y_axis.length() < 1e-6) {
                // Fallback to arbitrary perpendicular vector
                y_axis.setValue(0.0, 1.0, 0.0);
            }
        }
    }
    y_axis.normalize();

    // Z-axis (up direction) - cross product of X and Y to ensure right-handed coordinate system
    z_axis = x_axis.cross(y_axis);
    if (z_axis.length() < 1e-6) {
        z_axis.setValue(0.0, 0.0, 1.0);
    }
    z_axis.normalize();
}

SemanticObject InspectionPlanner::extractSemanticFromMarker(
    const visualization_msgs::msg::Marker& marker) {
    SemanticObject obj;

    // Extract position from marker pose
    obj.position = marker.pose.position;

    // Extract semantic class from marker text (format: "class_name_#instance_id")
    std::string marker_text = marker.text;
    if (marker_text.empty() && marker.ns == "panoptic_instances") {
        // If no text, try to infer from namespace and id
        obj.semantic_class = "object";
        obj.instance_id = marker.id;
    } else {
        // Parse text to extract class name and instance id
        size_t hash_pos = marker_text.find('#');
        if (hash_pos != std::string::npos) {
            obj.semantic_class = marker_text.substr(0, hash_pos);
            try {
                obj.instance_id = std::stoi(marker_text.substr(hash_pos + 1));
            } catch (...) {
                obj.instance_id = marker.id;
            }
        } else {
            obj.semantic_class = marker_text;
            obj.instance_id = marker.id;
        }
    }

    // Set frame and timestamp
    obj.frame_id = marker.header.frame_id;
    obj.timestamp = rclcpp::Time(marker.header.stamp);

    // Initialize principal axis (will be updated if arrow marker is found)
    obj.principal_axis.x = 0.0;
    obj.principal_axis.y = 0.0;
    obj.principal_axis.z = 0.0;

    return obj;
}

geometry_msgs::msg::Vector3 InspectionPlanner::extractPrincipalAxisFromArrow(
    const visualization_msgs::msg::Marker& arrow_marker) {
    geometry_msgs::msg::Vector3 axis;

    if (arrow_marker.type == visualization_msgs::msg::Marker::ARROW &&
        arrow_marker.points.size() >= 2) {
        // Calculate direction vector from start to end point
        const auto& start = arrow_marker.points[0];
        const auto& end = arrow_marker.points[1];

        axis.x = end.x - start.x;
        axis.y = end.y - start.y;
        axis.z = end.z - start.z;

        // Normalize the vector
        double length = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        if (length > 1e-6) {
            axis.x /= length;
            axis.y /= length;
            axis.z /= length;
        }
    } else {
        // Default axis if arrow marker is invalid
        axis.x = 1.0;
        axis.y = 0.0;
        axis.z = 0.0;
    }

    return axis;
}

geometry_msgs::msg::PoseArray InspectionPlanner::generateInspectionPoses(
    const std::vector<SemanticObject>& semantic_objects) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = map_frame_id_;
    pose_array.header.stamp = this->get_clock()->now();

    for (const auto& obj : semantic_objects) {
        // For each semantic object, generate inspection poses for all offset positions
        for (const auto& offset_vec : inspection_offsets_) {
            if (offset_vec.size() >= 3) {
                geometry_msgs::msg::Point offset;
                offset.x = offset_vec[0];
                offset.y = offset_vec[1];
                offset.z = offset_vec[2];

                geometry_msgs::msg::Pose inspection_pose = calculateInspectionPose(obj, offset);
                pose_array.poses.push_back(inspection_pose);
            }
        }
    }

    return pose_array;
}

geometry_msgs::msg::Pose InspectionPlanner::calculateInspectionPose(
    const SemanticObject& obj, const geometry_msgs::msg::Point& offset) {
    geometry_msgs::msg::Pose pose;

    // Create local coordinate frame based on semantic object
    // X-axis: principal axis of the semantic object
    // Y-axis: left of the semantic object (perpendicular to principal axis)
    // Z-axis: up (computed as cross product)
    tf2::Vector3 x_axis, y_axis, z_axis;
    generateLocalFrame(obj.principal_axis, x_axis, y_axis, z_axis);

    // Transform offset from local frame to world frame
    // tf2::Matrix3x3 constructor takes rows, so we need to transpose our column vectors
    tf2::Matrix3x3 rotation_matrix(x_axis.x(), x_axis.y(), x_axis.z(), y_axis.x(), y_axis.y(),
                                   y_axis.z(), z_axis.x(), z_axis.y(), z_axis.z());

    tf2::Vector3 offset_local(offset.x, offset.y, offset.z);
    tf2::Vector3 offset_world = rotation_matrix * offset_local;
    if (abs(offset_world.x()) < 0.1) {
        std::cout
            << "------------------------------------------------------------------------------"
            << std::endl;
        std::cout << "offset_world: " << offset_world.x() << ", " << offset_world.y() << ", "
                  << offset_world.z() << std::endl;
    }

    // Calculate inspection position in world frame
    pose.position.x = obj.position.x + offset_world.x();
    pose.position.y = obj.position.y + offset_world.y();
    pose.position.z = obj.position.z + offset_world.z();

    // Calculate inspection pose orientation:
    // - X-axis should be parallel to semantic object's local Y-axis AND point toward object
    // - Z-axis should be vertical up
    // - Semantic object should be on positive X-axis direction
    // - Y-axis computed to complete right-handed coordinate system

    // Direction from inspection pose to semantic object
    tf2::Vector3 to_object(obj.position.x - pose.position.x, obj.position.y - pose.position.y,
                           obj.position.z - pose.position.z);

    tf2::Vector3 robot_x, robot_y, robot_z;

    // Check if the distance is too small (robot at same position as object)
    if (to_object.length() < 1e-6) {
        // Use default orientation if robot is at object position
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }

    // to_object.normalize();

    // Robot's Z-axis is vertical up
    robot_z.setValue(0.0, 0.0, 1.0);

    // We want X-axis parallel to object's Y-axis, but pointing toward the object
    // Project the object's Y-axis onto the plane perpendicular to Z-axis (horizontal plane)
    tf2::Vector3 obj_y_horizontal(y_axis.x(), y_axis.y(), 0.0);

    if (obj_y_horizontal.length() < 1e-6) {
        // Object's Y-axis is vertical, use direction to object in horizontal plane
        tf2::Vector3 to_obj_horizontal(to_object.x(), to_object.y(), 0.0);
        if (to_obj_horizontal.length() > 1e-6) {
            robot_x = to_obj_horizontal.normalized();
        } else {
            // Object is directly above/below, use object's X-axis projected horizontally
            tf2::Vector3 obj_x_horizontal(x_axis.x(), x_axis.y(), 0.0);
            if (obj_x_horizontal.length() > 1e-6) {
                robot_x = obj_x_horizontal.normalized();
            } else {
                robot_x.setValue(1.0, 0.0, 0.0);  // Default fallback
            }
        }
    } else {
        // Object's Y-axis has horizontal component
        obj_y_horizontal.normalize();

        // Choose the direction of object's Y-axis that points toward the object
        tf2::Vector3 to_obj_horizontal(to_object.x(), to_object.y(), 0.0);
        if (to_obj_horizontal.length() > 1e-6) {
            // to_obj_horizontal.normalize();

            // Check which direction of the Y-axis is closer to the object direction
            double dot_positive = obj_y_horizontal.dot(to_obj_horizontal);
            double dot_negative = (-obj_y_horizontal).dot(to_obj_horizontal);

            // Transform robot position to object's local coordinate frame
            tf2::Vector3 robot_pos_world(pose.position.x, pose.position.y, pose.position.z);
            tf2::Vector3 obj_pos_world(obj.position.x, obj.position.y, obj.position.z);
            tf2::Vector3 robot_relative = robot_pos_world - obj_pos_world;

            // Project robot position onto object's local y-axis to get local y coordinate
            double robot_local_y = offset_local.y();

            if (abs(offset_world.x()) < 0.1) {
                std::cout << "robot_local_y: " << robot_local_y << std::endl;
            }

            if (abs(offset_world.x()) < 0.1) {
                std::cout << "obj_y_horizontal: " << obj_y_horizontal.x() << ", "
                          << obj_y_horizontal.y() << ", " << obj_y_horizontal.z() << std::endl;
                std::cout << "to_obj_horizontal: " << to_obj_horizontal.x() << ", "
                          << to_obj_horizontal.y() << ", " << to_obj_horizontal.z() << std::endl;
                std::cout << "dot_positive: " << dot_positive << std::endl;
                std::cout << "dot_negative: " << dot_negative << std::endl;
            }

            // Set robot_x based on robot's position in object's local frame
            if (robot_local_y < 0.0) {
                // Robot is on negative y-side, point toward positive y direction
                robot_x = obj_y_horizontal;
            } else {
                // Robot is on positive y-side, point toward negative y direction
                robot_x = -obj_y_horizontal;
            }

            if (abs(obj_y_horizontal.x()) < 0.1) {
                if (robot_local_y < 0.0) {
                    // Robot is on negative y-side, point toward positive y direction
                    robot_x = -obj_y_horizontal;
                } else {
                    // Robot is on positive y-side, point toward negative y direction
                    robot_x = obj_y_horizontal;
                }
            }
        } else {
            // Object is directly above/below, just use positive Y-axis direction
            robot_x = obj_y_horizontal;
        }
    }

    // Robot's Y-axis computed as cross product to ensure right-handed system
    robot_y = robot_z.cross(robot_x);

    robot_x.normalize();
    robot_y.normalize();
    robot_z.normalize();

    // Validate all vectors before creating rotation matrix
    if (robot_x.length() < 1e-6 || robot_y.length() < 1e-6 || robot_z.length() < 1e-6) {
        // Fallback to identity orientation if vectors are invalid
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }

    // Create rotation matrix and convert to quaternion
    // tf2::Matrix3x3 constructor takes rows, so we need to transpose our column vectors
    tf2::Matrix3x3 robot_rotation(robot_x.x(), robot_x.y(), robot_x.z(), robot_y.x(), robot_y.y(),
                                  robot_y.z(), robot_z.x(), robot_z.y(), robot_z.z());

    tf2::Quaternion robot_orientation;
    robot_rotation.getRotation(robot_orientation);

    // Validate quaternion before assigning
    if (std::isnan(robot_orientation.x()) || std::isnan(robot_orientation.y()) ||
        std::isnan(robot_orientation.z()) || std::isnan(robot_orientation.w())) {
        // Fallback to identity orientation if quaternion is invalid
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        RCLCPP_WARN_ONCE(this->get_logger(),
                         "Invalid quaternion detected, using identity orientation");
    } else {
        pose.orientation.x = robot_orientation.x();
        pose.orientation.y = robot_orientation.y();
        pose.orientation.z = robot_orientation.z();
        pose.orientation.w = robot_orientation.w();
    }

    return pose;
}

geometry_msgs::msg::PoseArray InspectionPlanner::generateSemanticObjectPoses(
    const std::vector<SemanticObject>& semantic_objects) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = map_frame_id_;
    pose_array.header.stamp = this->get_clock()->now();

    for (const auto& obj : semantic_objects) {
        geometry_msgs::msg::Pose object_pose = createSemanticObjectPose(obj);
        pose_array.poses.push_back(object_pose);
    }

    return pose_array;
}

geometry_msgs::msg::PoseArray InspectionPlanner::generateLocalFramePoses(
    const std::vector<SemanticObject>& semantic_objects) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = map_frame_id_;
    pose_array.header.stamp = this->get_clock()->now();

    for (const auto& obj : semantic_objects) {
        geometry_msgs::msg::Pose frame_pose = createLocalFramePose(obj);
        pose_array.poses.push_back(frame_pose);
    }

    return pose_array;
}

geometry_msgs::msg::Pose InspectionPlanner::createSemanticObjectPose(const SemanticObject& obj) {
    geometry_msgs::msg::Pose pose;

    // Position is the semantic object's position
    pose.position = obj.position;

    // Orientation represents the object's principal axis direction
    // Create a coordinate frame where X-axis is the principal axis
    tf2::Vector3 x_axis, y_axis, z_axis;

    // Handle case where principal axis is not set or is zero
    if (obj.principal_axis.x == 0.0 && obj.principal_axis.y == 0.0 && obj.principal_axis.z == 0.0) {
        // Use identity orientation for objects without principal axis
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }

    // Generate local coordinate frame
    generateLocalFrame(obj.principal_axis, x_axis, y_axis, z_axis);

    // Create rotation matrix and convert to quaternion
    // tf2::Matrix3x3 constructor takes rows, so we need to transpose our column vectors
    tf2::Matrix3x3 rotation(x_axis.x(), x_axis.y(), x_axis.z(), y_axis.x(), y_axis.y(), y_axis.z(),
                            z_axis.x(), z_axis.y(), z_axis.z());

    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);

    // Validate quaternion before assigning
    if (std::isnan(quaternion.x()) || std::isnan(quaternion.y()) || std::isnan(quaternion.z()) ||
        std::isnan(quaternion.w())) {
        // Fallback to identity orientation if quaternion is invalid
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
    } else {
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
    }

    return pose;
}

geometry_msgs::msg::Pose InspectionPlanner::createLocalFramePose(const SemanticObject& obj) {
    // Local frame pose is the same as semantic object pose
    // The local frame has its origin at the object position
    // X-axis aligned with principal axis, Y-axis to the left, Z-axis up
    return createSemanticObjectPose(obj);
}

bool InspectionPlanner::isInCollision(const geometry_msgs::msg::Point& position) const {
    // Check if the robot at given position collides with any voxel in the VDB map
    if (!vdb_map_received_ || vdb_map_data_.type != visualization_msgs::msg::Marker::CUBE_LIST) {
        // No VDB map data available, assume no collision
        return false;
    }
    
    // Get voxel size from the marker
    double voxel_size = vdb_map_data_.scale.x;
    
    // Check distance to each voxel in the VDB map
    for (const auto& voxel_point : vdb_map_data_.points) {
        // Calculate distance from robot position to voxel center
        double dx = position.x - voxel_point.x;
        double dy = position.y - voxel_point.y;
        double dz = position.z - voxel_point.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        
        // Check if robot radius overlaps with voxel
        // Distance threshold = robot_radius + half_voxel_size
        double collision_threshold = robot_radius_ + (voxel_size / 2.0);
        
        if (distance < collision_threshold) {
            return true;  // Collision detected
        }
    }
    
    return false;  // No collision
}

bool InspectionPlanner::isInsideWorkspace(const geometry_msgs::msg::Point& position) const {
    // Check if position is within the defined workspace boundaries
    return (position.x >= workspace_x_min_ && position.x <= workspace_x_max_ &&
            position.y >= workspace_y_min_ && position.y <= workspace_y_max_ &&
            position.z >= workspace_z_min_ && position.z <= workspace_z_max_);
}

geometry_msgs::msg::PoseArray InspectionPlanner::filterCollisionFreePoses(
    const geometry_msgs::msg::PoseArray& poses) const {
    geometry_msgs::msg::PoseArray collision_free_poses;
    collision_free_poses.header = poses.header;
    
    int total_poses = poses.poses.size();
    int collision_free_count = 0;
    
    for (const auto& pose : poses.poses) {
        if (!isInCollision(pose.position)) {
            collision_free_poses.poses.push_back(pose);
            collision_free_count++;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Collision filtering: %d/%d poses are collision-free (robot_radius: %.2f m)",
                collision_free_count, total_poses, robot_radius_);
    
    return collision_free_poses;
}

geometry_msgs::msg::PoseArray InspectionPlanner::filterWorkspacePoses(
    const geometry_msgs::msg::PoseArray& poses) const {
    geometry_msgs::msg::PoseArray workspace_poses;
    workspace_poses.header = poses.header;
    
    int total_poses = poses.poses.size();
    int inside_workspace_count = 0;
    
    for (const auto& pose : poses.poses) {
        if (isInsideWorkspace(pose.position)) {
            workspace_poses.poses.push_back(pose);
            inside_workspace_count++;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Workspace filtering: %d/%d poses are inside workspace "
                "(x:[%.1f,%.1f], y:[%.1f,%.1f], z:[%.1f,%.1f])",
                inside_workspace_count, total_poses,
                workspace_x_min_, workspace_x_max_,
                workspace_y_min_, workspace_y_max_,
                workspace_z_min_, workspace_z_max_);
    
    return workspace_poses;
}
