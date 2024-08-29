#include <disparity_map_representation/disparity_map_representation.hpp>
#include <pluginlib/class_list_macros.hpp>

DisparityMapRepresentation::DisparityMapRepresentation()
    : MapRepresentation(), disp_graph(std::make_unique<disparity_graph::DisparityGraph>()) {
    points.ns = "obstacles";
    points.id = 0;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;

    // debug_pub = nh->advertise<visualization_msgs::MarkerArray>("disparity_map_debug", 1);
    debug_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("disparity_map_debug", 1);

    this->declare_parameter<int>("disparity_map/obstacle_check_points", 5);
    this->declare_parameter<double>("disparity_map/obstacle_check_radius", 2.0);

    this->get_parameter("disparity_map/obstacle_check_points", obstacle_check_points);
    this->get_parameter("disparity_map/obstacle_check_radius", obstacle_check_radius);

    // listener = new tf::TransformListener();
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

std::vector<std::vector<double> > DisparityMapRepresentation::get_values(
    std::vector<std::vector<geometry_msgs::msg::PointStamped> > trajectories) {
    std::vector<std::vector<double> > values(trajectories.size());

    for (int i = 0; i < trajectories.size(); i++) {
        for (int j = 0; j < trajectories[i].size(); j++) {
            values[i].push_back(0);
        }
    }

    for (int i = 0; i < trajectories.size(); i++) {
        // airstack_msgs::TrajectoryXYZVYaw trajectory = trajectories[i];
        for (int j = 0; j < trajectories[i].size(); j++) {
            tf2::Vector3 wp;
            tf2::fromMsg(trajectories[i][j].point, wp);

            // find the direction of the current trajectory segment between waypoints
            // this direction will be used to check for obstacles in a perpendicular direction
            tf2::Vector3 direction;
            tf2::Vector3 wp2 = wp;
            if (trajectories[i].size() < 2) {
                direction = tf2::Vector3(
                    1, 0,
                    0);  // TODO: make this point in the direction of the waypoints quaternion
            } else {
                if (j == 0) {
                    tf2::fromMsg(trajectories[i][1].point, wp2);
                    direction = wp2 - wp;
                } else {
                    tf2::fromMsg(trajectories[i][j - 1].point, wp);
                    direction = wp - wp2;
                }
            }

            points.header.frame_id = trajectories[i][j].header.frame_id;
            std_msgs::msg::ColorRGBA green;
            green.r = 0;
            green.b = 0;
            green.g = 1;
            green.a = 1;
            std_msgs::msg::ColorRGBA red;
            red.r = 1;
            red.b = 0;
            red.g = 0;
            red.a = 1;

            tf2::Quaternion q_up, q_down, q_left, q_right;
            q_up.setRPY(0, -M_PI / 2, 0);
            q_down.setRPY(0, M_PI / 2, 0);
            q_left.setRPY(0, 0, M_PI / 2);
            q_right.setRPY(0, 0, -M_PI / 2);
            std::vector<tf2::Quaternion> directions;
            directions.push_back(q_up);
            directions.push_back(q_down);
            directions.push_back(q_left);
            directions.push_back(q_right);

            tf2::Vector3 position = wp;
            tf2::Quaternion q = tf2::Quaternion(0, 0, 0,
                                                1);  // TODO: figure out if this makes sense
                                                     //
            tf2::Vector3 unit(1, 0, 0);
            double closest_obstacle_distance = obstacle_check_radius;

            direction.normalize();
            tf2::Vector3 up = tf2::Transform(q_up * q) * unit;
            up.normalize();

            tf2::Vector3 side = up.cross(direction);
            side.normalize();

            up = side.cross(direction);
            up.normalize();

            std::vector<tf2::Vector3> direction_vectors;
            direction_vectors.push_back(up);
            direction_vectors.push_back(side);
            direction_vectors.push_back(-up);
            direction_vectors.push_back(-side);

            for (int m = 0; m < directions.size(); m++) {
                tf2::Quaternion q_curr = directions[m];
                for (int k = 1; k < obstacle_check_points + 1; k++) {
                    double dist =
                        (double)k * obstacle_check_radius / (double)(obstacle_check_points + 1);

                    tf2::Vector3 point =
                        position +
                        dist * direction_vectors[m];  // tf::Transform(q_curr)*(dist*direction);
                    geometry_msgs::msg::PoseStamped check_pose;
                    check_pose.header = trajectories[i][j].header;
                    check_pose.pose.position = trajectories[i][j].point;
                    check_pose.pose.orientation.w = 1.0;

                    double occupancy;
                    bool collision =
                        !disp_graph->is_state_valid_depth_pose(check_pose, 0.9, occupancy);

                    std_msgs::msg::ColorRGBA c;
                    if (collision) {
                        closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
                        c = red;
                    } else
                        c = green;

                    points.points.push_back(check_pose.pose.position);
                    points.colors.push_back(c);
                }
            }

            points.points.push_back(trajectories[i][j].point);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = trajectories[i][j].header;  // trajectory.header;
            pose.pose.position = trajectories[i][j].point;
            pose.pose.orientation.w = 1.0;

            double occupancy;
            if (!disp_graph->is_state_valid_depth_pose(pose, 0.9, occupancy)) {
                values[i][j] = 0.f;
                points.colors.push_back(red);
            } else {
                points.colors.push_back(green);
            }

            values[i][j] = closest_obstacle_distance;
        }
    }

    return values;
}

double DisparityMapRepresentation::distance_to_obstacle(geometry_msgs::msg::PoseStamped pose,
                                                        tf2::Vector3 direction) {
    points.header.frame_id = pose.header.frame_id;
    std_msgs::msg::ColorRGBA green;
    green.r = 0;
    green.b = 0;
    green.g = 1;
    green.a = 1;
    std_msgs::msg::ColorRGBA red;
    red.r = 1;
    red.b = 0;
    red.g = 0;
    red.a = 1;

    tf2::Quaternion q_up, q_down, q_left, q_right;
    q_up.setRPY(0, -M_PI / 2, 0);
    q_down.setRPY(0, M_PI / 2, 0);
    q_left.setRPY(0, 0, M_PI / 2);
    q_right.setRPY(0, 0, -M_PI / 2);
    std::vector<tf2::Quaternion> directions;
    directions.push_back(q_up);
    directions.push_back(q_down);
    directions.push_back(q_left);
    directions.push_back(q_right);

    tf2::Vector3 position;
    tf2::fromMsg(pose.pose.position, position);
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    tf2::Vector3 unit(1, 0, 0);
    double closest_obstacle_distance = obstacle_check_radius;

    direction.normalize();
    tf2::Vector3 up = tf2::Transform(q_up * q) * unit;
    up.normalize();

    tf2::Vector3 side = up.cross(direction);
    side.normalize();

    up = side.cross(direction);
    up.normalize();

    std::vector<tf2::Vector3> direction_vectors;
    direction_vectors.push_back(up);
    direction_vectors.push_back(side);
    direction_vectors.push_back(-up);
    direction_vectors.push_back(-side);

    for (int m = 0; m < directions.size(); m++) {
        tf2::Quaternion q_curr = directions[m];
        for (int k = 1; k < obstacle_check_points + 1; k++) {
            double dist = (double)k * obstacle_check_radius / (double)(obstacle_check_points + 1);

            tf2::Vector3 point =
                position + dist * direction_vectors[m];  // tf::Transform(q_curr)*(dist*direction);
            geometry_msgs::msg::PoseStamped check_pose;
            check_pose.header = pose.header;
            check_pose.pose.position.x = point.x();
            check_pose.pose.position.y = point.y();
            check_pose.pose.position.z = point.z();
            check_pose.pose.orientation.w = 1.0;

            double occupancy;
            bool collision = !disp_graph->is_state_valid_depth_pose(check_pose, 0.9, occupancy);

            std_msgs::msg::ColorRGBA c;
            if (collision) {
                closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
                c = red;
            } else
                c = green;

            points.points.push_back(check_pose.pose.position);
            points.colors.push_back(c);
        }
    }

    points.points.push_back(pose.pose.position);
    points.colors.push_back(green);

    double occupancy;
    if (!disp_graph->is_state_valid_depth_pose(pose, 0.9, occupancy)) return 0.f;

    return closest_obstacle_distance;
}

void DisparityMapRepresentation::publish_debug() {
    points.header.stamp = this->now();
    markers.markers.push_back(points);

    debug_pub->publish(markers);

    markers.markers.clear();
    points.points.clear();
    points.colors.clear();
}

PLUGINLIB_EXPORT_CLASS(DisparityMapRepresentation, MapRepresentation)
