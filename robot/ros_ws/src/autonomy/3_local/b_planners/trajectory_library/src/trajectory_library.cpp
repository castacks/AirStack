#include <trajectory_library/trajectory_library.hpp>

Waypoint::Waypoint(double x, double y, double z, double yaw, double vx, double vy, double vz,
                   double ax, double ay, double az, double jx, double jy, double jz, double time) {
    x_ = x;
    y_ = y;
    z_ = z;
    vx_ = vx;
    vy_ = vy;
    vz_ = vz;
    ax_ = ax;
    ay_ = ay;
    az_ = az;
    jx_ = jx;
    jy_ = jy;
    jz_ = jz;
    yaw_ = yaw;
    time_ = time;
}

tf2::Quaternion Waypoint::quaternion() const {
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_);
    return quat;
}

tf2::Vector3 Waypoint::position() const { return tf2::Vector3(x_, y_, z_); }

tf2::Vector3 Waypoint::velocity() const { return tf2::Vector3(vx_, vy_, vz_); }

tf2::Vector3 Waypoint::acceleration() const { return tf2::Vector3(ax_, ay_, az_); }

tf2::Vector3 Waypoint::jerk() const { return tf2::Vector3(jx_, jy_, jz_); }

airstack_msgs::msg::Odometry Waypoint::as_odometry_msg(rclcpp::Time stamp,
                                                       std::string frame_id) const {
    airstack_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = frame_id;

    odom.pose.position.x = x_;
    odom.pose.position.y = y_;
    odom.pose.position.z = z_;

    tf2::Quaternion quat = quaternion();
    odom.pose.orientation.x = quat.x();
    odom.pose.orientation.y = quat.y();
    odom.pose.orientation.z = quat.z();
    odom.pose.orientation.w = quat.w();

    odom.twist.linear.x = vx_;
    odom.twist.linear.y = vy_;
    odom.twist.linear.z = vz_;

    odom.acceleration.x = ax_;
    odom.acceleration.y = ay_;
    odom.acceleration.z = az_;

    odom.jerk.x = jx_;
    odom.jerk.y = jy_;
    odom.jerk.z = jz_;

    return odom;
}

Waypoint Waypoint::interpolate(Waypoint wp, double t) {
    tf2::Vector3 pos_interp = position() + t * (wp.position() - position());
    tf2::Quaternion q_interp = quaternion().slerp(wp.quaternion(), t);
    tf2::Vector3 vel_interp = velocity() + t * (wp.velocity() - velocity());
    tf2::Vector3 accel_interp = acceleration() + t * (wp.acceleration() - acceleration());
    tf2::Vector3 jerk_interp = jerk() + t * (wp.jerk() - jerk());
    double time_interp = get_time() + t * (wp.get_time() - get_time());

    Waypoint wp_interp(pos_interp.x(), pos_interp.y(), pos_interp.z(), tf2::getYaw(q_interp),
                       vel_interp.x(), vel_interp.y(), vel_interp.z(), accel_interp.x(),
                       accel_interp.y(), accel_interp.z(), jerk_interp.x(), jerk_interp.y(),
                       jerk_interp.z(), time_interp);

    return wp_interp;
}

std::ostream& operator<<(std::ostream& os, const Waypoint& wp) {
    return os << "[pos: " << wp.get_x() << ", " << wp.get_y() << ", " << wp.get_z()
              << " vel: " << wp.get_vx() << ", " << wp.get_vy() << ", " << wp.get_vz()
              << " acc: " << wp.get_ax() << ", " << wp.get_ay() << ", " << wp.get_az()
              << " jerk: " << wp.get_jx() << ", " << wp.get_jy() << ", " << wp.get_jz()
              << " yaw: " << wp.get_yaw() << " time: " << wp.get_time();
}

Trajectory::Trajectory() { generated_waypoint_times = false; }

Trajectory::Trajectory(rclcpp::Node* node_ptr, std::string frame_id) : Trajectory() {
    if (buffer == NULL) {
        buffer = new tf2_ros::Buffer(node_ptr->get_clock());
        listener = new tf2_ros::TransformListener(*buffer);
    }

    this->frame_id = frame_id;
}

/**
 * @brief Construct a new Trajectory:: Trajectory object
 * Converts a nav_msgs::Path to a Trajectory object.
 * Only the position of each waypoint is used.
 * The velocity of each waypoint is set to 0.
 * TODO: add a time component to the trajectory?
 *
 * @param node_ptr
 * @param path
 */
Trajectory::Trajectory(rclcpp::Node* node_ptr, nav_msgs::msg::Path path)
    : Trajectory(node_ptr, path.header.frame_id) {
    this->stamp = path.header.stamp;

    // remove consecutive waypoints
    nav_msgs::msg::Path cleaned_path;
    cleaned_path.header = path.header;
    for (size_t i = 0; i < path.poses.size(); i++) {
        geometry_msgs::msg::PoseStamped pose = path.poses[i];
        if (i == 0)
            cleaned_path.poses.push_back(pose);
        else if (!(pose.pose.position.x == cleaned_path.poses.back().pose.position.x &&
                   pose.pose.position.y == cleaned_path.poses.back().pose.position.y &&
                   pose.pose.position.z == cleaned_path.poses.back().pose.position.z))
            cleaned_path.poses.push_back(pose);
    }

    // translate to Waypoint objects
    for (size_t i = 0; i < cleaned_path.poses.size(); i++) {
        geometry_msgs::msg::PoseStamped pose = cleaned_path.poses[i];

        Waypoint waypoint(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0);
        this->waypoints.push_back(waypoint);
    }
}

Trajectory::Trajectory(rclcpp::Node* node_ptr, airstack_msgs::msg::TrajectoryXYZVYaw path)
    : Trajectory(node_ptr, path.header.frame_id)

{
    this->stamp = path.header.stamp;

    // remove any consecutive duplicate waypoints
    airstack_msgs::msg::TrajectoryXYZVYaw cleaned_path;
    cleaned_path.header = path.header;
    for (size_t i = 0; i < path.waypoints.size(); i++) {
        airstack_msgs::msg::WaypointXYZVYaw wp = path.waypoints[i];
        if (i == 0)
            cleaned_path.waypoints.push_back(wp);
        else if (!(wp.position.x == cleaned_path.waypoints.back().position.x &&
                   wp.position.y == cleaned_path.waypoints.back().position.y &&
                   wp.position.z == cleaned_path.waypoints.back().position.z))
            cleaned_path.waypoints.push_back(wp);
    }

    // translate to Waypoint objects
    for (size_t i = 0; i < cleaned_path.waypoints.size(); i++) {
        airstack_msgs::msg::WaypointXYZVYaw wp = cleaned_path.waypoints[i];

        // calculate the x, y, z components of velocity
        tf2::Vector3 wp1(wp.position.x, wp.position.y, wp.position.z);
        tf2::Vector3 wp2(wp1);
        bool same = false;
        // for the first waypoint use the next waypoint to figure out direction of travel
        if (i == 0) {
            if (cleaned_path.waypoints.size() > 1)
                wp2 = tf2::Vector3(cleaned_path.waypoints[i + 1].position.x,
                                   cleaned_path.waypoints[i + 1].position.y,
                                   cleaned_path.waypoints[i + 1].position.z);
            else
                same = true;
        } else {  // otherwise use the previous waypoint
            wp1 = tf2::Vector3(cleaned_path.waypoints[i - 1].position.x,
                               cleaned_path.waypoints[i - 1].position.y,
                               cleaned_path.waypoints[i - 1].position.z);
        }

        tf2::Vector3 direction = (wp2 - wp1).normalized();
        // ROS_INFO_STREAM("direction: " << direction.x() << ", " << direction.y() << ", " <<
        // direction.z());
        tf2::Vector3 vel = wp.velocity * direction;
        // if i == 0 and cleaned_path.waypoints is empty, wp1 will equal wp2 which will make vel nan
        if (same) vel = tf2::Vector3(0, 0, 0);

        // ROS_INFO_STREAM("WAYPOINT " << i << " vel: " << vel.x() << " " << vel.y() << " " <<
        // vel.z()); ROS_INFO_STREAM("\twp1: " << wp1.x() << " " << wp1.y() << " " << wp1.z());
        // ROS_INFO_STREAM("\twp2: " << wp2.x() << " " << wp2.y() << " " << wp2.z());
        // ROS_INFO_STREAM("\tdirection: " << direction.x() << " " << direction.y() << " " <<
        // direction.z() << " wp.vel: " << wp.vel);

        Waypoint waypoint(wp.position.x, wp.position.y, wp.position.z, wp.yaw, vel.x(), vel.y(),
                          vel.z(), wp.acceleration.x, wp.acceleration.y, wp.acceleration.z,
                          wp.jerk.x, wp.jerk.y, wp.jerk.z);
        this->waypoints.push_back(waypoint);
    }
}
/*
void Trajectory::init_listener(){
  if(listener == NULL)
    listener = new tf2_ros::Buffer();
}
*/
void Trajectory::clear() {
    waypoints.clear();
    generated_waypoint_times = false;
}

/**
 * @brief Uses each waypoint's position and velocity to generate the expected time to reach each
 * waypoint.
 *
 */
void Trajectory::generate_waypoint_times() {
    if (generated_waypoint_times) return;

    if (waypoints.size() > 1) {
        waypoints[0].set_time(0.);
        Waypoint& curr_wp = waypoints[0];
    }

    for (size_t i = 1; i < waypoints.size(); i++) {
        Waypoint& curr_wp = waypoints[i];
        Waypoint& prev_wp = waypoints[i - 1];

        // ROS_INFO_STREAM(i << " wp vel: " << curr_wp.velocity().x() << ", " <<
        // curr_wp.velocity().y() << ", " << curr_wp.velocity().z()
        //	    << " | " << prev_wp.velocity().x() << ", " << prev_wp.velocity().y() << ", " <<
        // prev_wp.velocity().z());

        double distance = curr_wp.position().distance(prev_wp.position());
        if (distance == 0) {
            waypoints.erase(waypoints.begin() + i);
            i--;
            continue;
        }
        double velocity =
            std::max(0.01, (curr_wp.velocity().length() + prev_wp.velocity().length()) / 2.);
        // if(i+1 < waypoints.size()){
        //   velocity = (velocity + waypoints[i+1].velocity().length())/2.0;
        // }

        // ROS_INFO_STREAM("WAYPOINT " << i << " dist: " << distance << " vel: " << velocity << "
        // time: " << (prev_wp.time() + distance/velocity) << " prev_time: " << prev_wp.time() << "
        // inc: " << (distance/velocity));

        curr_wp.set_time(prev_wp.get_time() + distance / velocity);
    }

    generated_waypoint_times = true;
}

std::tuple<bool, Waypoint, size_t, double> Trajectory::get_closest_point(tf2::Vector3 point) {
    double closest_distance = std::numeric_limits<double>::max();
    double best_t = 0;
    size_t best_wp_index = 0;

    Waypoint closest(0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    double path_distance = 0;

    if (waypoints.size() == 0) {
        return std::make_tuple(false, closest, best_wp_index, path_distance);
    } else if (waypoints.size() == 1) {
        closest = waypoints[0];
    } else {
        for (size_t i = 1; i < waypoints.size(); i++) {
            // parameteric representation of segment between waypoints: segment_start +
            // t*segment_vec
            tf2::Vector3 segment_start = waypoints[i - 1].position();
            tf2::Vector3 segment_end = waypoints[i].position();
            tf2::Vector3 segment_vec = segment_end - segment_start;

            // project the vector (point - segment_start) onto the parametric line
            double t = ((point - segment_start).dot(segment_vec)) / (segment_vec.dot(segment_vec));
            t = std::max(0.0, std::min(1.0, t));
            // tf2::Vector3 closest_point = segment_start + t*segment_vec;
            Waypoint closest_waypoint = waypoints[i - 1].interpolate(waypoints[i], t);

            // find the distance between the point and the closest point on the current segment
            double distance = closest_waypoint.position().distance(point);
            if (distance < closest_distance) {
                closest_distance = distance;
                closest = closest_waypoint;  // closest_point;
                best_wp_index = i - 1;
                best_t = t;
            }
        }

        for (size_t i = 0; i <= best_wp_index; i++) {
            tf2::Vector3 segment_start = waypoints[i].position();
            tf2::Vector3 segment_end = waypoints[i + 1].position();
            tf2::Vector3 segment_vec = segment_end - segment_start;

            if (i == best_wp_index)
                path_distance += best_t * segment_vec.length();
            else
                path_distance += segment_vec.length();
        }
    }

    return std::make_tuple(true, closest, best_wp_index, path_distance);
}

bool Trajectory::get_trajectory_distance_at_closest_point(tf2::Vector3 point,
                                                          double* trajectory_distance) {
    double closest_distance = std::numeric_limits<double>::max();
    *trajectory_distance = 0;
    double current_trajectory_distance = 0;

    if (waypoints.size() == 0)
        return false;
    else if (waypoints.size() == 1)
        return true;
    else {
        for (size_t i = 1; i < waypoints.size(); i++) {
            // parameteric representation of segment between waypoints: segment_start +
            // t*segment_vec
            tf2::Vector3 segment_start = waypoints[i - 1].position();
            tf2::Vector3 segment_end = waypoints[i].position();
            tf2::Vector3 segment_vec = segment_end - segment_start;
            double segment_length = segment_start.distance(segment_end);

            // project the vector (point - segment_start) onto the parametric line
            double t = ((point - segment_start).dot(segment_vec)) / (segment_vec.dot(segment_vec));
            t = std::max(0.0, std::min(1.0, t));
            tf2::Vector3 closest_point = segment_start + t * segment_vec;

            // find the distance between the point and the closest point on the current segment
            double distance = closest_point.distance(point);
            if (distance < closest_distance) {
                closest_distance = distance;
                *trajectory_distance = current_trajectory_distance + t * segment_length;
            }

            current_trajectory_distance += segment_length;
        }
    }

    return true;
}

bool Trajectory::merge(Trajectory traj, double min_time) {
    if (traj.waypoints.size() == 0) {
        return true;
    }
    generated_waypoint_times = false;

    Trajectory transformed_traj;
    try {
        transformed_traj = traj.to_frame(frame_id, this->stamp);
    } catch (tf2::TransformException& ex) {
        std::cout << "Transform exception while merging trajectories: " << ex.what();
    }
    if (transformed_traj.waypoints.size() == 0) {
        return true;
    }

    if (waypoints.size() == 0) {
        waypoints.insert(waypoints.end(), transformed_traj.waypoints.begin(),
                         transformed_traj.waypoints.end());
        return true;
    }

    // tf2::Vector3 closest_point;
    auto [valid, closest_waypoint, wp_index, path_distance] =
        get_closest_point(transformed_traj.waypoints[0].position());

    if (closest_waypoint.get_time() >= min_time) {
        waypoints.erase(waypoints.begin() + wp_index + 1, waypoints.end());
        waypoints.insert(waypoints.end(), transformed_traj.waypoints.begin(),
                         transformed_traj.waypoints.end());
        if (wp_index == 0) {
            // ROS_WARN("FIRST TIME GENERATION");
            generate_waypoint_times();
        }
        return true;
    }

    std::cout << "COULDN'T MERGE BECAUSE OF min_time: " << closest_waypoint.get_time() << " "
              << min_time;
    return false;
}

bool Trajectory::get_closest_waypoint(tf2::Vector3 point, double initial_time, double end_time,
                                      Waypoint* waypoint) {
    double closest_distance = std::numeric_limits<double>::max();
    bool found = false;

    for (size_t i = 1; i < waypoints.size(); i++) {
        if (waypoints[i].get_time() < initial_time) continue;
        if (waypoints[i - 1].get_time() > end_time) break;

        Waypoint wp_start = waypoints[i - 1];
        Waypoint wp_end = waypoints[i];

        // handle the case that the initial_time is between waypoint i-1 and waypoint i
        if (wp_start.get_time() < initial_time)
            wp_start = wp_start.interpolate(wp_end, (initial_time - wp_start.get_time()) /
                                                        (wp_end.get_time() - wp_start.get_time()));
        // handle the case that the end_time is between waypoint i-1 and waypoint i
        if (wp_end.get_time() > end_time)
            wp_end = wp_start.interpolate(wp_end, (end_time - wp_start.get_time()) /
                                                      (wp_end.get_time() - wp_start.get_time()));

        // parameteric representation of segment between waypoints: segment_start + t*segment_vec
        tf2::Vector3 segment_start = wp_start.position();
        tf2::Vector3 segment_end = wp_end.position();
        tf2::Vector3 segment_vec = segment_end - segment_start;

        // project the vector (point - segment_start) onto the parametric line
        double t = ((point - segment_start).dot(segment_vec)) / (segment_vec.dot(segment_vec));
        t = std::max(0.0, std::min(1.0, t));
        tf2::Vector3 closest_point = segment_start + t * segment_vec;

        // find the distance between the point and the closest point on the current segment
        double distance = closest_point.distance(point);
        if (distance < closest_distance) {
            closest_distance = distance;
            found = true;
            *waypoint = wp_start.interpolate(wp_end, t);
        }
    }

    return found;
}

bool Trajectory::get_waypoint_distance_ahead(double initial_time, double distance,
                                             Waypoint* waypoint) {
    double current_distance = 0;

    for (size_t i = 1; i < waypoints.size(); i++) {
        if (waypoints[i].get_time() < initial_time) continue;

        Waypoint wp_start = waypoints[i - 1];
        Waypoint wp_end = waypoints[i];

        // handle the case that the initial_time is between waypoint i-1 and waypoint i
        if (wp_start.get_time() < initial_time)
            wp_start = wp_start.interpolate(wp_end, (initial_time - wp_start.get_time()) /
                                                        (wp_end.get_time() - wp_start.get_time()));

        double segment_distance = wp_start.position().distance(wp_end.position());
        if (current_distance + segment_distance >= distance) {
            *waypoint =
                wp_start.interpolate(wp_end, (distance - current_distance) / segment_distance);
            return true;
        } else
            current_distance += segment_distance;

        if (i == waypoints.size() - 1 && wp_end.get_time() >= initial_time) {
            *waypoint = wp_end;
            return true;
        }
    }

    return false;
}

bool Trajectory::get_waypoint_sphere_intersection(double initial_time, double ahead_distance,
                                                  double time_end, tf2::Vector3 sphere_center,
                                                  double sphere_radius, double min_velocity,
                                                  Waypoint* waypoint, Waypoint* end_waypoint) {
    double current_path_distance = 0;
    bool found = false;

    // ROS_INFO("\n\n");
    for (size_t i = 1; i < waypoints.size(); i++) {
        if (waypoints[i].get_time() < initial_time) continue;

        Waypoint wp_start = waypoints[i - 1];
        Waypoint wp_end = waypoints[i];

        // handle the case that the initial_time is between waypoint i-1 and waypoint i
        if (wp_start.get_time() < initial_time)
            wp_start = wp_start.interpolate(wp_end, (initial_time - wp_start.get_time()) /
                                                        (wp_end.get_time() - wp_start.get_time()));

        double segment_distance = wp_start.position().distance(wp_end.position());
        bool should_break = false;
        if (current_path_distance + segment_distance >= ahead_distance) {
            should_break = true;
            wp_end = wp_start.interpolate(
                wp_end, (ahead_distance - current_path_distance) / segment_distance);
            if (end_waypoint != NULL) *end_waypoint = wp_end;
        } else if (wp_end.get_time() > time_end) {
            should_break = true;
            wp_end = wp_start.interpolate(wp_end, (time_end - wp_start.get_time()) /
                                                      (wp_end.get_time() - wp_start.get_time()));
            if (end_waypoint != NULL) *end_waypoint = wp_end;
        } else
            current_path_distance += segment_distance;

        // sphere line intersection equations:
        // http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
        double x1 = wp_start.get_x();
        double y1 = wp_start.get_y();
        double z1 = wp_start.get_z();
        double x2 = wp_end.get_x();
        double y2 = wp_end.get_y();
        double z2 = wp_end.get_z();
        double xc = sphere_center.x();
        double yc = sphere_center.y();
        double zc = sphere_center.z();
        double a = pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2);
        double b = -2 * ((x2 - x1) * (xc - x1) + (y2 - y1) * (yc - y1) + (z2 - z1) * (zc - z1));
        double c = pow(xc - x1, 2) + pow(yc - y1, 2) + pow(zc - z1, 2) - pow(sphere_radius, 2);

        bool is_intersection = (b * b - 4 * a * c) >= 0;

        /*
        ROS_INFO_STREAM("xyz1: " << x1 << ", " << y1 << ", " << z1 << " xyz2: " << x2 << ", " << y2
        << ", " << z2
                        << " c: " << xc << ", " << yc << ", " << zc << " abc: " << a << ", " << b <<
        ", " << c
                        << "rad: " << sphere_radius << " int: " << is_intersection);
        //*/
        if (is_intersection) {
            double t1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            double t2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            // ROS_INFO_STREAM("t: " << t1 << ", " << t2);

            double t = 0.;
            bool t1_in_range = (t1 >= 0.0) && (t1 <= 1.0);
            bool t2_in_range = (t2 >= 0.0) && (t2 <= 1.0);
            if (t1_in_range || t2_in_range) {
                if (t1_in_range && t2_in_range)
                    t = std::max(t1, t2);
                else if (t1_in_range)
                    t = t1;
                else if (t2_in_range)
                    t = t2;

                if (waypoint != NULL) {
                    // ROS_INFO_STREAM("t: " << t);
                    *waypoint = wp_start.interpolate(wp_end, t);
                    found = true;
                }
            }
        }

        if (should_break) break;
    }
    // if(!found)
    //   ROS_INFO("NOT FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    return found;
}

bool Trajectory::get_waypoint(double time, Waypoint* waypoint) {
    generate_waypoint_times();

    for (size_t i = 1; i < waypoints.size(); i++) {
        if (waypoints[i].get_time() < time) continue;

        Waypoint wp_start = waypoints[i - 1];
        Waypoint wp_end = waypoints[i];

        *waypoint = wp_start.interpolate(
            wp_end, (time - wp_start.get_time()) / (wp_end.get_time() - wp_start.get_time()));
        return true;
    }

    if (waypoints.size() > 0) {
        *waypoint = waypoints.back();
        return true;
    }

    return false;
}

double Trajectory::get_duration() {
    generate_waypoint_times();

    if (waypoints.size() == 0) return 0;

    return waypoints.back().get_time();
}

/**
 * @brief Get the expected odometry at a given time. Performs interpolation if the time is between
 * waypoints.
 *
 * @param time
 * @param odom
 * @param stamp
 * @return true
 * @return false
 */
bool Trajectory::get_odom(double time, airstack_msgs::msg::Odometry* odom, rclcpp::Time stamp) {
    generate_waypoint_times();

    if (waypoints.size() == 0) {
        return false;
    } else if (waypoints.size() == 1) {
        *odom = waypoints[0].as_odometry_msg(stamp, frame_id);  // rclcpp::Time::now(), frame_id);
        return true;
    }

    // figure out what waypoints we are between
    Waypoint prev_wp = waypoints.front();
    Waypoint curr_wp = waypoints.front();
    for (size_t i = waypoints.size() - 1; i >= 1; i--) {
        if (time >= waypoints[i - 1].get_time()) {
            prev_wp = waypoints[i - 1];
            curr_wp = waypoints[i];
            break;
        }
    }

    // figure out where we are in between the current and previous waypoints
    double t = (time - prev_wp.get_time()) / (curr_wp.get_time() - prev_wp.get_time());
    t = std::max(0.0, std::min(1.0, t));  // TODO check for nan
    // ROS_INFO_STREAM("t: " << t);

    *odom = prev_wp.interpolate(curr_wp, t)
                .as_odometry_msg(stamp, frame_id);  // rclcpp::Time::now(), frame_id);
    // ROS_INFO_STREAM("odom: " << odom->pose.position.x << ", " << odom->pose.position.y << ", " <<
    // odom->pose.position.z << " | "
    //		  << odom->twist.linear.x << ", " << odom->twist.linear.y << ", " <<
    // odom->twist.linear.z);

    return true;
}

Trajectory Trajectory::to_frame(std::string target_frame, rclcpp::Time time) {
    // ROS_INFO("to frame");
    // init_listener();

    tf2::Stamped<tf2::Transform> transform;
    geometry_msgs::msg::TransformStamped t;
    t = buffer->lookupTransform(target_frame, frame_id, time, rclcpp::Duration::from_seconds(0.1));
    tf2::fromMsg(t, transform);
    // buffer->waitForTransform(target_frame, frame_id, time, rclcpp::Duration(0.1));
    // buffer->lookupTransform(target_frame, frame_id, time, transform);
    tf2::Transform rot(transform.getRotation());

    Trajectory transformed_traj;
    transformed_traj.stamp = time;
    transformed_traj.frame_id = target_frame;

    for (size_t i = 0; i < waypoints.size(); i++) {
        Waypoint wp = waypoints[i];

        tf2::Vector3 transformed_position = transform * wp.position();
        tf2::Vector3 transformed_velocity = rot * wp.velocity();
        tf2::Vector3 transformed_acceleration = rot * wp.acceleration();
        tf2::Vector3 transformed_jerk = rot * wp.jerk();
        tf2::Quaternion transformed_q = transform * wp.quaternion();

        // ROS_INFO_STREAM("jerk: " << wp.jerk().x() << ", " << wp.jerk().y() << ", " <<
        // wp.jerk().z() << " | "
        //		    << transformed_jerk.x() << ", " << transformed_jerk.y() << ", " <<
        // transformed_jerk.z());

        Waypoint transformed_wp(transformed_position.x(), transformed_position.y(),
                                transformed_position.z(), tf2::getYaw(transformed_q),
                                transformed_velocity.x(), transformed_velocity.y(),
                                transformed_velocity.z(), transformed_acceleration.x(),
                                transformed_acceleration.y(), transformed_acceleration.z(),
                                transformed_jerk.x(), transformed_jerk.y(), transformed_jerk.z());

        transformed_traj.waypoints.push_back(transformed_wp);
    }

    return transformed_traj;
}

/*Trajectory Trajectory::respace(double spacing){
  Trajectory traj;
  traj.stamp = stamp;
  traj.frame_id = frame_id;
  //ROS_INFO("RESPACE");

  if(waypoints.size() > 0){
    double distance = 0;
    traj.waypoints.push_back(waypoints[0]); // always add the first waypoint

    for(size_t i = 1; i < waypoints.size(); i++){
      if(i == waypoints.size()-1){ // always add the last waypoint
        traj.waypoints.push_back(waypoints[i]);
        continue;
      }
      //ROS_INFO_STREAM("Waypoint " << i);
      Waypoint prev_wp = waypoints[i-1];
      Waypoint curr_wp = waypoints[i];
      double segment_length = prev_wp.position().distance(curr_wp.position());
      //ROS_INFO_STREAM("segment_length: " << segment_length << " distance: " << distance << "
spacing: " << spacing);

      if(distance + segment_length >= spacing){
        //ROS_INFO("interpolating");
        Waypoint interp_wp = prev_wp.interpolate(curr_wp, (spacing - distance)/segment_length);
        traj.waypoints.push_back(interp_wp);
        distance = 0;
      }
      else
        distance += segment_length;
    }
  }

  return traj;
}

Trajectory Trajectory::shorten(double new_length){
  Trajectory traj;
  traj.stamp = stamp;
  traj.frame_id = frame_id;

  if(waypoints.size() > 0){
    double distance = 0;
    traj.waypoints.push_back(waypoints[0]); // always add the first waypoint

    for(size_t i = 1; i < waypoints.size(); i++){
      Waypoint prev_wp = waypoints[i-1];
      Waypoint curr_wp = waypoints[i];
      double segment_length = prev_wp.position().distance(curr_wp.position());

      if(distance + segment_length >= new_length){
        Waypoint interp_wp = prev_wp.interpolate(curr_wp, (new_length - distance)/segment_length);
        traj.waypoints.push_back(interp_wp);
        break;
      }
      else{
        traj.waypoints.push_back(curr_wp);
        distance += segment_length;
      }
    }
  }

  return traj;
}*/

/**
 * @brief Trim a subtrajectory between a start distance and end distance
 *
 * @param start
 * @param end
 * @return Trajectory
 */
Trajectory Trajectory::trim_trajectory_between_distances(double start_dist, double end_dist) {
    Trajectory traj;
    traj.stamp = this->stamp;
    traj.frame_id = this->frame_id;

    if (this->waypoints.size() > 0) {
        double distance = 0;

        if (start_dist == 0. && this->waypoints.size() == 1)
            traj.waypoints.push_back(this->waypoints[0]);

        for (size_t i = 1; i < this->waypoints.size(); i++) {
            Waypoint prev_wp = this->waypoints[i - 1];
            Waypoint curr_wp = this->waypoints[i];
            double segment_length = prev_wp.position().distance(curr_wp.position());

            if (start_dist >= distance && start_dist <= distance + segment_length) {
                Waypoint interp_start_wp =
                    prev_wp.interpolate(curr_wp, (start_dist - distance) / segment_length);
                traj.waypoints.push_back(interp_start_wp);
            }

            if (distance + segment_length > start_dist && distance + segment_length < end_dist) {
                traj.waypoints.push_back(curr_wp);
            }

            if (end_dist >= distance && end_dist <= distance + segment_length) {
                Waypoint interp_end_wp =
                    prev_wp.interpolate(curr_wp, (end_dist - distance) / segment_length);
                traj.waypoints.push_back(interp_end_wp);
            }

            distance += segment_length;
        }
    }

    return traj;
}

Trajectory Trajectory::get_reversed_trajectory() {
    generated_waypoint_times = false;
    generate_waypoint_times();
    Trajectory traj;
    traj.frame_id = this->frame_id;
    traj.stamp = this->stamp;
    traj.waypoints.assign(waypoints.begin(), waypoints.end());
    std::reverse(traj.waypoints.begin(), traj.waypoints.end());
    for (size_t i = 0; i < traj.waypoints.size(); i++) {
        Waypoint& wp = traj.waypoints[i];
        wp.vx_ *= -1;
        wp.vy_ *= -1;
        wp.vz_ *= -1;
        /*
        wp.ax_ *= -1;
        wp.ay_ *= -1;
        wp.az_ *= -1;
        wp.jx_ *= -1;
        wp.jy_ *= -1;
        wp.jz_ *= -1;
        */
    }
    traj.generate_waypoint_times();
    return traj;
}

float Trajectory::get_skip_ahead_time(float start_time, float max_velocity, float max_distance) {
    generate_waypoint_times();
    double current_path_distance = 0;
    float skip_time = start_time;

    for (size_t i = 1; i < waypoints.size(); i++) {
        if (waypoints[i].get_time() < start_time) continue;

        Waypoint wp_start = waypoints[i - 1];
        float wp_start_velocity = wp_start.velocity().length();
        if (wp_start_velocity > max_velocity) break;
        Waypoint wp_end = waypoints[i];
        float wp_end_velocity = wp_end.velocity().length();

        if (wp_start.get_time() < start_time)
            wp_start = wp_start.interpolate(wp_end, (start_time - wp_start.get_time()) /
                                                        (wp_end.get_time() - wp_start.get_time()));

        double segment_distance = wp_start.position().distance(wp_end.position());
        bool should_break = false;
        if (current_path_distance + segment_distance >= max_distance) {
            should_break = true;
            wp_end = wp_start.interpolate(
                wp_end, (max_distance - current_path_distance) / segment_distance);
        }

        if (wp_end_velocity > max_velocity) {
            should_break = true;
            wp_end = wp_start.interpolate(
                wp_end, std::min(1.f, std::max(0.f, (max_velocity - wp_start_velocity) /
                                                        (wp_end_velocity - wp_start_velocity))));
        }

        skip_time = wp_end.get_time();

        if (should_break) break;

        current_path_distance += segment_distance;
    }

    return skip_time;
}

void Trajectory::set_fixed_height(double height) {
    for (size_t i = 0; i < waypoints.size(); i++) waypoints[i].z_ = height;
}

size_t Trajectory::get_num_waypoints() const { return waypoints.size(); }

const Waypoint& Trajectory::get_waypoint(int index) const { return waypoints[index]; }

const std::vector<Waypoint>& Trajectory::get_waypoints() const { return waypoints; }

const std::string& Trajectory::get_frame_id() const { return frame_id; }

airstack_msgs::msg::TrajectoryXYZVYaw Trajectory::get_TrajectoryXYZVYaw_msg() {
    airstack_msgs::msg::TrajectoryXYZVYaw path;
    path.header.stamp = this->stamp;
    path.header.frame_id = this->frame_id;

    for (size_t i = 0; i < waypoints.size(); i++) {
        Waypoint wp = waypoints[i];

        airstack_msgs::msg::WaypointXYZVYaw w;
        w.position.x = wp.get_x();
        w.position.y = wp.get_y();
        w.position.z = wp.get_z();
        w.yaw = tf2::getYaw(wp.quaternion());
        w.velocity = wp.velocity().length();

        path.waypoints.push_back(w);
    }

    return path;
}

std::vector<geometry_msgs::msg::PointStamped> Trajectory::get_vector_PointStamped_msg() {
    std::vector<geometry_msgs::msg::PointStamped> points;

    for (size_t i = 0; i < waypoints.size(); i++) {
        Waypoint wp = waypoints[i];

        geometry_msgs::msg::PointStamped point;
        point.header.stamp = this->stamp;
        point.header.frame_id = this->frame_id;
        point.point.x = wp.get_x();
        point.point.y = wp.get_y();
        point.point.z = wp.get_z();

        points.push_back(point);
    }

    return points;
}

visualization_msgs::msg::MarkerArray Trajectory::get_markers(rclcpp::Time stamp,
                                                             const std::string& marker_namespace,
                                                             float r, float g, float b, float a,
                                                             bool show_poses, bool show_velocity,
                                                             float thickness) {
    visualization_msgs::msg::MarkerArray marker_array;
    // rclcpp::Time now = rclcpp::Time::now();

    visualization_msgs::msg::Marker clear;
    clear.ns = marker_namespace;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    visualization_msgs::msg::Marker lines;
    lines.header.stamp = stamp;
    lines.header.frame_id = frame_id;
    lines.ns = marker_namespace;
    lines.id = waypoints.size() + 1;
    lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = thickness;

    for (size_t i = 0; i < waypoints.size(); i++) {
        Waypoint wp = waypoints[i];

        if (show_poses) {
            visualization_msgs::msg::Marker arrow;
            arrow.header.stamp = stamp;
            arrow.header.frame_id = frame_id;
            arrow.ns = marker_namespace;
            arrow.id = i;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            arrow.pose.position.x = wp.position().x();
            arrow.pose.position.y = wp.position().y();
            arrow.pose.position.z = wp.position().z();
            arrow.pose.orientation.x = wp.quaternion().x();
            arrow.pose.orientation.y = wp.quaternion().y();
            arrow.pose.orientation.z = wp.quaternion().z();
            arrow.pose.orientation.w = wp.quaternion().w();
            arrow.scale.x = 0.2;   // length
            arrow.scale.y = 0.05;  // width
            arrow.scale.z = 0.05;  // height
            arrow.color.r = r;
            arrow.color.g = g;
            arrow.color.b = b;
            arrow.color.a = a;

            marker_array.markers.push_back(arrow);
        }

        if (show_velocity) {
            visualization_msgs::msg::Marker vel_arrow;
            vel_arrow.header.stamp = stamp;
            vel_arrow.header.frame_id = frame_id;
            vel_arrow.ns = marker_namespace + "_velocities";
            vel_arrow.id = i;
            vel_arrow.type = visualization_msgs::msg::Marker::ARROW;
            vel_arrow.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point point1, point2;
            point1.x = wp.position().x();
            point1.y = wp.position().y();
            point1.z = wp.position().z() + 0.2;
            point2.x = wp.position().x() + wp.velocity().normalized().x() / 3.;
            point2.y = wp.position().y() + wp.velocity().normalized().y() / 3.;
            point2.z = wp.position().z() + 0.2 + wp.velocity().normalized().z() / 3.;
            vel_arrow.points.push_back(point1);
            vel_arrow.points.push_back(point2);
            vel_arrow.scale.x = 0.1;
            vel_arrow.scale.y = 0.15;
            vel_arrow.scale.z = 0.1;
            vel_arrow.color.r = r;
            vel_arrow.color.g = g;
            vel_arrow.color.b = b;
            vel_arrow.color.a = a;

            marker_array.markers.push_back(vel_arrow);
        }

        geometry_msgs::msg::Point p;
        p.x = wp.position().x();
        p.y = wp.position().y();
        p.z = wp.position().z();
        lines.points.push_back(p);
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        lines.colors.push_back(color);
    }

    marker_array.markers.push_back(lines);

    return marker_array;
}

//===================================================================================
//--------------------------------- Dynamic Trajectories ----------------------------
//===================================================================================

TakeoffTrajectory::TakeoffTrajectory(double height, double velocity, double path_roll,
                                     double path_pitch, bool relative_to_orientation) {
    this->height = height;
    this->velocity = velocity;
    this->path_roll = path_roll;
    this->path_pitch = path_pitch;
    this->relative_to_orientation = relative_to_orientation;
}

airstack_msgs::msg::TrajectoryXYZVYaw TakeoffTrajectory::get_trajectory(
    airstack_msgs::msg::Odometry odom) {
    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    traj.header.frame_id = odom.header.frame_id;
    traj.header.stamp = odom.header.stamp;

    airstack_msgs::msg::WaypointXYZVYaw wp1, wp2, wp3;

    wp1.position.x = odom.pose.position.x;
    wp1.position.y = odom.pose.position.y;
    wp1.position.z = odom.pose.position.z;

    tf2::Quaternion q(odom.pose.orientation.x, odom.pose.orientation.y, odom.pose.orientation.z,
                      odom.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    wp1.yaw = yaw;  // tf2::getYaw(q);
    wp1.velocity = velocity;

    wp2.position.x =
        odom.pose.position.x + height * sin(path_pitch + (relative_to_orientation ? pitch : 0));
    wp2.position.y =
        odom.pose.position.y + height * sin(path_roll - (relative_to_orientation ? roll : 0));
    wp2.position.z = odom.pose.position.z + height - 0.01;
    wp2.yaw = wp1.yaw;
    wp2.velocity = velocity;

    wp3.position.x = wp2.position.x;
    wp3.position.y = wp2.position.y;
    wp3.position.z = wp2.position.z + 0.01;
    wp3.yaw = wp2.yaw;
    wp3.velocity = 0.01;

    traj.waypoints.push_back(wp1);
    traj.waypoints.push_back(wp2);
    traj.waypoints.push_back(wp3);

    return traj;
}

AccelerationTrajectory::AccelerationTrajectory(tf2_ros::Buffer* buffer, std::string frame,
                                               double ax, double ay, double az, double dt,
                                               double ht, double max_velocity) {
    // this->buffer = buffer;

    // ros::NodeHandle nh("acceleration_trajectory");
    // set_max_velocity_sub = nh.subscribe("set_max_velocity", 1,
    // &AccelerationTrajectory::set_max_velocity_callback, this);

    this->ax = ax;
    this->ay = ay;
    this->az = az;
    this->dt = dt;
    this->ht = ht;
    this->max_velocity = max_velocity;
    this->frame = frame;
}

// void AccelerationTrajectory::set_max_velocity_callback(std_msgs::Float32 msg){
//   max_velocity = msg.data;
// }

airstack_msgs::msg::TrajectoryXYZVYaw AccelerationTrajectory::get_trajectory(
    airstack_msgs::msg::Odometry odom) {
    airstack_msgs::msg::TrajectoryXYZVYaw traj;
    traj.header.frame_id = frame;
    traj.header.stamp = odom.header.stamp;

    try {
        tf2::Stamped<tf2::Transform> pose_tf;
        geometry_msgs::msg::TransformStamped t;
        t = buffer->lookupTransform(frame, odom.header.frame_id, rclcpp::Time(odom.header.stamp),
                                    rclcpp::Duration::from_seconds(0.1));
        tf2::fromMsg(t, pose_tf);
        // buffer->waitForTransform(frame, odom.header.frame_id, odom.header.stamp,
        // rclcpp::Duration(0.1)); buffer->lookupTransform(frame, odom.header.frame_id,
        // odom.header.stamp, pose_tf);
        tf2::Stamped<tf2::Transform> velocity_tf;
        t = buffer->lookupTransform(frame, odom.child_frame_id, rclcpp::Time(odom.header.stamp),
                                    rclcpp::Duration::from_seconds(0.1));
        tf2::fromMsg(t, velocity_tf);
        // buffer->waitForTransform(frame, odom.child_frame_id, odom.header.stamp,
        // rclcpp::Duration(0.1)); buffer->lookupTransform(frame, odom.child_frame_id,
        // odom.header.stamp, velocity_tf);
        velocity_tf.setOrigin(
            tf2::Vector3(0, 0, 0));  // only use rotation to transform the velocity

        tf2::Vector3 position = pose_tf * tflib::to_tf(odom.pose.position);
        tf2::Quaternion orientation = pose_tf * tflib::to_tf(odom.pose.orientation);
        tf2::Vector3 velocity = velocity_tf * tflib::to_tf(odom.twist.linear);

        tf2::Vector3 acceleration(ax, ay, az);

        airstack_msgs::msg::WaypointXYZVYaw wp0;
        wp0.position.x = position.x();
        wp0.position.y = position.y();
        wp0.position.z = position.z();
        wp0.yaw = 0;
        wp0.velocity = velocity.length();
        traj.waypoints.push_back(wp0);

        for (double t = 0; t < ht; t += dt) {
            velocity += acceleration * dt;
            position += velocity * dt;

            airstack_msgs::msg::WaypointXYZVYaw wp;
            wp.position.x = position.x();
            wp.position.y = position.y();
            wp.position.z = position.z();
            wp.yaw = 0;
            wp.velocity = std::min(velocity.length(), max_velocity);
            traj.waypoints.push_back(wp);
        }
    } catch (tf2::TransformException& e) {
        std::cout << "TransformException in AccelerationTrajectory::get_trajectory: " << e.what();
    }

    return traj;
}

//===================================================================================
//--------------------------------- Static Trajectories -----------------------------
//===================================================================================

CurveTrajectory::CurveTrajectory(float linear_velocity, float angular_velocity, std::string frame,
                                 float time, float dt, bool use_heading, float yaw) {
    this->linear_velocity = linear_velocity;
    this->angular_velocity = angular_velocity;
    this->frame = frame;
    this->time = time;
    this->dt = dt;
    this->use_heading = use_heading;
    this->yaw = yaw;

    generate_trajectory();
}

void CurveTrajectory::generate_trajectory() {
    float prev_x = 0.f;
    float prev_y = 0.f;
    float prev_heading = 0.f;

    // TODO: when to fill in trajectory.header.stamp?
    trajectory.header.frame_id = frame;

    for (float t = 0; t <= time; t += dt) {
        airstack_msgs::msg::WaypointXYZVYaw wp;

        if (t > 0) {
            wp.position.x = prev_x + cos(prev_heading) * linear_velocity * dt;
            wp.position.y = prev_y + sin(prev_heading) * linear_velocity * dt;
            wp.yaw = prev_heading + angular_velocity * dt;
            prev_x = wp.position.x;
            prev_y = wp.position.y;
            prev_heading = wp.yaw;
        }

        wp.velocity = linear_velocity;
        if (!use_heading) wp.yaw = yaw;

        trajectory.waypoints.push_back(wp);
    }
}

airstack_msgs::msg::TrajectoryXYZVYaw CurveTrajectory::get_trajectory() { return trajectory; }

//===================================================================================
//--------------------------------- Trajectory Library ------------------------------
//===================================================================================

TrajectoryLibrary::TrajectoryLibrary(std::string config_filename,
                                     rclcpp::Node::SharedPtr node_ptr) {  // tf2_ros::Buffer* b){

    RCLCPP_DEBUG(node_ptr->get_logger(), "TrajectoryLibrary constructor");
    RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
                        "Traj Library node_ptr name is: " << node_ptr->get_name());

    // accelertion trajectory parameters
    node_ptr->declare_parameter("dt", 69.);
    node_ptr->declare_parameter("ht", 69.);
    node_ptr->declare_parameter("ht_long", 69.);
    node_ptr->declare_parameter("max_velocity", 69.);
    node_ptr->declare_parameter("magnitude", 69.);

    this->node_ptr = node_ptr;
    if (buffer == NULL) {
        buffer = new tf2_ros::Buffer(node_ptr->get_clock());
        listener = new tf2_ros::TransformListener(*buffer);
    }

    YAML::Node yaml_config = YAML::LoadFile(config_filename);

    YAML::Node trajectories_yaml = yaml_config["trajectories"];
    for (size_t i = 0; i < trajectories_yaml.size(); i++) {
        YAML::Node traj_node = trajectories_yaml[i];
        std::string type = parse<std::string>(traj_node["type"]);

        if (type == "curve") {
            float linear_velocity = parse<double>(traj_node["linear_velocity"]);
            float angular_velocity = M_PI / 180. * parse<double>(traj_node["angular_velocity"]);
            std::string frame = parse<std::string>(traj_node["frame"]);
            float time = parse<double>(traj_node["time"]);
            float dt = parse<double>(traj_node["dt"]);
            bool use_heading = true;
            float yaw = 0.f;
            if (parse<std::string>(traj_node["yaw"]) == std::string("heading"))
                use_heading = true;
            else {
                use_heading = false;
                yaw = M_PI / 180. * parse<double>(traj_node["yaw"]);
            }

            CurveTrajectory* traj = new CurveTrajectory(linear_velocity, angular_velocity, frame,
                                                        time, dt, use_heading, yaw);

            static_trajectories.push_back(traj);
        } else if (type == "acceleration") {
            std::string frame = parse<std::string>(traj_node["frame"]);
            double x;
            double y;
            double z;

            if (traj_node["x"] && traj_node["y"] && traj_node["z"]) {
                x = parse<double>(traj_node["x"]);
                y = parse<double>(traj_node["y"]);
                z = parse<double>(traj_node["z"]);
            } else if (traj_node["magnitude"]) {
                double magnitude = parse<double>(traj_node["magnitude"]);
                double magnitude_yaw = M_PI / 180. * parse<double>(traj_node["magnitude_yaw"]);
                double magnitude_pitch = M_PI / 180. * parse<double>(traj_node["magnitude_pitch"]);

                tf2::Quaternion q;
                q.setRPY(0, magnitude_pitch, magnitude_yaw);
                tf2::Vector3 vector = tf2::Transform(q) * tf2::Vector3(magnitude, 0, 0);
                x = vector.x();
                y = vector.y();
                z = vector.z();
            } else {
                std::cout << "Couldn't construct acceleration trajectory. Need to have either x, "
                             "y, z or magnitude, magnitude_yaw, magnitude_pitch."
                          << std::endl;
                continue;
            }
            double dt = parse<double>(traj_node["dt"]);
            double ht = parse<double>(traj_node["ht"]);
            double max_velocity = parse<double>(traj_node["max_velocity"]);

            AccelerationTrajectory* traj =
                new AccelerationTrajectory(buffer, frame, x, y, z, dt, ht, max_velocity);

            dynamic_trajectories.push_back(traj);
        }
    }
    // RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
    //                     "dt: " << node_ptr->get_parameter("dt").as_double());
    // RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
    //                     "ht: " << node_ptr->get_parameter("ht").as_double());
    // RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
    //                     "ht_long: " << node_ptr->get_parameter("ht_long").as_double());
    // RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
    //                     "max_velocity: " << node_ptr->get_parameter("max_velocity").as_double());
    // RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
    //                     "magnitude: " << node_ptr->get_parameter("magnitude").as_double());
}

std::vector<Trajectory> TrajectoryLibrary::get_static_trajectories() {
    std::vector<Trajectory> trajectories;

    for (size_t i = 0; i < static_trajectories.size(); i++) {
        trajectories.push_back(
            Trajectory(node_ptr.get(), static_trajectories[i]->get_trajectory()));
    }

    return trajectories;
}

std::vector<Trajectory> TrajectoryLibrary::get_dynamic_trajectories(
    airstack_msgs::msg::Odometry odom) {
    std::vector<Trajectory> trajectories;

    for (size_t i = 0; i < dynamic_trajectories.size(); i++) {
        airstack_msgs::msg::TrajectoryXYZVYaw path = dynamic_trajectories[i]->get_trajectory(odom);
        if (path.waypoints.size() > 0) trajectories.push_back(Trajectory(node_ptr.get(), path));
    }

    return trajectories;
}
