#include <base/BaseNode.h>
#include <core_local_planner/local_planner.h>
#include <core_trajectory_controller/Trajectory.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
// #include <pointcloud_map_representation/pointcloud_map_representation.h>
#include <tflib/tflib.h>

//===================================================================================
//--------------------------------- Local Planner -----------------------------------
//===================================================================================

LocalPlanner::LocalPlanner(std::string node_name) : BaseNode(node_name) {}

bool LocalPlanner::initialize() {
    ros::NodeHandle* nh = get_node_handle();
    ros::NodeHandle* pnh = get_private_node_handle();

    // init subscribers
    global_plan_sub = nh->subscribe("global_plan", 10, &LocalPlanner::global_plan_callback, this);
    waypoint_sub = nh->subscribe("way_point", 10, &LocalPlanner::waypoint_callback, this);
    look_ahead_sub = nh->subscribe("look_ahead", 10, &LocalPlanner::look_ahead_callback, this);
    tracking_point_sub =
        nh->subscribe("tracking_point", 10, &LocalPlanner::tracking_point_callback, this);
    range_up_sub =
        nh->subscribe<sensor_msgs::Range>("range_up", 1, &LocalPlanner::range_up_callback, this);
    range_down_sub = nh->subscribe<sensor_msgs::Range>("range_down", 1,
                                                       &LocalPlanner::range_down_callback, this);
    custom_waypoint_sub = nh->subscribe<geometry_msgs::PoseStamped>(
        "custom_waypoint", 1, &LocalPlanner::custom_waypoint_callback, this);
    listener = new tf::TransformListener();

    // init publishers
    vis_pub = nh->advertise<visualization_msgs::MarkerArray>("trajectory_library_vis", 10);
    obst_vis_pub = nh->advertise<visualization_msgs::MarkerArray>("obstaccle_vis", 10);
    global_plan_vis_pub =
        nh->advertise<visualization_msgs::MarkerArray>("local_planner_global_plan_vis", 10);
    look_past_vis_pub = nh->advertise<visualization_msgs::MarkerArray>("look_past", 10);
    traj_pub = nh->advertise<core_trajectory_msgs::TrajectoryXYZVYaw>("trajectory", 10);
    traj_track_pub = nh->advertise<core_trajectory_msgs::TrajectoryXYZVYaw>("trajectory_track", 10);

    // init services
    traj_mode_client =
        nh->serviceClient<core_trajectory_controller::TrajectoryMode>("set_trajectory_mode");

    // init parameters
    waypoint_spacing = pnh->param("waypoint_spacing", 0.5);
    // obstacle_check_radius = pnh->param("obstacle_check_radius", 3.);
    // obstacle_check_points = pnh->param("obstacle_check_points", 3);
    obstacle_penalty_weight = pnh->param("obstacle_penalty_weight", 1.);
    forward_progress_penalty_weight = pnh->param("forward_progress_penalty_weight", 0.5);
    robot_radius = pnh->param("robot_radius", 0.75);
    look_past_distance = pnh->param("look_past_distance", 0);
    // use_fixed_height = pnh->param("use_fixed_height", false);
    height_mode = pnh->param("height_mode", 0);
    height_above_ground = pnh->param("height_above_ground", 1.);
    fixed_height = pnh->param("fixed_height", 1.);
    yaw_mode = pnh->param("yaw_mode", 0);
    map_representation =
        pnh->param("map_representation", std::string("PointCloudMapRepresentation"));

    waypoint_buffer_duration = pnh->param("waypoint_buffer_duration", 30.);
    waypoint_spacing_threshold = pnh->param("waypoint_spacing_threshold", 0.5);
    waypoint_angle_threshold = pnh->param("waypoint_angle_threshold", 30.) * M_PI / 180.;

    goal_mode = TRAJECTORY;
    custom_waypoint_timeout_factor = pnh->param("custom_waypoint_timeout_factor", 0.3);
    custom_waypoint_distance_threshold = pnh->param("custom_waypoint_distance_threshold", 0.5);

    std::string traj_lib_config_filename = pnh->param("trajectory_library_config", std::string(""));
    if (traj_lib_config_filename == "") {
        ROS_ERROR_STREAM("Trajectory library config file is invalid: " << traj_lib_config_filename);
        return false;
    }

    traj_lib = new TrajectoryLibrary(traj_lib_config_filename, listener);
    got_global_plan = false;
    global_plan_trajectory_distance = 0;
    got_look_ahead = false;
    got_tracking_point = false;
    got_range_up = false;
    got_range_down = false;

    pluginlib::ClassLoader<MapRepresentation> map_representation_loader(
        "core_map_representation_interface", "MapRepresentation");
    try {
        pc_map = map_representation_loader.createInstance(map_representation);
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("The MapRepresentation plugin failed to load. Error: %s", ex.what());
    }

    // init static trajectories and respace them
    static_trajectories = traj_lib->get_static_trajectories();
    for (int i = 0; i < static_trajectories.size(); i++)
        static_trajectories[i] = static_trajectories[i].respace(waypoint_spacing);

    return true;
}

bool LocalPlanner::execute() {
    update_waypoint_mode();

    if (!got_global_plan) return true;

    Trajectory gp(global_plan);
    // ROS_INFO_STREAM("initial gp waypoints size: " << gp.waypoint_count());
    // gp = gp.get_subtrajectory_distance(global_plan_trajectory_distance,
    //				       global_plan_trajectory_distance + 10.);

    // set the hieght of the global plan
    if (height_mode == FIXED_HEIGHT) {
        gp.set_fixed_height(fixed_height);
    } else if (height_mode == RANGE_SENSOR_HEIGHT) {
        if (!got_range_up || !got_range_down) return true;

        try {
            tf::StampedTransform transform_up, transform_down;
            listener->waitForTransform(gp.get_frame_id(), range_up.header.frame_id,
                                       range_up.header.stamp, ros::Duration(0.1));
            listener->lookupTransform(gp.get_frame_id(), range_up.header.frame_id,
                                      range_up.header.stamp, transform_up);
            listener->waitForTransform(gp.get_frame_id(), range_down.header.frame_id,
                                       range_down.header.stamp, ros::Duration(0.1));
            listener->lookupTransform(gp.get_frame_id(), range_down.header.frame_id,
                                      range_down.header.stamp, transform_down);

            tf::Vector3 range_up_gp_frame = transform_up * tf::Vector3(range_up.range, 0, 0);
            tf::Vector3 range_down_gp_frame = transform_down * tf::Vector3(range_down.range, 0, 0);

            /*
              if(gp.waypoint_count() > 0){
              double z_setpoint = gp.get_waypoint(0).z();
              z_setpoint = std::min(range_up_gp_frame.z() - 1.5*robot_radius,
              std::max(range_down_gp_frame.z() + 1.5*robot_radius, z_setpoint));
              gp.set_fixed_height(z_setpoint);
              }
            */
            //*
            double tunnel_height = range_up_gp_frame.z() - range_down_gp_frame.z();
            double z_setpoint = (range_up_gp_frame.z() + range_down_gp_frame.z()) / 2.;
            if (tunnel_height / 2. >= height_above_ground)
                z_setpoint = range_down_gp_frame.z() + height_above_ground;
            /*
              if(z_setpoint >= range_up_gp_frame.z() - robot_radius)
              z_setpoint = (range_up_gp_frame.z() + range_down_gp_frame.z())/2.;
            */

            gp.set_fixed_height(z_setpoint);
            //*/
        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM("Error transforming range data. " << ex.what());
        }
    }

    // transform the look ahead point to the global plan frame
    tf::Vector3 look_ahead_position = tflib::to_tf(look_ahead_odom.pose.pose.position);
    bool success =
        tflib::to_frame(listener, look_ahead_position, look_ahead_odom.header.frame_id,
                        gp.get_frame_id(), look_ahead_odom.header.stamp, &look_ahead_position);
    if (!success) {
        ROS_ERROR_STREAM("Couldn't transform from lookahead frame to global plan frame");
        return true;
    }

    // increment how far along the global plan we are
    double trajectory_distance;
    bool valid =
        gp.get_trajectory_distance_at_closest_point(look_ahead_position, &trajectory_distance);
    if (valid) {
        global_plan_trajectory_distance += trajectory_distance;
        gp = gp.get_subtrajectory_distance(trajectory_distance, trajectory_distance + 10.);
        // ROS_INFO_STREAM("after trim waypoints size: " << gp.waypoint_count());
    } else
        ROS_INFO("invalid");

    // publish the segment of the global plan currently being used, for visualization
    visualization_msgs::MarkerArray global_markers = gp.get_markers(0, 0, 1);
    global_plan_vis_pub.publish(global_markers);

    // get the dynamic trajectories
    std::vector<Trajectory> dynamic_trajectories =
        traj_lib->get_dynamic_trajectories(look_ahead_odom);

    // pick the best trajectory
    monitor.tic("get_best_trajectory");
    Trajectory best_traj;
    bool all_in_collision = get_best_trajectory(dynamic_trajectories, gp, &best_traj);
    monitor.toc("get_best_trajectory");

    // publish the trajectory
    // ROS_INFO_STREAM("all_in_collsion: " << all_in_collision);
    if (!all_in_collision) {
        core_trajectory_msgs::TrajectoryXYZVYaw path = best_traj.get_TrajectoryXYZVYaw();

        // set yaw
        if (yaw_mode == SMOOTH_YAW && path.waypoints.size() > 0) {
            bool found_initial_heading = false;
            double initial_heading = 0;
            try {
                tf::StampedTransform transform;
                listener->waitForTransform(best_traj.get_frame_id(),
                                           look_ahead_odom.header.frame_id,
                                           look_ahead_odom.header.stamp, ros::Duration(0.1));
                listener->lookupTransform(best_traj.get_frame_id(), look_ahead_odom.header.frame_id,
                                          look_ahead_odom.header.stamp, transform);

                transform.setOrigin(tf::Vector3(0, 0, 0));  // only care about rotation
                initial_heading =
                    tf::getYaw(transform * tflib::to_tf(look_ahead_odom.pose.pose.orientation));

                found_initial_heading = true;
            } catch (const tf::TransformException& ex) {
                ROS_ERROR_STREAM(
                    "Transform exception while finding yaw of lookahead: " << ex.what());
            }

            if (found_initial_heading) {
                path.waypoints[0].yaw = initial_heading;

                double alpha = 0.1;
                double sin_yaw_prev = sin(path.waypoints[0].yaw);
                double cos_yaw_prev = cos(path.waypoints[0].yaw);

                // ROS_INFO("-------------------------------------------------");
                // ROS_INFO_STREAM("heading: " <<
                // 180./M_PI*tf::getYaw(tflib::to_tf(look_ahead_odom.pose.pose.orientation)));
                for (int i = 1; i < path.waypoints.size(); i++) {
                    core_trajectory_msgs::WaypointXYZVYaw wp_prev = path.waypoints[i - 1];
                    core_trajectory_msgs::WaypointXYZVYaw& wp_curr = path.waypoints[i];

                    // ROS_INFO_STREAM(wp_curr.position.y - wp_prev.position.y << " " <<
                    // wp_curr.position.x - wp_prev.position.x);
                    double yaw = atan2(wp_curr.position.y - wp_prev.position.y,
                                       wp_curr.position.x - wp_prev.position.x);
                    double cos_yaw = alpha * cos(yaw) + (1 - alpha) * cos_yaw_prev;
                    double sin_yaw = alpha * sin(yaw) + (1 - alpha) * sin_yaw_prev;
                    yaw = atan2(sin_yaw, cos_yaw);

                    sin_yaw_prev = sin_yaw;
                    cos_yaw_prev = cos_yaw;

                    wp_curr.yaw = yaw;
                }
            }
        }

        path.header.stamp =
            ros::Time::now();  // TODO: make this match the time inside get_best_traj
        traj_pub.publish(path);
    }

    return true;
}

bool LocalPlanner::get_best_trajectory(std::vector<Trajectory> trajectories, Trajectory global_plan,
                                       Trajectory* best_traj) {
    // ROS_INFO("GET_BEST");
    double min_cost = std::numeric_limits<double>::max();
    int best_traj_index = 0;
    bool all_in_collision = true;

    ros::Time now = ros::Time::now();

    visualization_msgs::MarkerArray traj_lib_markers, look_past_markers;

    /*std::vector<core_trajectory_msgs::TrajectoryXYZVYaw> trajs;
    for(int i = 0; i < trajectories.size(); i++)
      trajs.push_back(trajectories[i].get_TrajectoryXYZVYaw());
    */
    std::vector<std::vector<geometry_msgs::PointStamped> > trajs;
    for (int i = 0; i < trajectories.size(); i++)
        trajs.push_back(trajectories[i].get_vector_PointStamped());
    std::vector<std::vector<double> > values = pc_map->get_values(trajs);

    for (int i = 0; i < trajectories.size(); i++) {
        Trajectory traj = trajectories[i];
        double average_distance = std::numeric_limits<double>::infinity();
        double closest_obstacle_distance = std::numeric_limits<double>::infinity();

        Trajectory global_plan_traj_frame;
        try {
            global_plan_traj_frame = global_plan.to_frame(traj.get_frame_id(), now);
        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM("TransformException in get_best_trajectory: " << ex.what());
            return true;
        }

        for (int j = 0; j < traj.waypoint_count(); j++) {
            // for(int j = traj.waypoint_count()-1; j < traj.waypoint_count(); j++){
            Waypoint wp = traj.get_waypoint(j);

            // find how far this waypoint is from an obstacle
            nav_msgs::Odometry odom = wp.odometry(now, traj.get_frame_id());
            geometry_msgs::PoseStamped pose;
            pose.header = odom.header;
            pose.pose = odom.pose.pose;
            closest_obstacle_distance = std::min(closest_obstacle_distance, values[i][j]);

            // find how far this waypoint is from the global plan path
            tf::Vector3 closest_point;
            int wp_index;
            double path_distance;
            bool valid = global_plan_traj_frame.get_closest_point(wp.position(), &closest_point,
                                                                  &wp_index, &path_distance);
            double forward_progress_penalty =
                -forward_progress_penalty_weight *
                path_distance;  //(global_plan_traj_frame.waypoint_count()-wp_index)*0.5;

            if (valid) {
                if (!std::isfinite(average_distance)) average_distance = 0;

                // average_distance += closest_point.distance(wp.position());
                average_distance +=
                    closest_point.distance(wp.position()) + forward_progress_penalty;
            }
        }
        // ROS_INFO_STREAM("CLOSEST: " << closest_obstacle_distance);

        average_distance /= traj.waypoint_count();
        bool collision = closest_obstacle_distance <= robot_radius;
        if (!collision) all_in_collision = false;

        visualization_msgs::MarkerArray traj_markers;
        if (collision)
            traj_markers = traj.get_markers(1, 0, 0, .5);
        else
            traj_markers = traj.get_markers(0, 1, 0, .5);

        if (look_past_distance > 0) {
            if (traj.waypoint_count() >= 2) {
                Waypoint curr_wp = traj.get_waypoint(traj.waypoint_count() - 1);
                Waypoint prev_wp = traj.get_waypoint(traj.waypoint_count() - 2);

                tf::Vector3 segment = curr_wp.position() - prev_wp.position();
                segment.normalize();

                tf::Vector3 position = curr_wp.position() + look_past_distance * segment;

                tf::Vector3 closest_point;
                int wp_index;
                bool valid =
                    global_plan_traj_frame.get_closest_point(position, &closest_point, &wp_index);

                if (!valid)
                    collision = true;
                else {
                    average_distance = position.distance(closest_point);
                }

                visualization_msgs::Marker marker;
                marker.header.stamp = now;
                marker.header.frame_id = traj.get_frame_id();
                marker.ns = "look_past";
                marker.id = i;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = position.x();
                marker.pose.position.y = position.y();
                marker.pose.position.z = position.z();
                marker.pose.orientation.w = 1;
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;
                look_past_markers.markers.push_back(marker);
            }
        }

        traj_lib_markers.markers.insert(traj_lib_markers.markers.end(),
                                        traj_markers.markers.begin(), traj_markers.markers.end());

        double cost =
            average_distance -
            obstacle_penalty_weight * std::min(closest_obstacle_distance, obstacle_check_radius);
        if (!collision && cost < min_cost) {
            min_cost = cost;
            best_traj_index = i;
            *best_traj = traj;
        }
    }

    // ROS_INFO_STREAM("Best traj index: " << best_traj_index);

    if (best_traj_index < look_past_markers.markers.size()) {
        look_past_markers.markers[best_traj_index].color.r = 0;
        look_past_markers.markers[best_traj_index].color.g = 1;
        look_past_markers.markers[best_traj_index].color.b = 0;
        look_past_markers.markers[best_traj_index].color.a = 1;
    }

    vis_pub.publish(traj_lib_markers);
    pc_map->publish_debug();
    look_past_vis_pub.publish(look_past_markers);

    return all_in_collision;
}

void LocalPlanner::global_plan_callback(core_trajectory_msgs::TrajectoryXYZVYaw global_plan) {
    ROS_INFO_STREAM("GOT GLOBAL PLAN, goal_mode: " << goal_mode);
    if (goal_mode != TRAJECTORY) return;

    this->global_plan = global_plan;
    got_global_plan = true;
    global_plan_trajectory_distance = 0;
}

void LocalPlanner::waypoint_callback(geometry_msgs::PointStamped wp) {
    if (goal_mode != AUTO_WAYPOINT) {
        waypoint_buffer.clear();
        waypoint_buffer.push_back(wp);
        return;
    }

    // TODO: REMOVE THIS
    if (wp.point.x == 0 && wp.point.y == 0) return;

    // remove old waypoints if necessary
    waypoint_buffer.push_back(wp);
    if ((wp.header.stamp - waypoint_buffer.front().header.stamp).toSec() > waypoint_buffer_duration)
        waypoint_buffer.pop_front();

    // stitch together the history of waypoints
    core_trajectory_msgs::TrajectoryXYZVYaw global_plan;
    global_plan.header.frame_id = wp.header.frame_id;
    global_plan.header.stamp = wp.header.stamp;

    std::vector<core_trajectory_msgs::WaypointXYZVYaw> backwards_global_plan;
    // double waypoint_spacing_threshold = 0.5;
    // double waypoint_angle_threshold = 30.*M_PI/180.;
    std::vector<geometry_msgs::PointStamped> prev_wps;
    for (auto it = waypoint_buffer.rbegin(); it != waypoint_buffer.rend(); it++) {
        geometry_msgs::PointStamped curr_wp = *it;
        core_trajectory_msgs::WaypointXYZVYaw waypoint;
        waypoint.position.x = curr_wp.point.x;
        waypoint.position.y = curr_wp.point.y;
        waypoint.position.z = curr_wp.point.z;

        if (prev_wps.size() > 1) {
            geometry_msgs::PointStamped prev_wp = prev_wps[prev_wps.size() - 1];
            geometry_msgs::PointStamped prev2_wp = prev_wps[prev_wps.size() - 2];
            float distance = sqrt(pow(curr_wp.point.x - prev_wp.point.x, 2) +
                                  pow(curr_wp.point.y - prev_wp.point.y, 2));

            float a = pow(prev_wp.point.x - curr_wp.point.x, 2) +
                      pow(prev_wp.point.y - curr_wp.point.y, 2);
            float b = pow(prev_wp.point.x - prev2_wp.point.x, 2) +
                      pow(prev_wp.point.y - prev2_wp.point.y, 2);
            float c = pow(prev2_wp.point.x - curr_wp.point.x, 2) +
                      pow(prev2_wp.point.y - curr_wp.point.y, 2);
            float angle = acos((a + b - c) / sqrt(4 * a * b));

            float angle_diff = fabs(atan2(sin(angle - M_PI), cos(angle - M_PI)));

            // ROS_INFO_STREAM("\tdistance: " << distance << " angle: " << angle*180./M_PI << "
            // angle_diff: " << angle_diff*180./M_PI);

            if (distance >= waypoint_spacing_threshold && angle_diff < waypoint_angle_threshold) {
                // ROS_INFO_STREAM("\tADDING wp: " << curr_wp.point.x << " " << curr_wp.point.y << "
                // " << curr_wp.point.z);
                backwards_global_plan.push_back(waypoint);
                prev_wps.push_back(curr_wp);
            }
        } else {
            if (prev_wps.size() == 0) {
                // ROS_INFO_STREAM("\tADDING 0 wp: " << curr_wp.point.x << " " << curr_wp.point.y <<
                // " " << curr_wp.point.z);
                prev_wps.push_back(curr_wp);
                backwards_global_plan.push_back(waypoint);
            } else if (prev_wps.size() == 1) {
                geometry_msgs::PointStamped prev_wp = prev_wps[prev_wps.size() - 1];
                float distance = sqrt(pow(curr_wp.point.x - prev_wp.point.x, 2) +
                                      pow(curr_wp.point.y - prev_wp.point.y, 2));
                if (distance >= waypoint_spacing_threshold) {
                    // ROS_INFO_STREAM("\tADDING 1 wp: " << curr_wp.point.x << " " <<
                    // curr_wp.point.y << " " << curr_wp.point.z);
                    prev_wps.push_back(curr_wp);
                    backwards_global_plan.push_back(waypoint);
                }
            }
        }
    }
    for (int i = backwards_global_plan.size() - 1; i >= 0; i--) {
        global_plan.waypoints.push_back(backwards_global_plan[i]);
    }
    /*for(auto it = waypoint_buffer.begin(); it != waypoint_buffer.end(); it++){
      geometry_msgs::PointStamped curr_wp = *it;
      core_trajectory_msgs::WaypointXYZVYaw waypoint;
      waypoint.position.x = curr_wp.point.x;
      waypoint.position.y = curr_wp.point.y;
      waypoint.position.z = curr_wp.point.z;
      global_plan.waypoints.push_back(waypoint);
    }
    */

    // add an extra segment onto the end of the path
    if (got_tracking_point) {
        try {
            tf::StampedTransform transform;
            listener->waitForTransform(tracking_point_odom.header.frame_id, wp.header.frame_id,
                                       wp.header.stamp, ros::Duration(0.1));
            listener->lookupTransform(tracking_point_odom.header.frame_id, wp.header.frame_id,
                                      wp.header.stamp, transform);

            tf::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.pose.position);
            tf::Vector3 wp_position = transform * tflib::to_tf(wp.point);

            tf::Vector3 direction = (wp_position - tp_position).normalized() * 3;
            tf::Vector3 wp2_position = wp_position + direction;

            // TODO: Are waypoints in the tracking point frame while the previous ones are in wp
            // frame?
            core_trajectory_msgs::WaypointXYZVYaw wp1, wp2;
            wp1.position.x = wp_position.x();
            wp1.position.y = wp_position.y();
            wp1.position.z = wp_position.z();
            wp2.position.x = wp2_position.x();
            wp2.position.y = wp2_position.y();
            wp2.position.z = wp2_position.z();
            global_plan.waypoints.push_back(wp1);
            global_plan.waypoints.push_back(wp2);
        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM("Transform exception in waypoint_callback: " << ex.what());
        }
    }

    global_plan_trajectory_distance = 0;
    this->global_plan = global_plan;
    this->got_global_plan = true;

    // ROS_INFO_STREAM("GLOBAL PLAN WAYPOINTS: " << global_plan.waypoints.size());

    // DEBUG ONLY REMOV THIS
    /*
    Trajectory gp(global_plan);
    visualization_msgs::MarkerArray global_markers = gp.get_markers(0, 0, 1);
    global_plan_vis_pub.publish(global_markers);
    */

    /*
    if(got_tracking_point){
      this->got_global_plan = true;
      global_plan_trajectory_distance = 0;

      try{
        tf::StampedTransform transform;
        listener->waitForTransform(tracking_point_odom.header.frame_id, wp.header.frame_id,
    wp.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(tracking_point_odom.header.frame_id, wp.header.frame_id,
    wp.header.stamp, transform);

        tf::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.pose.position);
        tf::Vector3 wp_position = transform*tflib::to_tf(wp.point);

        tf::Vector3 direction = (wp_position - tp_position).normalized()*3;
        tf::Vector3 wp2_position = wp_position + direction;

        core_trajectory_msgs::TrajectoryXYZVYaw global_plan;
        global_plan.header.frame_id = tracking_point_odom.header.frame_id;
        global_plan.header.stamp = wp.header.stamp;

        core_trajectory_msgs::WaypointXYZVYaw wp1, wp2;
        wp1.position.x = wp_position.x();
        wp1.position.y = wp_position.y();
        wp1.position.z = wp_position.z();
        wp2.position.x = wp2_position.x();
        wp2.position.y = wp2_position.y();
        wp2.position.z = wp2_position.z();
        global_plan.waypoints.push_back(wp1);
        global_plan.waypoints.push_back(wp2);

        this->global_plan = global_plan;
      }
      catch(tf::TransformException& ex){
        ROS_ERROR_STREAM("Transform exception in waypoint_callback: " << ex.what());
      }
    }
    */
}

void LocalPlanner::custom_waypoint_callback(geometry_msgs::PoseStamped wp) {
    if (!got_look_ahead) return;

    try {
        tf::StampedTransform transform;
        listener->waitForTransform(look_ahead_odom.header.frame_id, wp.header.frame_id,
                                   wp.header.stamp, ros::Duration(0.1));
        listener->lookupTransform(look_ahead_odom.header.frame_id, wp.header.frame_id,
                                  wp.header.stamp, transform);

        tf::Vector3 la_position = tflib::to_tf(look_ahead_odom.pose.pose.position);
        tf::Vector3 wp_position = transform * tflib::to_tf(wp.pose.position);

        core_trajectory_msgs::TrajectoryXYZVYaw global_plan;
        global_plan.header.frame_id = look_ahead_odom.header.frame_id;
        global_plan.header.stamp = ros::Time::now();

        core_trajectory_msgs::WaypointXYZVYaw wp1, wp2;
        wp1.position.x = la_position.x();  // la_position.x();
        wp1.position.y = la_position.y();  // la_position.y();
        wp1.position.z = la_position.z();  // la_position.z();
        wp2.position.x = wp_position.x();
        wp2.position.y = wp_position.y();
        wp2.position.z = wp_position.z();
        global_plan.waypoints.push_back(wp1);
        global_plan.waypoints.push_back(wp2);

        global_plan_trajectory_distance = 0;
        this->global_plan = global_plan;
        this->got_global_plan = true;

        goal_mode = CUSTOM_WAYPOINT;
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("TransformException in custom_waypoint_callback: " << ex.what());
    }
}

void LocalPlanner::update_waypoint_mode() {
    if (goal_mode == CUSTOM_WAYPOINT) {
        if (global_plan.waypoints.size() < 2) goal_mode = AUTO_WAYPOINT;

        // check if the time limit for reaching the waypoint has elapsed
        double elapsed_time = (ros::Time::now() - global_plan.header.stamp).toSec();
        double distance = 0;
        for (int i = 1; i < global_plan.waypoints.size(); i++) {
            core_trajectory_msgs::WaypointXYZVYaw prev_wp, curr_wp;
            prev_wp = global_plan.waypoints[i - 1];
            curr_wp = global_plan.waypoints[i];

            distance += sqrt(pow(prev_wp.position.x - curr_wp.position.x, 2) +
                             pow(prev_wp.position.y - curr_wp.position.y, 2));
        }

        // ROS_INFO_STREAM("elapsed: " << elapsed_time << " / " <<
        // distance/custom_waypoint_timeout_factor << " distance: " << distance);
        if (elapsed_time >= distance / custom_waypoint_timeout_factor) {
            // ROS_INFO_STREAM("CUSTOM WAYPOINT TIMEOUT REACHED");
            goal_mode = AUTO_WAYPOINT;
        }

        // check if we are close enough to the waypoint
        if (got_tracking_point) {
            try {
                tf::StampedTransform transform;
                listener->waitForTransform(tracking_point_odom.header.frame_id,
                                           global_plan.header.frame_id, global_plan.header.stamp,
                                           ros::Duration(0.1));
                listener->lookupTransform(tracking_point_odom.header.frame_id,
                                          global_plan.header.frame_id, global_plan.header.stamp,
                                          transform);

                tf::Vector3 tp_position = tflib::to_tf(tracking_point_odom.pose.pose.position);
                tp_position.setZ(0);
                tf::Vector3 wp_position =
                    transform * tflib::to_tf(global_plan.waypoints.back().position);
                wp_position.setZ(0);

                if (tp_position.distance(wp_position) < custom_waypoint_distance_threshold) {
                    // ROS_INFO_STREAM("CUSTOM WAYPOINT DISTANCE THRESHOLD MET");
                    goal_mode = AUTO_WAYPOINT;
                }
            } catch (tf::TransformException& ex) {
                ROS_ERROR_STREAM("TransformException in update_waypoint_mode: " << ex.what());
            }
        }
    }
}

void LocalPlanner::look_ahead_callback(nav_msgs::Odometry odom) {
    got_look_ahead = true;
    look_ahead_odom = odom;
}

void LocalPlanner::tracking_point_callback(nav_msgs::Odometry odom) {
    got_tracking_point = true;
    tracking_point_odom = odom;
}

void LocalPlanner::range_up_callback(sensor_msgs::Range range) {
    got_range_up = true;
    range_up = range;
}

void LocalPlanner::range_down_callback(sensor_msgs::Range range) {
    got_range_down = true;
    range_down = range;
}

LocalPlanner::~LocalPlanner() {}

BaseNode* BaseNode::get() {
    LocalPlanner* local_planner = new LocalPlanner("LocalPlanner");
    return local_planner;
}
