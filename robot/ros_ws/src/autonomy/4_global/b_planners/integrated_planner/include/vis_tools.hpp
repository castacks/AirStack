#ifndef VIS_TOOLS_H
#define VIS_TOOLS_H

#include <utility>
#include <vector>
#include <openvdb/openvdb.h>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

inline visualization_msgs::msg::MarkerArray path_to_marker_array(const std::vector<openvdb::Vec3d> &path_world, double alpha = 1.0)
{
    visualization_msgs::msg::MarkerArray arr;
    if (path_world.empty())
        return arr;

    visualization_msgs::msg::Marker spheres;
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::msg::Marker::ADD;
    spheres.ns = "astar_path";
    spheres.id = 0;
    spheres.pose.orientation.w = 1.0;
    spheres.scale.x = 0.1;
    spheres.scale.y = 0.1;
    spheres.scale.z = 0.1;
    spheres.color.r = 1.0f;
    spheres.color.g = 0.2f;
    spheres.color.b = 0.2f;
    spheres.color.a = alpha;

    visualization_msgs::msg::Marker line;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.ns = "astar_path";
    line.id = 1;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.05;
    line.color.r = 0.2f;
    line.color.g = 0.8f;
    line.color.b = 1.0f;
    line.color.a = alpha;

    spheres.points.reserve(path_world.size());
    line.points.reserve(path_world.size());

    for (const auto &w : path_world)
    {
        geometry_msgs::msg::Point p;
        p.x = w.x();
        p.y = w.y();
        p.z = w.z();
        spheres.points.push_back(p);
        line.points.push_back(p);
    }

    arr.markers.push_back(std::move(spheres));
    arr.markers.push_back(std::move(line));
    return arr;
}

#endif