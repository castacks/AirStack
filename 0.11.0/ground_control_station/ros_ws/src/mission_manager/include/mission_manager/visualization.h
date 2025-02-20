#ifndef VISUALIZATIONS_MM_H_INCLUDED
#define VISUALIZATIONS_MM_H_INCLUDED

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mission_manager/MissionManager.h"


visualization_msgs::msg::MarkerArray visualize_multi_agent_search_request(int num_agents, 
                                                                    std::vector<std::vector<double>> cluster_centroids, 
                                                                    std::vector<std::vector<ClusterPoint>> clusters,
                                                                    std::vector<std::vector<std::vector<double>>> convex_hulls)
{
    visualization_msgs::msg::MarkerArray ma;
    
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 10.0;
    m.scale.y = 10.0;
    m.scale.z = 10.0;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;

    for(std::vector<double> centroid : cluster_centroids)
    {
        geometry_msgs::msg::Point new_point;
        new_point.x = centroid[0];
        new_point.y = centroid[1];
        new_point.z = 5.0;
        m.points.push_back(new_point);
    }
    ma.markers.push_back(m);

    //publish visualization for different regions
    std::vector<std::vector<double>> agent_colors = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for(int agent_idx = 0; agent_idx < num_agents; ++agent_idx)
    {
        visualization_msgs::msg::Marker cluster_m;
        cluster_m.header.frame_id = "map";
        cluster_m.id = agent_idx + 1;
        cluster_m.type = visualization_msgs::msg::Marker::CUBE_LIST;
        cluster_m.action = visualization_msgs::msg::Marker::ADD;
        cluster_m.pose.orientation.w = 1.0;
        cluster_m.scale.x = 1.0;
        cluster_m.scale.y = 1.0;
        cluster_m.scale.z = 1.0;
        cluster_m.color.a = 0.5;
        std::vector<double> agent_color = agent_colors[agent_idx];
        cluster_m.color.r = agent_color[0];
        cluster_m.color.g = agent_color[1];
        cluster_m.color.b = agent_color[2];

        for(ClusterPoint cp : clusters[agent_idx])
        {
            geometry_msgs::msg::Point new_point;
            new_point.x = cp.x;
            new_point.y = cp.y;
            new_point.z = 1.0;
            cluster_m.points.push_back(new_point);
        }
        ma.markers.push_back(cluster_m);
    }

    //publish visualization for the convex hull of each region
    for(auto agent_idx = 0u; agent_idx < convex_hulls.size(); ++agent_idx)
    {
        visualization_msgs::msg::Marker convex_hull_m;
        convex_hull_m.header.frame_id = "map";
        convex_hull_m.id = agent_idx + 100;
        convex_hull_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        convex_hull_m.action = visualization_msgs::msg::Marker::ADD;
        convex_hull_m.pose.orientation.w = 1.0;
        convex_hull_m.scale.x = 2.0;
        convex_hull_m.color.a = 1.0;
        std::vector<double> agent_color = agent_colors[agent_idx];
        convex_hull_m.color.r = agent_color[0];
        convex_hull_m.color.g = agent_color[1];
        convex_hull_m.color.b = agent_color[2];

        auto convex_hull = convex_hulls[agent_idx];
        for(auto point : convex_hull)
        {
            // RCLCPP_INFO_STREAM(logger, point[0] << " " << point[1]);
            geometry_msgs::msg::Point new_point;
            new_point.x = point[0];
            new_point.y = point[1];
            new_point.z = 1.0;
            convex_hull_m.points.push_back(new_point);
        }
        ma.markers.push_back(convex_hull_m);
    }
    return ma;
}
#endif /* VISUALIZATIONS_MM_H_INCLUDED */