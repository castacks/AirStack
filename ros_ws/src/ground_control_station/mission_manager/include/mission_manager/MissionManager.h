#ifndef MISSIONMANAGER_H_INCLUDED
#define MISSIONMANAGER_H_INCLUDED

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <cstdint>
#include <string>
#include <limits>
#include <random>
#include <queue>
#include <boost/functional/hash.hpp>

#include <CGAL/Polygon_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/point32.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "airstack_msgs/msg/search_mission_request.hpp"
#include "airstack_msgs/msg/task_assignment.hpp"
#include "mission_manager/BeliefMap.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
typedef CGAL::Alpha_shape_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;
typedef Alpha_shape_2::Edge Edge;
typedef CGAL::Segment_2<K> Segment;

struct ClusterPoint
{
    double x, y;
    int cluster;
    double cluster_dist;

    ClusterPoint(double _x, double _y, int _cluster_idx, double _cluster_dist)
    {
      x = _x;
      y = _y;
      cluster = _cluster_idx;
      cluster_dist = _cluster_dist;
    }
};

  struct CompareClusterPoint 
{
  bool operator()(const ClusterPoint& a, const ClusterPoint& b) const 
  {
    return a.cluster_dist > b.cluster_dist; // Min-heap based on cluster_dist
  }
};


class MissionManager
{
  public:
    MissionManager(int max_number_agents, double active_agent_check_n_seconds, double min_agent_altitude_to_be_active, double time_till_agent_not_valid);

    std::vector<airstack_msgs::msg::TaskAssignment> assign_tasks(rclcpp::Logger logger,
                                                                const airstack_msgs::msg::SearchMissionRequest &plan_request,
                                                                rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                                                                bool visualize_search_allocation,
                                                                double max_planning_time,
                                                                double budget,
                                                                double desired_speed);
    bool check_agent_changes(rclcpp::Logger logger, uint8_t robot_id, geometry_msgs::msg::Pose robot_pose, rclcpp::Time current_time);
    bool check_target_changes(rclcpp::Logger logger, std::string target_list, rclcpp::Time current_time);
    std::vector<bool> get_valid_agents() const { return valid_agents_; }
    BeliefMap belief_map_; // TODO make private

  private: 
    int max_number_agents_;
    rclcpp::Duration active_agent_check_n_seconds_;
    double min_agent_altitude_to_be_active_;
    rclcpp::Duration time_till_agent_not_valid_;
    std::vector<rclcpp::Time> time_of_last_call_;
    rclcpp::Time time_of_last_check_;
    std::vector<geometry_msgs::msg::Pose> agent_poses_;
    std::vector<bool> valid_agents_;
    int scenario_coutner_ = 0;

    //search map allocation
    std::vector<std::vector<std::vector<double>>> allocate_search_map(rclcpp::Logger logger,
                                                                      int number_of_search_tasks,
                                                                      const airstack_msgs::msg::SearchMissionRequest &plan_request,
                                                                      std::vector<std::vector<double>> &cluster_centroids,
                                                                      std::vector<std::vector<ClusterPoint>> &clusters);
    std::vector<std::vector<double>> calculate_cluster_centroids(rclcpp::Logger logger, int num_agents, const airstack_msgs::msg::SearchMissionRequest &plan_request, std::vector<Point> &CGAL_bounds) const;
    std::vector<std::vector<ClusterPoint>> calculate_clusters(rclcpp::Logger logger, int num_agents, std::vector<std::vector<double>> cluster_centroids, const std::vector<Point> &CGAL_bounds) const;
    std::vector<std::vector<std::vector<double>>> calculate_cluster_bounds(rclcpp::Logger logger, std::vector<std::vector<ClusterPoint>> clusters) const;

  };
  #endif /* MISSIONMANAGER_H_INCLUDED */