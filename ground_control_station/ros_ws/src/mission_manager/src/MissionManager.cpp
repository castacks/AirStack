#include "mission_manager/MissionManager.h"
#include "mission_manager/visualization.h"

/*
Empty Constructor
*/
MissionManager::MissionManager(int max_number_agents, double active_agent_check_n_seconds, double min_agent_altitude_to_be_active, double time_till_agent_not_valid) : 
  max_number_agents_(max_number_agents),
  active_agent_check_n_seconds_(rclcpp::Duration::from_seconds(active_agent_check_n_seconds)),
  min_agent_altitude_to_be_active_(min_agent_altitude_to_be_active),
  time_till_agent_not_valid_(rclcpp::Duration::from_seconds(time_till_agent_not_valid))
{
  rclcpp::Time default_time(0, 0, RCL_ROS_TIME);
  time_of_last_call_.resize(max_number_agents_, default_time);
  time_of_last_check_ = default_time;
  agent_poses_.resize(max_number_agents_);
  valid_agents_.resize(max_number_agents_, false);
}

bool MissionManager::check_agent_changes(rclcpp::Logger logger, uint8_t robot_id, geometry_msgs::msg::Pose robot_pose, rclcpp::Time current_time)
{
  RCLCPP_INFO(logger, "Checking agent changes");

  time_of_last_call_[robot_id] = current_time;
  agent_poses_[robot_id] = robot_pose;

  // Only check at the specified number of loops
  if (time_of_last_check_ - current_time < active_agent_check_n_seconds_)
  {
    return false;
  }
  time_of_last_check_ = current_time;

  // Check how many agents have reported in the last x seconds
  // If change in the agents reporting, reassign tasks
  std::vector<bool> curr_valid_agents{false, false, false, false, false};
  rclcpp::Duration time_till_agent_not_valid = rclcpp::Duration::from_seconds(10.0);
  for (uint8_t i = 0; i < max_number_agents_; i++)
  {
    if (current_time - time_of_last_call_[i] < time_till_agent_not_valid &&
        agent_poses_[i].position.z > min_agent_altitude_to_be_active_)
    {
      curr_valid_agents[i] = true;
    }
  }
  bool change_in_agents = false;
  if (curr_valid_agents != valid_agents_)
  {
    change_in_agents = true;
  }
  valid_agents_ = curr_valid_agents;

  return change_in_agents;
}

bool MissionManager::check_target_changes(rclcpp::Logger logger, std::string target_list, rclcpp::Time current_time)
{
  // TODO
  RCLCPP_INFO_STREAM(logger, "Checking target changes at time " 
    << current_time.nanoseconds() << " with target list " << target_list);
  return false;
}

std::vector<std::vector<double>> MissionManager::calculate_cluster_centroids(rclcpp::Logger logger, int num_agents, const airstack_msgs::msg::SearchMissionRequest &search_mission_request, std::vector<Point> &CGAL_bounds) const
{
  std::random_device dev;
  std::mt19937 rng(dev());
  for(auto& point : search_mission_request.search_bounds.points)
  {
    // add to vector of bounds
    CGAL_bounds.push_back(Point(point.x, point.y));
  }
  double min_x = this->belief_map_.get_min_x();
  double max_x = this->belief_map_.get_max_x();
  double min_y = this->belief_map_.get_min_y();
  double max_y = this->belief_map_.get_max_y();

  std::vector<std::vector<double>> boundary = {{min_x, min_y},{max_x, min_y},{max_x, max_y},{min_x, max_y}};

  //divide bounds for the search area based on number of agents
  double x_step_size = (max_x - min_x) / num_agents;
  double y_step_size = (max_y - min_y) / num_agents;
  
  std::vector<std::vector<double>> rand_centroids;
  if(num_agents == 3)
  {
    rand_centroids = {{20.0,60.0}, {142.0, 40.0}, {128.0, 119.0}};
  }

  //generate bounds for each subregion
  while(static_cast<int>(rand_centroids.size()) < num_agents) // Assumes rand centroids not crazy big
  {
    int i = rand_centroids.size();

    //calculate bounds for random sampling
    double min_x_sample = min_x + i * x_step_size;
    double min_y_sample = min_y + i * y_step_size;
    double max_x_sample = min_x + (i+1) * x_step_size;
    double max_y_sample = min_y + (i+1) * y_step_size;

    //create uniform distributions to sample from
    std::uniform_real_distribution<double> rand_x_dist(min_x_sample, max_x_sample);
    std::uniform_real_distribution<double> rand_y_dist(min_y_sample, max_y_sample);

    //generate random samples
    double rand_x = rand_x_dist(rng);
    double rand_y = rand_y_dist(rng);

    //check if this random sample is within bounds
    if (CGAL::bounded_side_2(CGAL_bounds.begin(),
                              CGAL_bounds.end(),
                              Point(rand_x, rand_y), K()) == CGAL::ON_BOUNDED_SIDE)
    {
      rand_centroids.push_back({rand_x, rand_y});
      RCLCPP_DEBUG_STREAM(logger, "Search mission centroid:" << rand_x << " " << rand_y);
    }
  }
  return rand_centroids;
}

double compute_dist(double x1, double y1, double x2, double y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

std::vector<std::vector<ClusterPoint>> MissionManager::calculate_clusters(rclcpp::Logger logger, int num_agents, std::vector<std::vector<double>> cluster_centroids, const std::vector<Point> &CGAL_bounds) const
{
  double min_x = this->belief_map_.get_min_x();
  double max_x = this->belief_map_.get_max_x();
  double min_y = this->belief_map_.get_min_y();
  double max_y = this->belief_map_.get_max_y();
  //calculate points in bounds
  std::priority_queue<ClusterPoint, std::vector<ClusterPoint>, CompareClusterPoint> cluster_pq;
  for(int x_idx = min_x; x_idx < max_x; ++x_idx)
    {
      for(int y_idx = min_y; y_idx < max_y; ++y_idx)
      {
        if (CGAL::bounded_side_2(CGAL_bounds.begin(),
                                  CGAL_bounds.end(),
                                  Point(x_idx, y_idx), K()) == CGAL::ON_BOUNDED_SIDE)
        {
          //find closest and farthest centroids to this point
          int closest_centroid_idx = -1;
          int farthest_centroid_idx = -1;
          double min_dist = std::numeric_limits<double>::infinity();
          double max_dist = -std::numeric_limits<double>::infinity();
          for(int centroid_idx = 0; centroid_idx < static_cast<int>(cluster_centroids.size()); ++centroid_idx) // Assumes rand centroids not crazy big
          {
            double dist = compute_dist(cluster_centroids[centroid_idx][0], cluster_centroids[centroid_idx][1], x_idx, y_idx);
            if(dist < min_dist)
            {
              min_dist = dist;
              closest_centroid_idx = centroid_idx;
            }
            if(dist > max_dist)
            {
              max_dist = dist;
              farthest_centroid_idx = centroid_idx;
            }
          }
          if (closest_centroid_idx == -1 || farthest_centroid_idx == -1)
          {
            RCLCPP_ERROR_STREAM(logger, "No closest or farthest centroid found for point " << x_idx << " " << y_idx);
          }
          cluster_pq.push(ClusterPoint(x_idx, y_idx, closest_centroid_idx, min_dist - max_dist));
        }
      }
    }
  //actual do the clustering now
  std::vector<bool> cluster_full;
  std::vector<std::vector<ClusterPoint>> clusters;
  int cluster_size = cluster_pq.size() / num_agents;
  for(int i = 0; i < num_agents; ++i)
  {
    clusters.push_back({});
    cluster_full.push_back(false);
  }
  while(true)
  {
    ClusterPoint point = cluster_pq.top();
    // RCLCPP_INFO_STREAM(logger, point.cluster_dist << " " << point.x << " " << point.y);
    cluster_pq.pop();

    //check the size of preferred cluster
    if(static_cast<int>(clusters[point.cluster].size()) < cluster_size) // Assumes clusters not crazy big
    {
      clusters[point.cluster].push_back(point);
    }
    else if(!cluster_full[point.cluster])
    {
      //pop all remaining elements off priority queue and resort
      std::vector<ClusterPoint> remaining_points;
      while(!cluster_pq.empty())
      {
        ClusterPoint point = cluster_pq.top();
        remaining_points.push_back(point);
        cluster_pq.pop();
      }
      //iterate through vector and resort
      for(ClusterPoint cp : remaining_points)
      {
        //find closest and farthest centroids to this point
        int closest_centroid_idx = -1;
        int farthest_centroid_idx = -1;
        double min_dist = std::numeric_limits<double>::infinity();
        double max_dist = -std::numeric_limits<double>::infinity();
        for(int centroid_idx = 0; centroid_idx < static_cast<int>(cluster_centroids.size()); ++centroid_idx) // Assumes not crazy number of centroids
        {
          if(centroid_idx != point.cluster and !cluster_full[centroid_idx]) //check that it's not the existing full centroid OR a previous full centroid
          {
            double dist = compute_dist(cluster_centroids[centroid_idx][0], cluster_centroids[centroid_idx][1], cp.x, cp.y);
            if(dist < min_dist)
            {
              min_dist = dist;
              closest_centroid_idx = centroid_idx;
            }
            if(dist > max_dist)
            {
              max_dist = dist;
              farthest_centroid_idx = centroid_idx;
            }
          }
        }
        if (closest_centroid_idx == -1 || farthest_centroid_idx == -1)
        {
          RCLCPP_ERROR_STREAM(logger, "V2: No closest or farthest centroid found for point " << cp.x << " " << cp.y);
        }
        cluster_pq.push(ClusterPoint(cp.x, cp.y, closest_centroid_idx, min_dist - max_dist));
      }
      //set cluster as full so we don't redo this procedure till another cluster is full
      cluster_full[point.cluster] = true;
      // RCLCPP_INFO_STREAM(logger, "Cluster " << point.cluster << " full");
    }
    // RCLCPP_INFO_STREAM(logger, cluster_pq.size());
    if(cluster_pq.empty())
    {
      break;
    }
  }
  return clusters;
}

std::vector<std::vector<std::vector<double>>> MissionManager::calculate_cluster_bounds(rclcpp::Logger logger, std::vector<std::vector<ClusterPoint>> clusters) const
{
  std::vector<std::vector<std::vector<double>>> alpha_regions;
  std::unordered_map<std::pair<double,double>, Segment, boost::hash<std::pair<double, double>>> boundary_edges;
  if (clusters.size() == 0)
  {
    RCLCPP_ERROR(logger, "No clusters found");
  }
  for(std::vector<ClusterPoint> cluster : clusters)
  {
    //convert points inside polygon to cgal points
    std::vector<Point> cgal_points;
    for(ClusterPoint point : cluster)
    {
      cgal_points.push_back(Point(point.x, point.y));
    }
    //calculate alpha shape formed by the polygon points
    double alpha = 0.5;
    Alpha_shape_2 A(cgal_points.begin(), cgal_points.end(), alpha, Alpha_shape_2::REGULARIZED);
    //extract the boundary edges for the alpha shape
    for (auto it = A.alpha_shape_edges_begin(); it != A.alpha_shape_edges_end(); ++it) 
    {
      if (A.classify(*it) == Alpha_shape_2::REGULAR)
      {
        auto segment = A.segment(*it);
        // Get the source and target points of the segment
        Point source = segment.source();
        // Point target = segment.target();
        //put into our hashmap
        boundary_edges[std::make_pair(source.x(), source.y())] = segment;
      }
    }
    //grab the first key in the dictionary and set it's value as our start segment
    std::pair<double, double> start_key;
    auto it = boundary_edges.begin();
    if (it != boundary_edges.end()) 
    {
      start_key = it->first;
    }
    Segment start_seg = boundary_edges[start_key];
    Segment curr_seg = start_seg;

    //iterate through the segments and form the polygon
    std::vector<std::vector<double>> region;
    while(true)
    {
      //we want to find the target of our current segment as the source for a new segment
      Point curr_seg_target = curr_seg.target();
      region.push_back({curr_seg_target.x(), curr_seg_target.y()});

      //check if the current node target equals the start node source (we completed the loop)
      if(curr_seg_target == start_seg.source())
      {
        break;
      }

      std::pair<double, double> curr_key = std::make_pair(curr_seg_target.x(), curr_seg_target.y());
      if(boundary_edges.find(curr_key) != boundary_edges.end())
      {
        Segment next_seg = boundary_edges[curr_key];

        // RCLCPP_INFO_STREAM(logger, "(" << curr_seg.source().x() << "," << curr_seg.source().y() 
        //                         << ") (" << curr_seg.target().x() << "," << curr_seg.target().y() 
        //                         << ") -> (" << next_seg.source().x() << "," << next_seg.source().y()
        //                         << ") (" << next_seg.target().x() << "," << next_seg.target().y() << ")");
        
        curr_seg = next_seg;
      }
    }
    alpha_regions.push_back(region);
  }
  return alpha_regions;
}

std::vector<std::vector<std::vector<double>>> MissionManager::allocate_search_map(rclcpp::Logger logger,
                                                                      int number_of_search_tasks,
                                                                      const airstack_msgs::msg::SearchMissionRequest &search_mission_request,
                                                                      std::vector<std::vector<double>> &cluster_centroids,
                                                                      std::vector<std::vector<ClusterPoint>> &clusters)
{
  std::vector<Point> CGAL_bounds;
  //calculate random centroids
  cluster_centroids = calculate_cluster_centroids(logger, number_of_search_tasks, search_mission_request, CGAL_bounds);
  //calculate the clusters
  clusters = calculate_clusters(logger, number_of_search_tasks, cluster_centroids, CGAL_bounds);
  //calculate boundaries of clusters formed
  return calculate_cluster_bounds(logger, clusters);
}

std::vector<airstack_msgs::msg::TaskAssignment> MissionManager::assign_tasks(
  rclcpp::Logger logger,
  const airstack_msgs::msg::SearchMissionRequest &search_mission_request,
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
  bool visualize_search_allocation,
  double max_planning_time,
  double budget,
  double desired_speed)
{
  RCLCPP_INFO(logger, "Assigning tasks to drones");

  // Find how many active robots
  // int num_active_agents = std::accumulate(valid_agents_.begin(), valid_agents_.end(), 0);
  int num_active_agents = 3; //MAGIC NUMBER. TODO: UPDATE FOR LINE ABOVE WHEN REST OF SYSTEM COMPLETE

  // Decide how many search vs track tasks to assign
  int number_of_track_tasks = 0; // TODO
  int number_of_search_tasks = std::max(num_active_agents - number_of_track_tasks, 0);
  RCLCPP_INFO_STREAM(logger, "Assigning " << number_of_search_tasks << " search tasks and " << number_of_track_tasks << " track tasks");
  if (number_of_search_tasks > 3)
  {
    RCLCPP_ERROR(logger, "Too many search tasks requested for the current implementation");
    return {};
  }
  if (number_of_search_tasks == 0)
  {
    RCLCPP_WARN(logger, "No activate agents available for search tasks");
    return {};
  }

  // Send out the request for the search map division
  std::vector<std::vector<double>> cluster_centroids;
  std::vector<std::vector<ClusterPoint>> clusters;
  std::vector<std::vector<std::vector<double>>> cluster_bounds = allocate_search_map(logger, number_of_search_tasks, search_mission_request, cluster_centroids, clusters);
  
  //convert to task assignment
  std::vector<airstack_msgs::msg::TaskAssignment> task_assignments(max_number_agents_);
  // TODO: should allocate both search and track. Right now assumes all search tasks
  for(int i = 0; i < max_number_agents_; ++i)
  {
    if (!valid_agents_[i])
    {
      continue;
    }
    task_assignments[i].assigned_task_type = airstack_msgs::msg::TaskAssignment::SEARCH;
    task_assignments[i].assigned_task_number = i;
    task_assignments[i].plan_request.start_pose = agent_poses_[i];
    task_assignments[i].plan_request.max_planning_time = max_planning_time;
    task_assignments[i].plan_request.maximum_range = budget;

    task_assignments[i].plan_request.desired_speed = desired_speed;
    task_assignments[i].plan_request.search_bounds = search_mission_request.search_bounds;

    task_assignments[i].plan_request.clear_tree = true;
    task_assignments[i].plan_request.scenario = scenario_coutner_;


    grid_map::Polygon search_prior_polygon;
    search_prior_polygon.setFrameId(this->belief_map_.map_.getFrameId());
    for(std::vector<double> coord : cluster_bounds[i])
    {
      search_prior_polygon.addVertex(grid_map::Position(coord[0], coord[1])); // TODO coordinate frame
      geometry_msgs::msg::Point32 new_point;
      new_point.x = coord[0];
      new_point.y = coord[1];
      new_point.z = 0.0;
      task_assignments[i].plan_request.search_bounds.points.push_back(new_point);
    }

    // Copy over the cell probabilities and priorities into new task
    airstack_msgs::msg::SearchPrior task_search_prior;
    geometry_msgs::msg::Point32 point_prior;
    grid_map::Matrix& probability_data = this->belief_map_.map_["probability"];
    grid_map::Matrix& priority_data = this->belief_map_.map_["priority"];
    for (grid_map::PolygonIterator iterator(this->belief_map_.map_, search_prior_polygon);
      !iterator.isPastEnd(); ++iterator)
    {
      const grid_map::Index index(*iterator);
      grid_map::Position position;
      this->belief_map_.map_.getPosition(*iterator, position);
      point_prior.x = position.x();
      point_prior.y = position.y();
      point_prior.z = 0.0;
      task_search_prior.points_list.points.push_back(point_prior);
      task_search_prior.value.push_back(probability_data(index(0), index(1)));
      task_search_prior.priority.push_back(priority_data(index(0), index(1)));
    }
    task_assignments[i].plan_request.search_priors.push_back(task_search_prior);
  }

  //publish visualizations for search request
  if (visualize_search_allocation)
  {
    pub->publish(visualize_multi_agent_search_request(num_active_agents, cluster_centroids, clusters, cluster_bounds));
  }

  scenario_coutner_++;
  return task_assignments;
}