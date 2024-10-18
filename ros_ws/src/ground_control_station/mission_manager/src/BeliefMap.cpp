#include "mission_manager/BeliefMap.h"


BeliefMap::BeliefMap()
: map_(std::vector<std::string>({"probability"}))
{ }

bool BeliefMap::reset_map(rclcpp::Logger logger, airstack_msgs::msg::SearchMissionRequest search_mission_request)
{
  RCLCPP_INFO(logger, "Resetting map");
  // Setting up map.
  // Get the max and min x and y values from the search area
  double resolution = 5.0;
  double min_x = DBL_MAX;
  double max_x = -DBL_MAX;
  double min_y = DBL_MAX;
  double max_y = -DBL_MAX;
  for (const auto& point : search_mission_request.search_bounds.points)
  {
    if (point.x < min_x)
    {
      min_x = point.x;
    }
    if (point.x > max_x)
    {
      max_x = point.x;
    }
    if (point.y < min_y)
    {
      min_y = point.y;
    }
    if (point.y > max_y)
    {
      max_y = point.y;
    }
  }
  // TODO param for grid resolution
  // TODO fix coordinate frames. Match airstack to IPP to grid map. grid map is NWU, IPP is ENU, airstack is ...
  // round to nearest resolution
  min_x = std::floor(min_x / resolution) * resolution;
  max_x = std::ceil(max_x / resolution) * resolution;
  min_y = std::floor(min_y / resolution) * resolution;
  max_y = std::ceil(max_y / resolution) * resolution;
  double x_length = std::abs(max_x - min_x);
  double y_length = std::abs(max_y - min_y);
  
  map_.setGeometry(grid_map::Length(x_length, y_length),
                   resolution,
                   grid_map::Position(std::round(min_x + x_length / 2.0),
                                      std::round(min_y + y_length / 2.0)));
  map_.setFrameId("map"); // TODO update to correct frame or set TF frame somewhere. There is already a map frame
  map_.clearAll();

  for (auto& search_prior : search_mission_request.search_priors)
  {
    if (search_prior.grid_prior_type == airstack_msgs::msg::SearchPrior::POLYGON_PRIOR)
    {
      if (search_prior.value.size() > 1)
      {
        RCLCPP_ERROR(logger, "Polygon priors with multiple values not valid");
        continue;
      }
      grid_map::Polygon polygon;
      polygon.setFrameId(map_.getFrameId());
      for (const auto& point : search_prior.points_list.points)
      {
        polygon.addVertex(grid_map::Position(point.x, point.y)); // TODO coordinate frame
      }
      for (grid_map::PolygonIterator iterator(map_, polygon);
        !iterator.isPastEnd(); ++iterator)
      {
        float& current_value = map_.at("probability", *iterator);
        if (std::isnan(current_value) || current_value < search_prior.value[0]) {
          current_value = search_prior.value[0];
        }
      }
    }
    else if (search_prior.grid_prior_type == airstack_msgs::msg::SearchPrior::LINE_SEG_PRIOR)
    {
      RCLCPP_ERROR(logger, "Line segment priors not implemented");
    }
    else if (search_prior.grid_prior_type == airstack_msgs::msg::SearchPrior::POINT_PRIOR)
    {
      RCLCPP_ERROR(logger, "Point priors not implemented");
    }
    else
    {
      RCLCPP_ERROR(logger, "Unknown grid prior type");
    }
  }
  

  return true;
}

bool BeliefMap::update_map(rclcpp::Logger logger, const airstack_msgs::msg::BeliefMapData::SharedPtr new_belief_data)
{
  if (new_belief_data->x_start >= new_belief_data->x_end || 
      new_belief_data->y_start >= new_belief_data->y_end)
  {
    RCLCPP_ERROR(logger, "Received belief map data with origin issues");
    RCLCPP_WARN_STREAM(logger, "x_start: " << new_belief_data->x_start << " x_end: " << new_belief_data->x_end);
    RCLCPP_WARN_STREAM(logger, "y_start: " << new_belief_data->y_start << " y_end: " << new_belief_data->y_end);
    return false;
  }

  grid_map::Index submapStartIndex(new_belief_data->x_start, new_belief_data->y_start); // check coordinate frame!
  grid_map::Index submapBufferSize(new_belief_data->x_end - new_belief_data->x_start, 
                         new_belief_data->y_end - new_belief_data->y_start); // check coordinate frame!
  int cur_index_num = 0;
  grid_map::Matrix& data = map_["probability"];
  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator)
  {
    // map_.at("probability", *iterator) = 1.0;
    const grid_map::Index index(*iterator);
    // always save with the lower value
    if (new_belief_data->map_values[cur_index_num] < data(index(0), index(1)))
    {
      // Scale the value back from 0-UIN16_MAX to 0-1
      data(index(0), index(1)) = static_cast<float>(new_belief_data->map_values[cur_index_num]) / static_cast<float>(UINT16_MAX); // TODO ensure maps are encoded correctly to match the iterator
    }
    cur_index_num++;
  }
  return true;
}
