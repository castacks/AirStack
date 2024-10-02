#include "mission_manager/BeliefMap.h"


BeliefMap::BeliefMap()
: map_(std::vector<std::string>({"probability"}))
{ }

bool BeliefMap::reset_map(rclcpp::Logger logger, airstack_msgs::msg::SearchMissionRequest search_mission_request)
{
  // Setting up map.
  // Get the max and min x and y values from the search area
  double resolution = 10.0;
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
  map_.setFrameId("search_map"); // TODO update to correct frame or set TF frame somewhere. There is already a map frame
  map_.clearAll();

  for (auto& search_prior : search_mission_request.search_priors)
  {
    if (search_prior.grid_prior_type == airstack_msgs::msg::SearchPrior::POLYGON_PRIOR)
    {
      grid_map::Polygon polygon;
      polygon.setFrameId(map_.getFrameId());
      for (const auto& point : search_prior.points_list.points)
      {
        polygon.addVertex(grid_map::Position(point.x, point.y)); // TODO coordinate frame
      }
      for (grid_map::PolygonIterator iterator(map_, polygon);
        !iterator.isPastEnd(); ++iterator)
      {
        map_.at("probability", *iterator) = search_prior.value;
      }
    }
    else if (search_prior.grid_prior_type == airstack_msgs::msg::SearchPrior::LINE_SEG_PRIOR)
    {
      // TODO
    }
    else if (search_prior.grid_prior_type == airstack_msgs::msg::SearchPrior::POINT_PRIOR)
    {
      // TODO
    }
    else
    {
      RCLCPP_ERROR(logger, "Unknown grid prior type");
    }
  }
  

  return true;
}

// TODO subscribe to the shared search areas for updates. 