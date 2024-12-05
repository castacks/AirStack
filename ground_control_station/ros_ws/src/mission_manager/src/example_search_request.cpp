#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "airstack_msgs/msg/search_mission_request.hpp"
#include "airstack_msgs/msg/search_prior.hpp"
#include "airstack_msgs/msg/keep_out_zone.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<airstack_msgs::msg::SearchMissionRequest>("search_mission_request", 10);
			airstack_msgs::msg::SearchMissionRequest msg;
			msg.header.stamp = this->now();
			msg.header.frame_id = "map"; // TODO

			geometry_msgs::msg::Polygon search_bounds;
			geometry_msgs::msg::Point32 point;
			point.x = 0;
			point.y = 0;
			point.z = 0;
			search_bounds.points.push_back(point);
			point.x = 100;
			point.y = -28;
			point.z = 0;
			search_bounds.points.push_back(point);
			point.x = 230;
			point.y = 0;
			point.z = 0;
			search_bounds.points.push_back(point);
			point.x = 250;
			point.y = 100;
			point.z = 0;
			search_bounds.points.push_back(point);
			point.x = 130;
			point.y = 156;
			point.z = 0;
			search_bounds.points.push_back(point);
			point.x = 110;
			point.y = 191;
			point.z = 0;
			search_bounds.points.push_back(point);
			point.x = -40;
			point.y = 141;
			point.z = 0;
			search_bounds.points.push_back(point);
			msg.search_bounds = search_bounds;
			
			airstack_msgs::msg::SearchPrior search_prior;
			search_prior.value.push_back(0.5);
			search_prior.priority.push_back(10.0);
			search_prior.sensor_model_id.push_back(0);
			search_prior.grid_prior_type = 1;
			search_prior.header.frame_id = "local_enu";
			geometry_msgs::msg::Polygon grid_bounds;
			point.x = 0;
			point.y = 160;
			point.z = 0;
			grid_bounds.points.push_back(point);
			point.x = 12;
			point.y = 135;
			point.z = 0;
			grid_bounds.points.push_back(point);
			point.x = 92;
			point.y = 150;
			point.z = 0;
			grid_bounds.points.push_back(point);
			point.x = 80;
			point.y = 175;
			point.z = 0;
			grid_bounds.points.push_back(point);
			search_prior.points_list = grid_bounds;
			msg.search_priors.push_back(search_prior);

			airstack_msgs::msg::SearchPrior search_prior2;
			search_prior2.value.push_back(0.2);
			search_prior2.priority.push_back(1.0);
			search_prior2.sensor_model_id.push_back(0);
			search_prior2.grid_prior_type = 1;
			search_prior2.header.frame_id = "local_enu";
			geometry_msgs::msg::Polygon points_list;
			point.x = 0;
			point.y = 0;
			point.z = 0;
			points_list.points.push_back(point);
			point.x = 100;
			point.y = -28;
			point.z = 0;
			points_list.points.push_back(point);
			point.x = 230;
			point.y = 0;
			point.z = 0;
			points_list.points.push_back(point);
			point.x = 250;
			point.y = 100;
			point.z = 0;
			points_list.points.push_back(point);
			point.x = 130;
			point.y = 156;
			point.z = 0;
			points_list.points.push_back(point);
			point.x = 110;
			point.y = 191;
			point.z = 0;
			points_list.points.push_back(point);
			point.x = -40;
			point.y = 141;
			point.z = 0;
			points_list.points.push_back(point);
			search_prior2.points_list = points_list;
			msg.search_priors.push_back(search_prior2);



			publisher_->publish(msg);
			RCLCPP_INFO(this->get_logger(), "Publishing example search request");
			// Shutdown after publishing
			// rclcpp::shutdown();
    }

  private:
    rclcpp::Publisher<airstack_msgs::msg::SearchMissionRequest>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}


/*
# keep out zone
keep_out_zones:
  - 
    header: 
      frame_id: "local_enu"
    x: 0
    y: 139
    radius: 15
  - 
    header: 
      frame_id: "local_enu"
    x: 192
    y: -21
    radius: 10
  - 
    header: 
      frame_id: "local_enu"
    x: 210
    y: -10
    radius: 8
		
*/