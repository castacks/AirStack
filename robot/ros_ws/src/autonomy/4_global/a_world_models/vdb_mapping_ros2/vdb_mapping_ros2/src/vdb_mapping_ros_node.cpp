#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vdb_mapping/OccupancyVDBMapping.h"
#include "vdb_mapping_ros2/VDBMappingROS2.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "well hello there" << std::endl;
  std::shared_ptr<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping> > vdb_mapping =
    std::make_shared<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping> >();

  rclcpp::spin(vdb_mapping);

  rclcpp::shutdown();
  return 0;
}
