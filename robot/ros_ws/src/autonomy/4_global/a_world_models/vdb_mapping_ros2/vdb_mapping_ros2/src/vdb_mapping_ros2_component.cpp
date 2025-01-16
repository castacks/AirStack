#include "rclcpp/rclcpp.hpp"

#include "vdb_mapping/OccupancyVDBMapping.h"
#include "vdb_mapping_ros2/VDBMappingROS2.hpp"

namespace vdb_mapping_ros2 {
using vdb_mapping_ros2_component = VDBMappingROS2<vdb_mapping::OccupancyVDBMapping>;
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vdb_mapping_ros2::vdb_mapping_ros2_component)
