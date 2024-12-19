
#include "mission_manager/mission_manager_node.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
