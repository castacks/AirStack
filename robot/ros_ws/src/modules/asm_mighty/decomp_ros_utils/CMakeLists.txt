cmake_minimum_required(VERSION 3.8)
project(decomp_ros_utils)

find_package(ament_cmake REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Header-only: install data_ros_utils.hpp under the decomp_rviz_plugins/
# path expected by upstream mighty includes.
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(decomp_util decomp_ros_msgs sensor_msgs nav_msgs pcl_conversions)
ament_package()
