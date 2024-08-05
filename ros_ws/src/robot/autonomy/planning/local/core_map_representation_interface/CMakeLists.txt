cmake_minimum_required(VERSION 2.8.3)
project(core_map_representation_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  pluginlib
  core_trajectory_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES core_map_representation_interface
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf visualization_msgs core_trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
