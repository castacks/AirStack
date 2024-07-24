cmake_minimum_required(VERSION 2.8.3)
project(core_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  core_trajectory_library
  core_trajectory_controller
  core_map_representation_interface
  tflib
  actionlib_msgs
  behavior_tree
  core_trajectory_msgs
  pluginlib
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

find_package(base)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(local_planner src/local_planner.cpp)

add_dependencies(local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(local_planner
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
