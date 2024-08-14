cmake_minimum_required(VERSION 3.5)
project(core_local_planner)

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  std_msgs
#  sensor_msgs
#  nav_msgs
#  core_trajectory_library
#  core_trajectory_controller
#  core_map_representation_interface
#  tflib
#  actionlib_msgs
#  behavior_tree
#  core_trajectory_msgs
#  pluginlib
#)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(core_trajectory_library REQUIRED)
find_package(core_trajectory_controller REQUIRED)
find_package(core_map_representation_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(core_trajectory_msgs REQUIRED)
find_package(pluginlib REQUIRED)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

find_package(base)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${core_trajectory_library_INCLUDE_DIRS}
  ${core_trajectory_controller_INCLUDE_DIRS}
  ${core_map_representation_interface_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  
)

add_executable(local_planner src/local_planner.cpp)
ament_target_dependencies(local_planner
rclcpp
rclpy
std_msgs
sensor_msgs
nav_msgs
core_trajectory_library
core_trajectory_controller
core_map_representation_interface
tf2_ros
action_msgs
core_trajectory_msgs
pluginlib
  # Add other dependencies here
)

##add_dependencies(local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(local_planner
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)

install(TARGETS local_planner
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
#install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
