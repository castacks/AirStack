cmake_minimum_required(VERSION 2.8.3)
project(core_disparity_map_representation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  stereo_msgs
  nav_msgs
  tf
  visualization_msgs
  cv_bridge
  core_map_representation_interface
  disparity_graph
  tflib
  pluginlib
  core_trajectory_msgs
)


###########
## Build ##
###########

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp tf disparity_graph core_map_representation_interface core_trajectory_msgs tflib
  LIBRARIES disparity_map_representation
)

include_directories(${catkin_INCLUDE_DIRS} include)


add_library(disparity_map_representation
  src/disparity_map_representation.cpp
  )

target_link_libraries(disparity_map_representation ${catkin_LIBRARIES})


install(TARGETS disparity_map_representation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

