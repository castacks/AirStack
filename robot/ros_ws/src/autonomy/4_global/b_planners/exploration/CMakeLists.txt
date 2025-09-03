cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})
project(exploration_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# set(OpenVDB_INCLUDE_DIR "/usr/local/include/openvdb" CACHE STRING "Path to OpenVDB installation")
find_package(OpenVDB REQUIRED)
find_package(vdb_mapping REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########

# Specify additional locations of header files

# Declare a C++ executable
add_executable(exploration_planner 
  src/exploration_node_run.cpp 
  src/exploration_node.cpp 
  src/exploration_logic.cpp
  src/viewpoint_sampling.cpp
  src/collision_checker.cpp
  src/rrt_planner.cpp
  src/utils.cpp)

target_include_directories(exploration_planner
  PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(exploration_planner
  vdb_mapping::vdb_mapping
  OpenVDB::openvdb
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(waypoint_routing_planner 
  src/waypoint_routing_node.cpp
  src/exploration_node.cpp
  src/exploration_logic.cpp
  src/viewpoint_sampling.cpp
  src/collision_checker.cpp
  src/rrt_planner.cpp
  src/utils.cpp)

target_include_directories(waypoint_routing_planner
  PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(waypoint_routing_planner
  vdb_mapping::vdb_mapping
  OpenVDB::openvdb
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES})

# Link libraries
ament_target_dependencies(exploration_planner
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  tf2_eigen
  tf2_geometry_msgs
)

ament_target_dependencies(waypoint_routing_planner
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  tf2_eigen
  tf2_geometry_msgs
)

# Install executable
install(TARGETS
  exploration_planner
  waypoint_routing_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME})


#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# ament_add_gtest(${PROJECT_NAME}-test test/test_exploration_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_node)
# endif()