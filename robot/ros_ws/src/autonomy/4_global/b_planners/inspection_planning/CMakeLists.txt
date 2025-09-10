cmake_minimum_required(VERSION 3.5)
project(inspection_planning)

# Default to C++17
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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vdb_mapping REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########

# Declare a C++ executable
add_executable(inspection_planner 
  src/inspection_planner_node.cpp
  src/inspection_planner.cpp)

target_include_directories(inspection_planner
  PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(inspection_planner
  vdb_mapping::vdb_mapping
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES})

# Link libraries
ament_target_dependencies(inspection_planner
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  tf2_eigen
  tf2_geometry_msgs
  message_filters
)

# Install executable
install(TARGETS
  inspection_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  include
  DESTINATION share/${PROJECT_NAME})
