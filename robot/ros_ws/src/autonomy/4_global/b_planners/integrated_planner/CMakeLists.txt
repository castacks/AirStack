cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})
project(integrated_planner)

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
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# set(OpenVDB_INCLUDE_DIR "/usr/local/include/openvdb" CACHE STRING "Path to OpenVDB installation")
find_package(OpenVDB REQUIRED)
find_package(vdb_edt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

# Airstack msg, for fake takeoff landing only
find_package(airstack_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########

# Specify additional locations of header files

# Declare a C++ executable
add_executable(integrated_planner 
  src/integrated_node_run.cpp 
  src/integrated_node.cpp 
  src/astar_vdb.cpp)

target_include_directories(integrated_planner
  PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(integrated_planner
  OpenVDB::openvdb
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES})

# Link libraries
ament_target_dependencies(integrated_planner
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  trajectory_msgs
  tf2
  tf2_ros
  pcl_conversions
  tf2_eigen
  tf2_geometry_msgs
  vdb_edt
)

add_executable(cpa_node 
  src/cpa_node.cpp 
  src/astar_vdb.cpp)

target_include_directories(cpa_node
  PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(cpa_node
  OpenVDB::openvdb
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES})

# Link libraries
ament_target_dependencies(cpa_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  trajectory_msgs
  tf2
  tf2_ros
  pcl_conversions
  tf2_eigen
  tf2_geometry_msgs
  vdb_edt
)

add_executable(takeoff_hack_node 
  src/takeoff_hack_node.cpp)

target_include_directories(takeoff_hack_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(takeoff_hack_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  airstack_msgs # Airstack takeoff only
  mavros_msgs # Airstack takeoff only
)

add_executable(odom_tf_node 
  src/model_ai_tfpub.cpp)

target_include_directories(odom_tf_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(odom_tf_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

add_executable(frame_fix_node 
  src/model_ai_framefix.cpp)

target_include_directories(frame_fix_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(frame_fix_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

add_executable(model_ai_px4_test 
  src/model_ai_px4_test.cpp)

target_include_directories(model_ai_px4_test
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(model_ai_px4_test
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# Install executable
install(TARGETS
  integrated_planner
  cpa_node
  takeoff_hack_node
  frame_fix_node
  model_ai_px4_test
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