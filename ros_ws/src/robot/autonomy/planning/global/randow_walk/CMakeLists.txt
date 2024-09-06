cmake_minimum_required(VERSION 3.5)
project(random_walk_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(world_abstraction REQUIRED)
find_package(misc_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(core_trajectory_msgs REQUIRED)
find_package(base REQUIRED)

################################################
## Declare ROS messages, services, and actions ##
################################################

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RandomWalkPlannerCommand.msg"
  "msg/RandomWalkPlannerStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${base_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/random_walk_planner.cpp)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  world_abstraction
  misc_utils
  visualization_msgs
  core_trajectory_msgs
  base
)

# Install executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# ament_add_gtest(${PROJECT_NAME}-test test/test_random_walk_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_node)
# endif()

# Install launch files and other necessary files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)