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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(airstack_msgs REQUIRED)

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########

# Specify additional locations of header files

# Declare a C++ executable
add_executable(random_walk_planner src/random_walk_node.cpp)

# Link libraries
ament_target_dependencies(random_walk_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  airstack_msgs
)

# Install executable
install(TARGETS
random_walk_planner
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