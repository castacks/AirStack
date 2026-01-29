cmake_minimum_required(VERSION 3.5)
project(random_walk_planner)

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

###################################
## ament specific configuration  ##
###################################

ament_package()

###########
## Build ##
###########

# Specify additional locations of header files

# Declare a C++ executable
add_executable(random_walk_planner src/random_walk_node.cpp src/random_walk_logic.cpp)

# Link libraries
ament_target_dependencies(random_walk_planner
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# Install executable
install(TARGETS
  random_walk_planner
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
# ament_add_gtest(${PROJECT_NAME}-test test/test_random_walk_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_node)
# endif()