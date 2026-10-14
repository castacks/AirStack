cmake_minimum_required(VERSION 3.16)
project(ia_tigris_search_planner LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(mtl_msgs REQUIRED)   # the search INTERFACE (action, plan, follower status) - no MTL planner code

# ---- ROS-free IA-TIGRIS core (std only): scenario, Dubins, belief/rewards, incremental tree search
#      with tree recycling, receding horizon
add_library(ia_tigris_core STATIC
  src/scenario.cpp
  src/dubins.cpp
  src/belief.cpp
  src/ia_tigris.cpp
  src/receding.cpp)
target_include_directories(ia_tigris_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(ia_tigris_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(ia_tigris_core PRIVATE -Wall -Wextra)

# ---- ROS 2 node
add_executable(ia_tigris_search_planner_node src/ia_tigris_search_planner_node.cpp)
target_link_libraries(ia_tigris_search_planner_node ia_tigris_core)
ament_target_dependencies(ia_tigris_search_planner_node
  rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs visualization_msgs airstack_msgs mtl_msgs)

# ---- ROS-free CLI (kinematic receding-horizon sortie; scripts/ia_tigris_offline_mission.py)
add_executable(ia_tigris_search_plan src/ia_tigris_search_plan_cli.cpp)
target_link_libraries(ia_tigris_search_plan ia_tigris_core)

install(TARGETS ia_tigris_search_planner_node ia_tigris_search_plan DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# The stdlib Python mirror of the two reward models is NOT duplicated here: the rewards are
# the same as TIGRIS's, and scripts/analyze_tigris_run.py uses tigris_search_planner.rewards.

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_core test/test_ia_tigris_core.cpp)
  if(TARGET ${PROJECT_NAME}_test_core)
    target_link_libraries(${PROJECT_NAME}_test_core ia_tigris_core)
  endif()
endif()

ament_package()
