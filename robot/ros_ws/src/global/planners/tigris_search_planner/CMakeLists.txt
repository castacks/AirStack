cmake_minimum_required(VERSION 3.16)
project(tigris_search_planner LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(mtl_msgs REQUIRED)   # the search INTERFACE (action, plan, follower status) - no MTL planner code

# ---- ROS-free TIGRIS core (std only): scenario, Dubins, belief/rewards, tree search, receding horizon
add_library(tigris_core STATIC
  src/scenario.cpp
  src/dubins.cpp
  src/belief.cpp
  src/tigris.cpp
  src/receding.cpp)
target_include_directories(tigris_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(tigris_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(tigris_core PRIVATE -Wall -Wextra)

# ---- ROS 2 node
add_executable(tigris_search_planner_node src/tigris_search_planner_node.cpp)
target_link_libraries(tigris_search_planner_node tigris_core)
ament_target_dependencies(tigris_search_planner_node
  rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs visualization_msgs airstack_msgs mtl_msgs)

# ---- ROS-free CLI (kinematic receding-horizon sortie; scripts/tigris_offline_mission.py)
add_executable(tigris_search_plan src/tigris_search_plan_cli.cpp)
target_link_libraries(tigris_search_plan tigris_core)

install(TARGETS tigris_search_planner_node tigris_search_plan DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# stdlib Python mirror of the two reward models (used by scripts/analyze_tigris_run.py)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_core test/test_tigris_core.cpp)
  if(TARGET ${PROJECT_NAME}_test_core)
    target_link_libraries(${PROJECT_NAME}_test_core tigris_core)
  endif()
  # Python tests (test/test_*.py) run through the root harness (tests/colcon_unit_test_packages.yaml).
endif()

ament_package()
