cmake_minimum_required(VERSION 3.16)
project(ipp_baselines_search_planner LANGUAGES CXX)

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

# ---- ROS-free core (std only): scenario, Dubins, belief/rewards, the ia-tigris baselines
#      (greedy, rect_coverage, mcts_search), receding horizon
add_library(ipp_baselines_core STATIC
  src/scenario.cpp
  src/dubins.cpp
  src/belief.cpp
  src/planner_core.cpp
  src/greedy.cpp
  src/rect_coverage.cpp
  src/mcts_search.cpp
  src/planners.cpp
  src/receding.cpp
  src/params.cpp)
target_include_directories(ipp_baselines_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(ipp_baselines_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(ipp_baselines_core PRIVATE -Wall -Wextra)

# ---- ROS 2 node
add_executable(ipp_baselines_search_planner_node src/ipp_baselines_search_planner_node.cpp)
target_link_libraries(ipp_baselines_search_planner_node ipp_baselines_core)
ament_target_dependencies(ipp_baselines_search_planner_node
  rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs visualization_msgs airstack_msgs mtl_msgs)

# ---- ROS-free CLI (kinematic receding-horizon sortie; scripts/ipp_baselines_offline_mission.py)
add_executable(ipp_baselines_search_plan src/ipp_baselines_search_plan_cli.cpp)
target_link_libraries(ipp_baselines_search_plan ipp_baselines_core)

install(TARGETS ipp_baselines_search_planner_node ipp_baselines_search_plan DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_core test/test_ipp_baselines_core.cpp)
  if(TARGET ${PROJECT_NAME}_test_core)
    target_include_directories(${PROJECT_NAME}_test_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(${PROJECT_NAME}_test_core ipp_baselines_core)
  endif()
  ament_add_gtest(${PROJECT_NAME}_test_planners test/test_ipp_baselines_planners.cpp)
  if(TARGET ${PROJECT_NAME}_test_planners)
    target_include_directories(${PROJECT_NAME}_test_planners PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_link_libraries(${PROJECT_NAME}_test_planners ipp_baselines_core)
  endif()
endif()

ament_package()
