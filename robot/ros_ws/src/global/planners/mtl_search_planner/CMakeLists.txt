cmake_minimum_required(VERSION 3.16)
project(mtl_search_planner LANGUAGES CXX)

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
find_package(mtl_msgs REQUIRED)

# ---- vendored cpp_planner (mtl::planner), Eigen only ------------------------
include(cmake/mtl_vendored.cmake)

# ---- ROS-free adapter: scenario JSON <-> planner, map-frame tracks ------------
add_library(mtl_search_core STATIC src/search_problem.cpp)
target_include_directories(mtl_search_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(mtl_search_core PUBLIC mtl_planner_vendored)
set_target_properties(mtl_search_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(mtl_search_core PRIVATE -Wall -Wextra)

# ---- ROS 2 node ---------------------------------------------------------------
add_executable(mtl_search_planner_node src/mtl_search_planner_node.cpp)
target_link_libraries(mtl_search_planner_node mtl_search_core)
ament_target_dependencies(mtl_search_planner_node
  rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs visualization_msgs airstack_msgs mtl_msgs)

# ---- ROS-free CLI (tuning / offline mission) ----------------------------------
add_executable(mtl_search_plan src/mtl_search_plan_cli.cpp)
target_link_libraries(mtl_search_plan mtl_search_core)

install(TARGETS mtl_search_planner_node mtl_search_plan DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Pure-Python scenario module (stdlib only): mtl_search_planner.scenario — also
# imported by path from the Isaac scene and scripts/ (no install needed there).
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_search_problem test/test_search_problem.cpp)
  if(TARGET ${PROJECT_NAME}_test_search_problem)
    target_link_libraries(${PROJECT_NAME}_test_search_problem mtl_search_core)
  endif()
  # Upstream cpp_planner self-tests (dubins, kmeans, orienteering, geometry, pipeline).
  mtl_vendored_add_selftests()
  # Python tests (test/test_*.py) run through the root harness
  # (`airstack test -m unit`, tests/colcon_unit_test_packages.yaml).
endif()

ament_package()
