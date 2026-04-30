cmake_minimum_required(VERSION 3.8)
project(coverage_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(task_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)

add_executable(coverage_planner
  src/coverage_planner_node.cpp
  src/coverage_planner_logic.cpp
)
target_include_directories(coverage_planner PRIVATE include)

ament_target_dependencies(coverage_planner
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  task_msgs
  airstack_msgs
)

install(TARGETS coverage_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
