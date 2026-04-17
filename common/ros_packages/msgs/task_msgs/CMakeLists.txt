cmake_minimum_required(VERSION 3.8)
project(task_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CoverageTask.action"
  "action/ExplorationTask.action"
  "action/FixedTrajectoryTask.action"
  "action/LandTask.action"
  "action/NavigateTask.action"
  "action/SemanticSearchTask.action"
  "action/TakeoffTask.action"
  DEPENDENCIES action_msgs geometry_msgs nav_msgs airstack_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
