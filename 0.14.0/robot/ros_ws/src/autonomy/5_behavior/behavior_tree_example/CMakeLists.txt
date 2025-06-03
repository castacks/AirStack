cmake_minimum_required(VERSION 3.5)
project(behavior_tree_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(tf2 REQUIRED)

#add_executable(behavior_tree_example src/behavior_tree_example.cpp)
#ament_target_dependencies(talker rclcpp std_msgs)

add_executable(behavior_tree_example src/behavior_tree_example.cpp)
ament_target_dependencies(behavior_tree_example rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs behavior_tree_msgs behavior_tree tf2)

add_executable(robot_node src/robot_node.cpp)
ament_target_dependencies(robot_node rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs behavior_tree_msgs tf2)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  )

install(TARGETS
  behavior_tree_example
  robot_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
