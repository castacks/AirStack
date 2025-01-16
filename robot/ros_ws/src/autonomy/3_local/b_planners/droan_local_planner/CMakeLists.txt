cmake_minimum_required(VERSION 3.8)
project(droan_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(airstack_common REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cost_map_interface REQUIRED)
# find_package(disparity_graph_cost_map REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_controller REQUIRED)
find_package(trajectory_library REQUIRED)

add_executable(droan_local_planner src/droan_local_planner.cpp)
target_include_directories(droan_local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(droan_local_planner PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(
  droan_local_planner
  "airstack_msgs"
  "airstack_common"
  "nav_msgs"
  "cost_map_interface"
  # "disparity_graph_cost_map"
  "trajectory_controller"
  "trajectory_library"
  "pluginlib"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
)

install(TARGETS droan_local_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
