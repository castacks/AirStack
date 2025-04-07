cmake_minimum_required(VERSION 3.8)
project(takeoff_landing_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(trajectory_library REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(takeoff_landing_planner src/takeoff_landing_planner.cpp)
target_include_directories(takeoff_landing_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(takeoff_landing_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  takeoff_landing_planner
  std_msgs
  airstack_msgs
  airstack_common
  trajectory_library
  mavros_msgs
  std_srvs
)

install(TARGETS takeoff_landing_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME})

# Isolated Integration Testing from (https://arnebaeyens.com/blog/2024/ros2-integration-testing/)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)

  add_ros_isolated_launch_test(
    test/scripts/test_takeoff_landing_planner.py
  )
endif()

ament_package()
