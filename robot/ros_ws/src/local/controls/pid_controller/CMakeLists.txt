cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pid_controller_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller PRIVATE include)
target_compile_features(pid_controller PRIVATE cxx_std_17)
ament_target_dependencies(pid_controller
  rclcpp
  std_msgs
  mav_msgs
  nav_msgs
  pid_controller_msgs
  airstack_msgs
  airstack_common
  tf2
  tf2_ros)

install(TARGETS
  pid_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_state test/test_control_state.cpp)
  target_include_directories(test_control_state PRIVATE include)
  target_compile_features(test_control_state PRIVATE cxx_std_17)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME isolated_lifecycle
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=197 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_lifecycle.py
      $<TARGET_FILE:pid_controller> ${CMAKE_CURRENT_SOURCE_DIR}/config/pid_controller.yaml)
  set_tests_properties(isolated_lifecycle PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
  add_test(NAME isolated_clock_order
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=197 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_clock_order.py
      $<TARGET_FILE:pid_controller> ${CMAKE_CURRENT_SOURCE_DIR}/config/pid_controller.yaml)
  set_tests_properties(isolated_clock_order PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
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
