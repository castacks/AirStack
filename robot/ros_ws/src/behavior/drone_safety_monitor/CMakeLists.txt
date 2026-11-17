cmake_minimum_required(VERSION 3.8)
project(drone_safety_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)

add_executable(drone_safety_monitor src/drone_safety_monitor.cpp)
target_include_directories(drone_safety_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(drone_safety_monitor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  drone_safety_monitor
  rclcpp
  std_msgs
  nav_msgs
  airstack_msgs
  airstack_common
)

install(TARGETS drone_safety_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME isolated_watchdog_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=197 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_watchdog.py)
  set_tests_properties(isolated_watchdog_contract PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
  add_test(NAME isolated_operator_preemption_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=197 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_operator_preemption.py
      --executable $<TARGET_FILE:drone_safety_monitor>)
  set_tests_properties(isolated_operator_preemption_contract
    PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
  add_test(NAME isolated_escalation_contract
    COMMAND ${CMAKE_COMMAND} -E env ROS_DOMAIN_ID=195 ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_escalation.py
      --executable $<TARGET_FILE:drone_safety_monitor>)
  set_tests_properties(isolated_escalation_contract
    PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
