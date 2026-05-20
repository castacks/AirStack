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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
