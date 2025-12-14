cmake_minimum_required(VERSION 3.8)
project(behavior_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ONNXRUNTIME_DIR "/home/robot/AirStack/robot/ros_ws/onnxruntime-linux-x64-1.23.2")
set(onnxruntime_DIR "/home/robot/AirStack/robot/ros_ws/onnxruntime-linux-x64-1.23.2")
include_directories("/home/robot/AirStack/robot/ros_ws/include")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(mavros_msgs REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(robot_interface REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

# find_package(onnxruntime REQUIRED) 

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)


add_executable(behavior_executive src/behavior_executive.cpp)
target_include_directories(behavior_executive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GeographicLib_INCLUDE_DIRS}
  "${ONNXRUNTIME_DIR}/include")

target_compile_features(behavior_executive PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(behavior_executive
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  nav_msgs
  behavior_tree
  behavior_tree_msgs
  airstack_msgs
  airstack_common
  mavros_msgs
  robot_interface
  pluginlib
  tf2
  GeographicLib
  geometry_msgs
  visualization_msgs
  cv_bridge
  OpenCV
  sensor_msgs
  Eigen3
  # onnxruntime
)

target_link_libraries(behavior_executive "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")

install(TARGETS behavior_executive
  DESTINATION lib/${PROJECT_NAME})

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
