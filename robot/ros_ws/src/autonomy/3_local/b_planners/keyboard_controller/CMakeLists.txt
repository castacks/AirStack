cmake_minimum_required(VERSION 3.8)
project(keyboard_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(airstack_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# --- custom srv: KeyboardInput.srv ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KeyboardInput.srv"
)

# rosidl_default_runtime 을 외부에 export
ament_export_dependencies(rosidl_default_runtime)

# --- node executable ---
add_executable(keyboard_controller_node
  src/keyboard_controller_node.cpp
  src/keyboard_controller.cpp
)

ament_target_dependencies(keyboard_controller_node
  rclcpp
  std_msgs
  mav_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  airstack_msgs
  airstack_common

)

rosidl_target_interfaces(keyboard_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_include_directories(keyboard_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(keyboard_controller_node PUBLIC cxx_std_17)

# --- install ---
install(TARGETS
  keyboard_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()