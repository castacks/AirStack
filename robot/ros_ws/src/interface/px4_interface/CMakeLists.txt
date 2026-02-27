cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(robot_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ── Library ─────────────────────────────────────────────────────────────────
add_library(px4_interface src/px4_interface.cpp)

target_compile_features(px4_interface PUBLIC cxx_std_17)

target_include_directories(px4_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(px4_interface
  robot_interface
  pluginlib
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  mav_msgs
  tf2)

pluginlib_export_plugin_description_file(robot_interface plugins.xml)

# ── Install ──────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS px4_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ── Export ───────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(px4_interface)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
