cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(robot_interface REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)

add_library(px4_interface src/px4_interface.cpp)
target_compile_features(px4_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(px4_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  px4_interface
  robot_interface
  mavros_msgs
  pluginlib
  tf2
)

pluginlib_export_plugin_description_file(robot_interface plugins.xml)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(px4_interface PRIVATE "PX4_INTERFACE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS px4_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  px4_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
