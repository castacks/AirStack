cmake_minimum_required(VERSION 3.5)
project(disparity_graph_cost_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(airstack_common REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(cost_map_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(disparity_graph REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_library REQUIRED)


add_library(disparity_graph_cost_map src/disparity_graph_cost_map.cpp)
target_compile_features(disparity_graph_cost_map PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(disparity_graph_cost_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  disparity_graph_cost_map
  airstack_common
  airstack_msgs
  cost_map_interface
  cv_bridge
  disparity_graph
  geometry_msgs
  image_geometry
  image_transport
  nav_msgs
  pcl_msgs
  pluginlib
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  stereo_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  trajectory_library
  )

pluginlib_export_plugin_description_file(cost_map_interface disparity_graph_cost_map_plugin.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
#target_compile_definitions(disparity_graph_cost_map PRIVATE "disparity_graph_cost_map_INTERFACE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS disparity_graph_cost_map
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
  disparity_graph_cost_map
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
