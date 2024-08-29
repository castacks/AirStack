cmake_minimum_required(VERSION 3.5)
project(disparity_map_representation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(core_map_representation_interface REQUIRED)
find_package(disparity_graph REQUIRED)
find_package(tflib REQUIRED)
find_package(pluginlib REQUIRED)
find_package(airstack_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_msgs REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(disparity_graph)

include_directories(
  ${ament_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${core_map_representation_interface_INCLUDE_DIRS}
  ${disparity_graph_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}

  # ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(disparity_map_representation
  src/disparity_map_representation.cpp
)

target_link_libraries(disparity_map_representation
  disparity_graph
  ${pluginlib_LIBRARIES} # Ensure this variable is set correctly
)

ament_target_dependencies(disparity_map_representation
  rclcpp
  tf2
  tf2_ros
  disparity_graph
  core_map_representation_interface
  airstack_msgs
  cv_bridge
  tflib
  pluginlib
)

ament_export_targets(disparity_map_representationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp
  tf2
  tf2_ros
  disparity_graph
  core_map_representation_interface
  airstack_msgs
  cv_bridge
  tflib
  pluginlib
)

install(
  TARGETS disparity_map_representation
  EXPORT disparity_map_representationTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
target_link_libraries(disparity_map_representation
  ${disparity_graph_LIBRARIES}
)

# ament_export_include_directories(include)
# ament_export_libraries(disparity_map_representation)
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
